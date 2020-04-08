function heuristic_presolve(t)

    sleep(20)
    print("\n", time() - t, "\n")
    print("----------------------------")
    return true
end

function solve_TS_MILP(data)

    #print(data.lines,"\n")
    #print(data.generators,"\n")
    #print(data.line_reactance,"\n")

    # Parameters
    #--------------

    M = 10000

    TS = Model(with_optimizer(Gurobi.Optimizer, TimeLimit = 15, Threads = 3))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses] >= 0)
    @variable(TS, power_flow_var[data.lines])
    @variable(TS, switched[data.lines], Bin)

    #JuMP.fix(switched[1], 1; force = true)
    #JuMP.fix(switched[2], 1; force = true)
    #JuMP.fix(switched[3], 1; force = true)

    #Set angle at slack node
    #JuMP.fix(theta[data.busses[1]], pi; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_costs[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    #Voltage law
    @constraint(TS, voltage_1[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) + (1 - switched[l]) * M >= power_flow_var[l])

    @constraint(TS, voltage_2[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) <= power_flow_var[l] + (1 - switched[l]) * M)

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity[g])

    #Angle limits
    @constraint(TS, theta_limit_1[n = data.buses], theta[n] <= pi)

    #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines], power_flow_var[l] <= data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_2[l in data.lines], power_flow_var[l] >= -data.line_capacity[l] * switched[l])

    output = Vector{Float64}()
    solution = Vector{Float64}()
    solution_passed = true
    started = false
    solutions_produced = false
    t = 0
    @inline function pscc_callback(cbdata::CallbackData, where::Cint)
        # CALLBACK FUNCTION - HOOKS INTO GUROBI
        # PSCC2020 CONTRIBUTION
        #----------------------
        # SOLVER STATES:
        #---------------
            #const CB_POLLING = 0
            #const CB_PRESOLVE = 1
            #const CB_SIMPLEX = 2
            #const CB_MIP = 3
            #const CB_MIPSOL = 4
            #const CB_MIPNODE = 5
            #const CB_MESSAGE = 6
            #const CB_BARRIER = 7

        # READS VARIABLES AFTER PRESOLVE AND INJECTS INITIAL GUESS
        #---------------------------------------------------------
        #vars_after_presolve::S where S <: Any
        if solutions_produced

            print("\n", time(), "\n")
            solutions_produced = false

        end

        if where == convert(Cint, Gurobi.CB_PRESOLVE)

            @async solutions_produced = heuristic_presolve(time())

            if !started
                status, vars = solve_TS_LP_CB(data)
                for l in 1:length(data.lines)
                    push!(vars, 1)
                end
                started = true
                solution = vars
                solution_passed = false
                #current_rel = cbget_mipnode_rel(cbdata, convert(Int32, 2))[1]
                #push!(output, current_rel)
            end
        end

        if where == convert(Cint, Gurobi.CB_MIPNODE)
            if !solution_passed
                Gurobi.cbsolution(cbdata, solution)
            end
        end
    end

    t = time()
    MOIU.attach_optimizer(TS)

    heuristic = true
    if heuristic
        Gurobi.set_callback_func!(backend(TS).optimizer.model.inner, pscc_callback)
    end

    grb_model = backend(TS).optimizer.model.inner
    optimize!(TS)
    #Gurobi.optimize(backend(TS).optimizer.model.inner)
    #print(Gurobi.get_dblattrarray(grb_model, "X", 1, num_vars(grb_model)))
    #status = termination_status(TS)

    #if status == MOI.TerminationStatusCode(2)
        #output = [0,0]
        #get_iis(TS)
    #else

    #status = optimize!(TS, with_optimizer(GLPK.Optimizer, msg_lev = 3))
    #status = optimize!(TS, with_optimizer(Cbc.Optimizer, LogLevel = 1))
    te = time()

    #Returing Results
    #----------------
    #print("Time:\n")
    #print(te-t,"\n")
    #print("Costs:\n")
    #print(JuMP.objective_value(TS),"\n")
    #print("\nGeneration:\n")
    #print(value.(generation).data)
    #print("\n",adjacency)
    #print("\n",getvalue(theta))
    #print("\nPowerFlow:\n")
    #print("\n",value.(power_flow_var).data)
        #print("\nSwitched:\n")
        #print("\n",value.(switched).data)
    #print("\nTheta:\n")
    #print(value.(theta).data)
        print("MLIP - Time: ", te-t,", Objective Value: ")#,JuMP.objective_value(TS),".\n")
        output = [te-t]
    #end

    return output, value.(switched).data
end
