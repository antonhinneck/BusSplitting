logger = Logger(0, Vector{Float64}(), Vector{Float64}(), Vector{Float64}(), Vector{Float64}())
last = -1.0

function solve_DCOPF_OBSP2SOS(data; x0 = nothing, split = nothing)

    M = 10000
    M2 = 2 * THETAMAX

    TS = Model()
    set_optimizer(TS, with_optimizer(Gurobi.Optimizer, TimeLimit = 600, Cuts = 3))

    buses = Vector{Int64}()
    for b in data.buses
        #if data.bus_type[b] != 1
            push!(buses, b)
        #end
    end

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[n = buses])
    @variable(TS, power_flow_var[data.lines])
    @variable(TS, switched[l = data.lines; data.line_is_aux[l]], Bin)

    if split != nothing
        for l in 1:split #length(data.lines)
            if data.line_is_aux[l]
                JuMP.fix(switched[l], x0[l]; force = true)
            end
        end
    end

    for l in 1:length(data.lines)#length(data.lines)
        if data.line_is_proxy[l]
            JuMP.fix(switched[l], 1.0; force = true)
        end
    end

    ## Generate SOS indicators
    ##------------------------
    @inline function _SOSindicators()
        SOSindicators = Dict{Int64, Vector{Float64}}()
        for b in data.buses
            if (data.bus_type[b] == 3) || (data.bus_type[b] == 4) || (data.bus_type[b] == 5)
                push!(SOSindicators, b => Vector{Float64}())
                for l in data.lines_at_bus[b]
                    if data.line_is_aux[l]
                        push!(SOSindicators[b], l)
                    end
                end
            end
        end
        return SOSindicators
    end

    SOSindicators = _SOSindicators()
    SOSkeys = [k for k in keys(SOSindicators)]

    #Set angle at slack node
    #JuMP.fix(theta[data.buses[1]], 0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_c1[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    #@constraint(TS, cut[l = data.lines; data.line_is_proxy[l]], switched[l] == 1)
    #Voltage law
    # @constraint(TS, voltage_1[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) + (1 - switched[l]) * M >= power_flow_var[l])
    #
    # @constraint(TS, voltage_2[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) <= power_flow_var[l] + (1 - switched[l]) * M)

    @constraint(TS, voltage[l = data.lines; !data.line_is_aux[l]],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) == power_flow_var[l])

    # @constraint(TS, connectors[b = buses; (data.bus_type[b] == 3) || (data.bus_type[b] == 4) || (data.bus_type[b] == 5)],
    # sum(switched[l] for l in data.lines_at_bus[b] if data.line_is_aux[l]) <= 1)

    # @constraint(TS, connectors[b = buses; (data.bus_type[b] == 3) || (data.bus_type[b] == 4) || (data.bus_type[b] == 5)],
    # [switched[l] for l in data.lines_at_bus[b] if data.line_is_aux[l]] in MathOptInterface.SOS1())

    @constraint(TS, connectors[b = SOSkeys], [switched[Int64(l)] for l in SOSindicators[b]] in MathOptInterface.SOS1(SOSindicators[b]))

    # @constraint(TS, root_lines[b = data.buses; data.bus_is_root[b] && data.bus_decomposed[b]],
    # sum(switched[l] for l in data.lines_at_bus[b] if data.line_is_aux[l]) >= 1)

    @constraint(TS, theta1[l = data.lines; data.line_is_aux[l]],
    theta[data.line_start[l]] <= theta[data.line_end[l]] + (1 - switched[l]) * M2)
    #
    @constraint(TS, theta2[l = data.lines; data.line_is_aux[l]],
    theta[data.line_start[l]] + (1 - switched[l]) * M2 >= theta[data.line_end[l]])

    # @constraint(TS, theta3[l = data.lines; data.line_is_aux[l]],
    # switched[l] => { theta[data.line_start[l]] == theta[data.line_end[l]] })

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity_max[g])

    #Angle limits
    # @constraint(TS, theta_limit_1[n = buses], theta[n] <= THETAMAX)
    # @constraint(TS, theta_limit_2[n = buses], theta[n] >= THETAMIN)

    # #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines; data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_2[l in data.lines; data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_3[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l])
    @constraint(TS, power_flow_limit_4[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l])

    MOIU.attach_optimizer(TS)
    grb_model = backend(TS).optimizer.model

    cb_wrapper_c = @cfunction(function cb_wrapper(
        model_pointer::Ptr{Cvoid}, cbdata::Ptr{Cvoid}, 
        where::Cint, 
        user_data_pointer::Ptr{Cvoid})
            # Saving progress
            #----------------
            if where == convert(Cint, Gurobi.GRB_CB_MIPNODE);
                time = Ref{Cdouble}()
                Gurobi.GRBcbget(cbdata, where, GRB_CB_RUNTIME, time)
                if time[] - last > 1
                    inc = Ref{Cdouble}()
                    lb = Ref{Cdouble}()
                    Gurobi.GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBST, inc)
                    Gurobi.GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBND, lb)
                    src = 0.0
                    save(logger, [inc[], lb[], time[], src])
                    global last = time[]
                end
            end
            return Cint(0)
        end,
        Cint, (Ptr{Cvoid}, Ptr{Cvoid}, Cint, Ptr{Cvoid}))
    Gurobi.GRBsetcallbackfunc(grb_model, cb_wrapper_c, C_NULL)

    if x0 != nothing
        for l in data.lines
            if data.line_is_aux[l]
                grb_idx = TS.moi_backend.model_to_optimizer_map[index(switched[l])].value
                Gurobi.GRBsetdblattrelement(grb_model, "Start", grb_idx, x0[l])
            end
        end
    end

    Gurobi.GRBupdatemodel(grb_model)
    #optimize!(TS)

    Gurobi.GRBoptimize(grb_model)
    println("------------------")
    println(termination_status(TS))

    # println(Gurobi.GRBcomputeIIS(grb_model))
    # Gurobi.GRBwrite(grb_model, "model.ilp")
    # Gurobi.GRBwrite(grb_model, "model.lp")
    # Gurobi.GRBwrite(grb_model, "model.mps")
    #map_optimizer_constraints(TS)[14]
    #print(index(TS[:market_clearing][14]))
    #print(optimizer_index(TS[:power_flow_limit_1][]))

    # grb_model.write("model.ilp")

    # lineStateDict = Dict{Int64, Float64}()
    #
    # for l in data.lines
    #     if data.line_is_aux[l]
    #         push!(lineStateDict, l => value(TS[:switched][l]))
    #     end
    # end

    return objective_value(TS), value.(TS[:power_flow_var]).data, value.(TS[:theta]).data, value.(TS[:generation]).data, logger, value.(TS[:switched])
end
