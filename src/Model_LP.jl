function solve_LP(grb_env, data; threads = 1)

    TS = Model(optimizer_with_attributes(() -> Gurobi.Optimizer(grb_env), "Threads" => 16,  "OutputFlag" => 1))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses])
    @variable(TS, power_flow_var[data.lines])

    JuMP.fix(theta[data.buses[1]], 0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_costs[g] * generation[g] * data.base_mva for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n] / data.base_mva)

    #Voltage law
    @constraint(TS, voltage_1[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) == power_flow_var[l])

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity[g] / data.base_mva)

    #Angle limits
    @constraint(TS, theta_limit1[n = data.buses], theta[n] <= THETAMAX)
    @constraint(TS, theta_limit2[n = data.buses], theta[n] >= THETAMIN)

    #Line limit
    zeta1 = @constraint(TS, power_flow_limit_1[l in data.lines], power_flow_var[l] <= data.line_capacity[l] / data.base_mva)
    zeta2 = @constraint(TS, power_flow_limit_2[l in data.lines], power_flow_var[l] >= -data.line_capacity[l] / data.base_mva)

    optimize!(TS)
    grb_model = backend(TS).optimizer.model.inner
    time = Gurobi.get_runtime(grb_model)
    status = termination_status(TS)
    objective = 0.0
    solution = [0.0]
    dual_indicators = [false]
    duals = [0.0]

    if status == MOI.TerminationStatusCode(1)

        @inline function expand(arr::Array{Float64, 1}, idctr::Array{Bool, 1})
            output = Vector{Float64}()
            ctr = 1
            for i in 1:length(idctr)
                if idctr[ctr]
                    push!(output, arr[ctr])
                    ctr += 1
                else
                    push!(output, 0.0)
                end
            end
            return output
        end

        objective = objective_value(TS)

        #grb_model = backend(TS).optimizer.model.inner
        #nv = Gurobi.get_intattr(grb_model, "NumVars")
        @inline function convertArray(type::Type, array::T where T <: Array{Bool, 1})

            out = Vector{type}()
            for i in 1:length(array)
                if array[i]
                    push!(out, 1.0)
                else
                    push!(out, 0.0)
                end
            end
            return out
        end

        # @inline function get_solution_vector()
        #     output = [value.(generation).data...,
        #               convertArray(Float64, line_vector)...,
        #               value.(theta).data...,
        #               expand(value.(power_flow_var).data, line_vector)...]
        #     return output
        # end
        #
        # @inline function get_dual_indicators()
        #
        #     _dual_indicators = Vector{Bool}()
        #     d1 = dual.(zeta1).data
        #     d2 = dual.(zeta2).data
        #
        #     for i in 1:length(line_vector)
        #         if line_vector[i]
        #             indctr = d1[data.lines[i]] != 0.0 || d2[data.lines[i]] != 0.0 #|| dual.(beta).data[data.lines[i]] != 0.0
        #             push!(_dual_indicators, indctr)
        #         else
        #             push!(_dual_indicators, false)
        #         end
        #     end
        #
        #     return _dual_indicators
        # end

    end

    print(value.(TS[:generation]).data)

    return time, status, objective, value.(TS[:power_flow_var]).data, value.(TS[:theta]).data, value.(TS[:generation]).data
end
