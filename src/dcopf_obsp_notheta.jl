function solve_DCOPF_OBSP(data; x0 = nothing, split = nothing)

    M = 10000
    M2 = 2 * THETAMAX

    TS = Model()
    set_optimizer(TS, with_optimizer(Gurobi.Optimizer, TimeLimit = 600))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses])
    @variable(TS, power_flow_var[data.lines])
    @variable(TS, switched[l = data.lines; data.line_is_aux[l]], Bin)

    if split != nothing
        for l in 1:split #length(data.lines)
            if data.line_is_aux[l]
                JuMP.fix(switched[l], x0[l]; force = true)
            end
        end
    end

    #Set angle at slack node
    JuMP.fix(theta[data.buses[1]], 0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_c1[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    #Voltage law
    # @constraint(TS, voltage_1[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) + (1 - switched[l]) * M >= power_flow_var[l])

    # @constraint(TS, voltage_2[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) <= power_flow_var[l] + (1 - switched[l]) * M)
    #
    @constraint(TS, voltage[l = data.lines; !data.line_is_aux[l]],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) == power_flow_var[l])

    @constraint(TS, connectors[b = data.buses; data.bus_type[b] == 3],
    sum(switched[l] for l in data.lines_at_bus[b] if data.line_is_aux[l]) <= 1)
    #
    @constraint(TS, root_lines[b = data.buses; data.bus_is_root[b] && data.bus_decomposed[b]],
    sum(switched[l] for l in data.lines_at_bus[b] if data.line_is_aux[l]) >= 1)

    @constraint(TS, theta1[l = data.lines; data.line_is_aux[l]],
    theta[data.line_start[l]] <= theta[data.line_end[l]] + (1 - switched[l]) * M2)

    @constraint(TS, theta2[l = data.lines; data.line_is_aux[l]],
    theta[data.line_start[l]] + (1 - switched[l]) * M2 >= theta[data.line_end[l]])

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity_max[g])

    #Angle limits
    @constraint(TS, theta_limit_1[n = data.buses], theta[n] <= THETAMAX)
    @constraint(TS, theta_limit_2[n = data.buses], theta[n] >= THETAMIN)

    # #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines; data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_2[l in data.lines; data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_3[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l])
    @constraint(TS, power_flow_limit_4[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l])

    MOIU.attach_optimizer(TS)
    grb_model = backend(TS).optimizer.model.inner
    update_model!(grb_model)
    #println(grb_model)

    if x0 != nothing
        for l in data.lines
            if data.line_is_aux[l]
                grb_idx = TS.moi_backend.model_to_optimizer_map[index(switched[l])].value
                Gurobi.set_dblattrelement!(grb_model, "Start", grb_idx, x0[l])
            end
        end
    end

    optimize!(TS)

    lineStateDict = Dict{Int64, Int64}()

    for l in data.lines
        if data.line_is_aux[l]
            push!(lineStateDict, l => value(TS[:switched][l]))
        end
    end

    return objective_value(TS), value.(TS[:power_flow_var]).data, lineStateDict, value.(TS[:theta]).data, value.(TS[:generation]).data
end
