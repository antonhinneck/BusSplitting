function solve_DCOPF_OTSP(data; start = false)

    M = 100000

    TS = Model()
    set_optimizer(TS, with_optimizer(Gurobi.Optimizer))
    #set_optimizer(TS, with_optimizer(Cbc.Optimizer, LogLevel = 2))
    #set_optimizer(TS, with_optimizer(GLPK.Optimizer, msg_lev = 2))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses])
    @variable(TS, power_flow_var[data.lines])
    @variable(TS, switched[data.lines], Bin)

    #Set angle at slack node
    JuMP.fix(theta[data.buses[1]], 0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_c1[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    #Voltage law
    @constraint(TS, voltage_1[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) + (1 - switched[l]) * M >= power_flow_var[l])

    @constraint(TS, voltage_2[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) <= power_flow_var[l] + (1 - switched[l]) * M)

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity_max[g])

    #Angle limits
    @constraint(TS, theta_limit_1[n = data.buses], theta[n] <= 0.6)
    @constraint(TS, theta_limit_2[n = data.buses], theta[n] >= -0.6)

    #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines], power_flow_var[l] <= data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_2[l in data.lines], power_flow_var[l] >= -data.line_capacity[l] * switched[l])

    MOIU.attach_optimizer(TS)
    grb_model = backend(TS).optimizer.model.inner
    update_model!(grb_model)

    if start
        for i in 1:length(data.lines)
            grb_idx = TS.moi_backend.model_to_optimizer_map[index(switched[i])].value
            Gurobi.set_dblattrelement!(grb_model, "Start", grb_idx, 1.0)
        end
    end

    optimize!(TS)

    return objective_value(TS), value.(TS[:power_flow_var]).data, value.(TS[:switched]).data, value.(TS[:theta]).data, value.(TS[:generation]).data
end
