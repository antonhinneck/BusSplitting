function solve_DCOPF_OBSP(data)

    M = 100000

    TS = Model()
    set_optimizer(TS, with_optimizer(Gurobi.Optimizer))
    #set_optimizer(TS, with_optimizer(Cbc.Optimizer, LogLevel = 2))
    #set_optimizer(TS, with_optimizer(GLPK.Optimizer, msg_lev = 2))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses])
    @variable(TS, power_flow_var[data.lines])
    @variable(TS, switched[l in data.lines; data.line_is_aux[l]], Bin)

    #Set angle at slack node
    JuMP.fix(theta[data.buses[1]], 0; force = true)
    #JuMP.fix(switched[11], 0.0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_costs[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    # # Voltage law
    # @constraint(TS, voltage_1[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) + (1 - switched[l]) * M >= power_flow_var[l])
    #
    # @constraint(TS, voltage_2[l = data.lines; data.line_is_aux[l]],
    # (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) <= power_flow_var[l] + (1 - switched[l]) * M)

    @constraint(TS, voltage_3[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) == power_flow_var[l])

    @constraint(TS, assign[rb = data.root_buses], sum(switched[l] for l in data.lines if data.line_is_aux[l] && !data.line_is_proxy[l] && (data.root_bus[data.line_start[l]] == rb || data.root_bus[data.line_end[l]] == rb)) == 1)

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity[g])

    #Angle limits
    @constraint(TS, theta_limit_1[n = data.buses], theta[n] <= 0.6)
    @constraint(TS, theta_limit_2[n = data.buses], theta[n] >= -0.6)

    @constraint(TS, equal_voltage[n1 = data.buses, n2 = data.buses; data.root_bus[n1] == data.root_bus[n2] && data.root_bus[n1] != 0], theta[n1] == theta[n2])

    #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines; data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l] * switched[l])
    @constraint(TS, power_flow_limit_2[l in data.lines; data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l] * switched[l])

    @constraint(TS, power_flow_limit_3[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] <= data.line_capacity[l])
    @constraint(TS, power_flow_limit_4[l in data.lines; !data.line_is_aux[l]], power_flow_var[l] >= -data.line_capacity[l])

    optimize!(TS)

    return objective_value(TS), value.(TS[:power_flow_var]).data, value.(TS[:switched]).data, value.(TS[:theta]).data
end
