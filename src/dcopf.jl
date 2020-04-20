function solve_DCOPF(data)

    TS = Model()

    set_optimizer(TS, with_optimizer(Gurobi.Optimizer))
    #set_optimizer(TS, with_optimizer(Cbc.Optimizer, LogLevel = 2))
    #set_optimizer(TS, with_optimizer(GLPK.Optimizer, msg_lev = 2))

    @variable(TS, generation[data.generators] >= 0)
    @variable(TS, theta[data.buses] >= 0)
    @variable(TS, power_flow_var[data.lines])

    #Set angle at slack node
    JuMP.fix(theta[data.buses[1]], 0; force = true)

    #Minimal generation costs
    @objective(TS, Min, sum(data.generator_costs[g] * generation[g] for g in data.generators))

    #Current law
    @constraint(TS, market_clearing[n = data.buses],
    sum(generation[g] for g in data.generators_at_bus[n]) + sum(power_flow_var[l] for l in data.lines_start_at_bus[n]) - sum(power_flow_var[l] for l in data.lines_end_at_bus[n]) == data.bus_demand[n])

    #Voltage law
    @constraint(TS, voltage_1[l = data.lines],
    (data.base_mva / data.line_reactance[l]) * (theta[data.line_start[l]] - theta[data.line_end[l]]) == power_flow_var[l])

    #Capacity constraint
    @constraint(TS, production_capacity[g = data.generators], generation[g] <= data.generator_capacity[g])

    #Angle limits
    @constraint(TS, theta_limit_1[n = data.buses], theta[n] <= 0.6)
    @constraint(TS, theta_limit_2[n = data.buses], theta[n] >= -0.6)

    #Line limit
    @constraint(TS, power_flow_limit_1[l in data.lines], power_flow_var[l] <= data.line_capacity[l])
    @constraint(TS, power_flow_limit_2[l in data.lines], power_flow_var[l] >= -data.line_capacity[l])

    optimize!(TS)

    return objective_value(TS)
end
