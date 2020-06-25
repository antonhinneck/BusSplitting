using JuMP, Gurobi
using MathOptInterface ## Cbc, GLPK, HTTP, JSON
include("C:/Users/Anton Hinneck/juliaPackages/GitHub/PowerGrids.jl/src/PowerGrids.jl")
using LightGraphs.SimpleGraphs: nv, ne
using .PowerGrids

datasources = PowerGrids.datasets()
case = PowerGrids.readDataset(datasources[38]) # 5 Bus
case0 = PowerGrids.readDataset(datasources[38]) # 5 Bus
ge = Gurobi.Env()
THETAMAX = 0.6
THETAMIN = -0.6

include("Model_LP.jl") # Change formulation, error
lp = solve_LP(ge, case0)
lp_obj = lp[3]
lp_pf = lp[4]
lp_gen = lp[6]
lp_theta = lp[5]

case.line_reactance

# graph = PowerGrids.toGraph(case)

include("dcopf_otsp.jl")
otsp = solve_DCOPF_OTSP(case0)
otsp_obj = otsp[1]
otsp_pf = otsp[2] / 100
otsp_gen = otsp[5] / 100
otsp_theta = otsp[4]

include("dcopf_obsp_change.jl")
line_status = solve_DCOPF_OBSP(case)[3]

solve_DCOPF_OBSP(case)

case.sub_grids[5]
case.sub_grids[1]
dfs_components(case, case.sub_grids[5], l_stat = line_status)
case.sub_grids

PowerGrids.splitBus!(case, 1)
PowerGrids.splitBus!(case, 2)
PowerGrids.splitBus!(case, 3)
PowerGrids.splitBus!(case, 4)
PowerGrids.splitBus!(case, 5)

include("dcopf_obsp_change.jl")
line_status = solve_DCOPF_OBSP(case)[3]

case = PowerGrids.readDataset(datasources[38])

function dfs_components(pg::PowerGrids.PowerGrid, sg::PowerGrids.sub_grid; l_stat = nothing) # TODO: op_status

    l_stat = round.(l_stat)
    visited = fill(false, length(sg.buses))
    visited_lines = Vector{Int64}()
    all_visited = false
    components = Vector{PowerGrids.sub_grid}()
    connected_buses = [sg.root_bus]
    iter = 0

    while !all_visited && iter < 3

        for i in 1:length(visited)
            if !visited[i]
                connected_buses = [sg.buses[i]]
                break
            end
        end
        _sg = PowerGrids.sub_grid(connected_buses[1], Vector{Int64}(), Vector{Int64}())

        while length(connected_buses) > 0

            new_buses = Vector{Int64}()
            for b in connected_buses
                push!(_sg.buses, b)
                for l in pg.lines_at_bus[b]
                    if l in Set(sg.lines) && !(l in Set(visited_lines)) && l_stat[l] == 1.0
                        if !(l in Set(visited_lines))
                            if pg.line_start[l] == b
                                push!(new_buses, pg.line_end[l])
                            else
                                push!(new_buses, pg.line_start[l])
                            end
                            push!(_sg.lines, l)
                            push!(visited_lines, l)
                        end
                    end
                end
            end

            for (i, b) in enumerate(sg.buses)
                if b in Set(connected_buses)
                    visited[i] = true
                end
            end
            connected_buses = deepcopy(new_buses)
        end

        all_visited = true
        for v in visited
            if !v
                all_visited = false
                break
            end
        end
        iter += 1
        push!(components, _sg)
    end

    return components
end

function redirect_line(pg::PowerGrids.PowerGrid, line_id, fbus, tbus)

    @inline function swap_tb_fb()
        tmp = deepcopy(tbus)
        tbus = deepcopy(fbus)
        fbus = deepcopy(tmp)
    end

    fbus_is_new = false
    tbus_is_new = false

    if fbus == pg.line_start[line_id] || fbus == pg.line_end[line_id] tbus_is_new = true end
    if tbus == pg.line_start[line_id] || tbus == pg.line_end[line_id] tbus_is_new = true end

    if !((tbus_is_new && fbus_is_new) || (tbus == fbus))

        fbus_old = pg.line_end[line_id]
        tbus_old = pg.line_start[line_id]

        if fbus != fbus_old
            pg.line_start[line_id] = fbus
            if fbus_old in Set(pg.lines_start_at_bus)
                idx = indexin(line_id, pg.lines_start_at_bus[fbus_old])[]
                deleteat!(pg.lines_start_at_bus[fbus_old], idx)
            end
            if fbus_old in Set(pg.lines_at_bus)
                idx = indexin(line_id, pg.lines_at_bus[fbus_old])[]
                deleteat!(pg.lines_at_bus[fbus_old], idx)
            end
        end

        if tbus != tbus_old
            pg.line_end[line_id] = tbus
            if tbus_old in Set(pg.lines_end_at_bus)
                idx = indexin(line_id, pg.lines_end_at_bus[tbus_old])[]
                deleteat!(pg.lines_end_at_bus[tbus_old], idx)
            end
            if tbus_old in Set(pg.lines_at_bus)
                idx = indexin(line_id, pg.lines_at_bus[tbus_old])[]
                deleteat!(pg.lines_at_bus[tbus_old], idx)
            end
        end
    else
        print("No changes made: Choose tbus and fbus to be different and assign new values.")
    end
end

function newBus!(pg::PowerGrid; demand = 0.0, root = 0)
    nb = length(pg.buses)
    id = pg.buses[nb] + 1
    push!(pg.buses, id)
    push!(pg.bus_decomposed, false)
    push!(pg.bus_demand, id => demand)
    push!(pg.bus_is_root, true)
    push!(pg.lines_at_bus, id => Vector{Int64}())
    push!(pg.lines_start_at_bus, id => Vector{Int64}())
    push!(pg.lines_end_at_bus, id => Vector{Int64}())
    push!(pg.adjacent_nodes, Vector{Int64}())
    push!(pg.generators_at_bus, id => Vector{Int64}())
    push!(pg.root_bus, id => root)
    return id
end

function reduce_grid(pg::PowerGrids.PowerGrid, pg_id::Int64, l_stat)

    datasources = PowerGrids.datasets()
    reset_case = PowerGrids.readDataset(datasources[pg_id])

    if pg.sub_grids != nothing
        for sg in pg.sub_grids
            components = dfs_components(pg, sg, l_stat = l_stat)

            for (i,c) in enumerate(components)
                connected_lines = Vector{Int64}()

                for b in c.buses
                    for l in pg.lines_at_bus[b]
                        if l in Set(reset_case.lines)
                            push!(connected_lines, l)
                        end
                    end
                end

                if !(c.root_bus in Set(reset_case.buses))

                    nbid = newBus!(reset_case)
                    obid = pg.root_bus[c.root_bus]

                    for l in connected_lines
                        if reset_case.line_start[l] == obid
                            redirect_line(reset_case, l, nbid, reset_case.line_end[l])
                        elseif reset_case.line_end[l] == obid
                            redirect_line(reset_case, l, reset_case.line_start[l], nbid)
                        else
                            print("Line has no matching start or end.")
                        end
                    end
                end
            end
        end
    else
        print("This function can only be applied after splitBus! has been called.")
    end

    return reset_case
end

new = reduce_grid(case, 38, line_status)

case.sub_grids

for sg in case.sub_grids
    println(sg)
end

# case = PowerGrids.readDataset(datasources[38])
# PowerGrids.splitBus!(case, 1)
# PowerGrids.splitBus!(case, 2)
# PowerGrids.splitBus!(case, 3)
# PowerGrids.splitBus!(case, 4)
# PowerGrids.splitBus!(case, 5)

include("dcopf_obsp_change.jl")
line_status = solve_DCOPF_OBSP(case)[3]

components = dfs_components(case, case.sub_grids[4], l_stat = line_status)

include("Model_LP.jl") # Change formulation, error
obsp = solve_LP(ge, new)
obsp_obj = obsp[3]
obsp_pf = obsp[4]
obsp_gen = obsp[6]
obsp_theta = obsp[5]


for i in 1:6
    println(lp_pf[i]," & ",otsp_pf[i]," & ",obsp_pf[i]," \\\\ ")
end

for i in 1:5
    println(i," & ",case0.generator_costs[i]," & ", case0.generator_capacity[i] / 100," & ", lp_gen[i]," & ",otsp_gen[i]," & ",obsp_gen[i]," \\\\ ")
end

for i in 1:5
    println(i," & ", lp_theta[i]," & ",otsp_theta[i]," & ",obsp_theta[i]," \\\\ ")
end
obsp_theta[6]
obsp_theta[7]

print(lp_obj)
print(otsp_obj)
print(obsp_obj)

for i in 1:6
    println(case0.line_capacity[i] / 100," & ", lp_pf[i]," & ",otsp_pf[i]," & ",obsp_pf[i]," \\\\ ")
end

case0.generators_at_bus[4]

case0.bus_demand
