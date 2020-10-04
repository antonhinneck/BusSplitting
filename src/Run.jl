using JuMP, Gurobi, MathOptInterface.Utilities
using MathOptInterface ## Cbc, GLPK, HTTP, JSON
include("C:/Users/Anton Hinneck/juliaPackages/GitHub/PowerGrids.jl/src/PowerGrids.jl")
using LightGraphs.SimpleGraphs: nv, ne
using .PowerGrids

ge = Gurobi.Env()
THETAMAX = 0.6
THETAMIN = -0.6

include("Model_LP.jl")
include("dcopf_otsp.jl")
include("dcopf_obsp_change.jl")

# for (i, ds) in enumerate(datasources)
#     println(i, " ", ds)
# end

# case = PowerGrids.readDataset(datasources[27]) # 30 Bus as
# for l in case.lines case.line_capacity[l] *= 0.6 end
#
# case = PowerGrids.readDataset(datasources[28]) # 30 Bus fsr
# for l in case.lines case.line_capacity[l] *= 0.8 end
#
# case = PowerGrids.readDataset(datasources[29]) # 30 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.9 end
#
 # case = PowerGrids.readDataset(datasources[5]) # 14 Bus
 # for l in case.lines case.line_capacity[l] *= 0.55 end
#
# case = PowerGrids.readDataset(datasources[38]) # 5 Bus

# case = PowerGrids.readDataset(datasources[2]) # 118 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.74 end

# case = PowerGrids.readDataset(datasources[36]) # 57 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.3 end

# case = PowerGrids.readDataset(datasources[15]) # 24 Bus ieee rts
# for l in case.lines case.line_capacity[l] *= 0.5 end

# case = PowerGrids.readDataset(datasources[11]) # 179 Bus goc
# for l in case.lines case.line_capacity[l] *= 0.6 end

# case = PowerGrids.readDataset(datasources[38]) # 5 Bus

#case = PowerGrids.readDataset(datasources[28]) # 30 Bus

# case = PowerGrids.readDataset(datasources[29]) # 30 Bus
#for l in case.lines case.line_capacity[l] *= 0.6 end

# case = readDataset(datasources[38]) # 5 Bus

#set_csv_path("C:/Users/Anton Hinneck/Documents/Git/pglib2csv/pglib/2020-08-21.19-54-30-275/csv")
csv_cases(verbose = true)
PowerGrids.select_csv_case(48)
case = PowerGrids.loadCase() # 118 Bus ieee
#for l in case.lines case.line_capacity[l] *= 0.74 end

include("Model_LP.jl") # Change formulation, error
lp = solve_LP(ge, case)
lp_obj = lp[3]
lp_pf = lp[4]
lp_gen = lp[6]
lp_theta = lp[5]

include("dcopf_otsp.jl")
otsp = solve_DCOPF_OTSP(case, start = false)
otsp_obj = otsp[1]
otsp_pf = otsp[2] / 100
otsp_gen = otsp[5] / 100
otsp_theta = otsp[4]
otsp_lineStatus = otsp[3]

for i in 1:length(case.buses)
    # PowerGrids._splitBus!(case, i, 2)
    PowerGrids.__splitBus!(case, i, 2)
end

case.sub_grids

case.generators_at_bus[12]

x0_otsp = split_build_x0(case, otsp[3])
x0_lp = split_build_x0(case)

case.sub_grids

include("dcopf_obsp_notheta2.jl")
line_status = solve_DCOPF_OBSP2(case)

case.sub_grids

include("dcopf_obsp_notheta.jl")
line_status = solve_DCOPF_OBSP(case)
line_status = solve_DCOPF_OBSP(case, x0 = x0_otsp, split = 600)
line_status = solve_DCOPF_OBSP(case, x0 = x0_lp)

case.generators_at_bus[10]
1 = 1

# include("dcopf_obsp_change.jl")
# line_status = solve_DCOPF_OBSP(case, x0 = x0)

function dfs_components(pg::PowerGrids.PowerGrid, sg::PowerGrids.sub_grid; l_stat = nothing) # TODO: op_status

    visited = fill(false, length(sg.buses))
    visited_lines = Vector{Int64}()
    all_visited = false
    components = Vector{PowerGrids.sub_grid}()
    connected_buses = [sg.root_bus]
    iter = 0

    while !all_visited #&& iter < 3

        for i in 1:length(visited)
            if !visited[i]
                connected_buses = [sg.buses[i]]
                break
            end
        end

        _sg = PowerGrids.sub_grid(connected_buses[1], Vector{Int64}(), Vector{Int64}(), Vector{Int64}(), Vector{Int64}(), Vector{Int64}(), Dict{Int64, Int64}(), Dict{Int64, Dict{Int64, Int64}}())

        while length(connected_buses) > 0

            new_buses = Vector{Int64}()
            for b in connected_buses
                push!(_sg.buses, b)
                for l in pg.lines_at_bus[b]
                    if l in Set(sg.lines) && !(l in Set(visited_lines)) && round(l_stat[l]) == 1.0
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
        #push!(components, _sg)
        single_connector = false
        if length(_sg.buses) == 1
            if (pg.bus_type[_sg.buses[1]] == 3)
                single_connector = true
            end
        end

        if length(_sg.buses) > 1 || single_connector
            push!(components, _sg)
        end
    end

    return components
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
                            update_line(reset_case, l, nbid, pg.line_end[l])
                        elseif reset_case.line_end[l] == obid
                            update_line(reset_case, l, pg.line_start[l], nbid, update_fbus = false)
                        else
                            print("Line has no matching start or end.")
                        end
                    end
                    #println(reset_case.lines_at_bus)
                end
            end
        end
    else
        print("This function can only be applied after splitBus! has been called.")
    end

    return reset_case
end

for l in 7:14
    if case.line_is_aux[l]
        println(l," ",x0[l])
    end
end

case.sub_grids

new = reduce_grid(case, 2, line_status[3])

new

case.sub_grids[87]

function pl(case, id, line_status)
    if !case.line_is_aux[id]
        stat = 1
    else
        stat = line_status[3][id]
    end
    println("Id: ",id," Stat:", line_status[3][id], " fbus:", case.line_start[id], " tbus:", case.line_end[id], " flow:", line_status[2][id], " type:", case.line_is_aux[id])
end

case.sub_grids[87]
components = dfs_components(case, case.sub_grids[87], l_stat = line_status[3])
case.lines_at_bus[573]
case.bus_type[573]
line_status[3][926]

println("----")
for i in 923:926
    pl(case, i, line_status)
end

for b in 1:length(new.lines_at_bus)
    if length(new.lines_at_bus[b]) == 0
        println(b)
    end
end

case.line_is_aux
new.lines_at_bus
components = dfs_components(case, case.sub_grids[1], l_stat = line_status[3])

new.sub_grids

include("Model_LP.jl") # Change formulation, error
obsp = solve_LP(ge, new)
obsp_obj = obsp[3]
obsp_pf = obsp[4]
obsp_gen = obsp[6]
obsp_theta = obsp[5]

for i in 199:206
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

function construct_initial_solution(data, solution)

    x0 = zeros(length(data.lines))

    #Iteratie through all sub grids
    for sg in case.sub_grids
        # Get lowest index bus bar
        bus_bar = minimum(sg.bus_bars)
        # Set state of root connection
        x0[sg.bus_bar_root_line[bus_bar]] = 1.0
        # Copy external states
        for extLine in sg.externalLines
            internal = sg.internalLineByBusBar[bus_bar][extLine]
            x0[internal] = solution[extLine]
        end
    end

    return x0
end
