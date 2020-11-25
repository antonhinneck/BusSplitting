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
include("logger.jl")

# for (i, ds) in enumerate(datasources)
#     println(i, " ", ds)
# end

# case = PowerGrids.readDataset(datasources[32]) # 30 Bus as
# for l in case.lines case.line_capacity[l] *= 0.6 end
#
# case = PowerGrids.readDataset(datasources[28]) # 30 Bus fsr
# for l in case.lines case.line_capacity[l] *= 0.8 end
#
# case = PowerGrids.readDataset(datasources[33]) # 30 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.9 end
#
 # case = PowerGrids.readDataset(datasources[6]) # 14 Bus
 # for l in case.lines case.line_capacity[l] *= 0.55 end
#
# case = PowerGrids.readDataset(datasources[48]) # 5 Bus

# case = PowerGrids.readDataset(datasources[3]) # 118 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.74 end

# case = PowerGrids.readDataset(datasources[46]) # 57 Bus ieee
# for l in case.lines case.line_capacity[l] *= 0.3 end

# case = PowerGrids.readDataset(datasources[18]) # 24 Bus ieee rts
# for l in case.lines case.line_capacity[l] *= 0.5 end

# case = PowerGrids.readDataset(datasources[11]) # 179 Bus goc
# for l in case.lines case.line_capacity[l] *= 0.6 end

# case = PowerGrids.readDataset(datasources[29]) # 30 Bus
#for l in case.lines case.line_capacity[l] *= 0.6 end

# case = readDataset(datasources[38]) # 5 Bus

#set_csv_path("C:/Users/Anton Hinneck/Documents/Git/pglib2csv/pglib/2020-08-21.19-54-30-275/csv")
csv_cases(verbose = true)
PowerGrids.select_csv_case(53)
case = PowerGrids.loadCase() # 118 Bus ieee
#case.line_capacity[6] = 245.0
for l in case.lines case.line_capacity[l] *= 0.48 end

include("Model_LP.jl") # Change formulation, error
lp = solve_LP(ge, case)
lp_obj = lp[3]
lp_pf = lp[4]
lp_gen = lp[6]
lp_theta = lp[5]
lp_duals1 = dual.(lp[7][:power_flow_limit_1]).data
lp_duals2 = dual.(lp[7][:power_flow_limit_2]).data

include("dcopf_otsp.jl")
otsp = solve_DCOPF_OTSP(case)
otsp_obj = otsp[1]
otsp_pf = otsp[2] / 100
otsp_gen = otsp[5] / 100
otsp_theta = otsp[4]
otsp_lineStatus = otsp[3]
log_otsp = otsp[7]
value.(otsp[6][:power_flow_var])

include("Model_OTSP_LP.jl")
otspConv = solve_otsp_lp(ge, case, otsp_lineStatus)
otspConv_duals1 = dual.(otspConv[7][:power_flow_limit_1]).data
otspConv_duals2 = dual.(otspConv[7][:power_flow_limit_2]).data

#PowerGrids.__splitBus!(case, 1, 2)

for i in 1:length(case.buses)
    PowerGrids.__splitBus!(case, i, 2)
    #println(length(case.lines_at_bus[i]))
    # if length(case.lines_at_bus[i]) >= 4
    #     PowerGrids.__splitBus!(case, i, 2)
    # #end
    # else
    #      PowerGrids.__splitBus!(case, i, 1)
    # end
end

case.sub_grids

case.generators_at_bus[12]

x0_otsp = split_build_x0(case, otsp[3])
x0_lp = split_build_x0(case)


sol_allActive(case)
sol_otsp2obsp(case, otsp[3])

case.sub_grids

a = Dict{Int64, Int64}()
push!(a, 1 => 1)
push!(a, 2 => 2)
[i for i in keys(a)]

length([l for l in case.lines if case.line_is_aux[l]])

include("dcopf_obsp_notheta2_sos.jl")
obsp1 = solve_DCOPF_OBSP2SOS(case)
obsp2 = solve_DCOPF_OBSP2SOS(case, x0 = PowerGrids.sol_allActive(case), split = 400)
obsp3 = solve_DCOPF_OBSP2SOS(case, x0 = PowerGrids.sol_otsp2obsp(case, otsp[3]), split = 700)

@inline function opt()
    csplit = length(obsp3[6]) - 80
    sol = Vector{Float64}()
    for l in 1:length(case.lines)
        if case.line_is_aux[l]
            push!(sol, obsp3[6][l])
        else
            push!(sol, 0.0)
        end
    end
    for i in 1:15
        nsol = solve_DCOPF_OBSP2SOS(case, x0 = sol, split = csplit)
        sol = Vector{Float64}()
        for l in 1:length(case.lines)
            if case.line_is_aux[l]
                push!(sol, nsol[6][l])
            else
                push!(sol, 0.0)
            end
        end
        csplit -= 10
    end
end

opt()

line_status = obsp1[6]

log_obsp1 = obsp1[5]
log_obsp2 = obsp2[5]
log_obsp3 = obsp3[5]

include("dcopf_obsp_notheta2.jl")
line_status = solve_DCOPF_OBSP2(case)

case.sub_grids
sg = case.sub_grids[3]
println("--------")
for l in sg.lines
    println(string(l," s:",case.line_start[l]," e:",case.line_end[l]," sol:",line_status[3][l]))
end

case.generators_at_bus[10]
1 = 1

# include("dcopf_obsp_change.jl")
# line_status = solve_DCOPF_OBSP(case, x0 = x0)


## Reduction


case = PowerGrids.loadCase()
#for l in case.lines case.line_capacity[l] *= 0.6 end

for i in 1:length(case.buses)
    __splitBus!(case, i, 2)
end

case.sub_grids

include("dcopf_obsp_notheta2.jl")
line_status = solve_DCOPF_OBSP2(case)

#dfs_components(case, case.sub_grids[1], l_stat = line_status[3])

reduced_case = reduce_grid(case, 48, line_status)
case.bus_demand[2]
include("Model_LP.jl") # Change formulation, error
lpObsp = solve_LP(ge, reduced_case)
lpObsp_obj = lpObsp[3]
lpObsp_pf = lpObsp[4]
lpObsp_gen = lpObsp[6]
lpObsp_theta = lpObsp[5]
lpObsp_duals1 = dual.(lpObsp[7][:power_flow_limit_1]).data
lpObsp_duals2 = dual.(lpObsp[7][:power_flow_limit_2]).data

for l in 1:length(reduced_case.lines)
    println(string(reduced_case.line_start[l], "  ",reduced_case.line_end[l]))
end

reduced_case.line_capacity[6]  = 245.0
for l in 1:length(case.lines)
    println(string(case.line_start[l], "  ",case.line_end[l]))
end

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

cd(@__DIR__)
using PyPlot

fig = figure(figsize=(6, 3))
rc("font", family = "serif", style = "italic", size = 14)
rc("text", usetex = true)
rc("lines", linewidth = 1)

ax = fig.add_axes([0.15,0.16,0.84,0.815])
grid(linewidth = 0.2, linestyle = (0, (10, 10)), color = "lightgray")
ax.tick_params(direction="in",top=true,right=true,width=1.4)

#ax.set_axisbelow(true)
#ax.set_yscale("log")
ylabel("\$z^{*} [\\frac{\\\$}{h}] \$")
xlabel("\$t[s]\$")
ylim(bottom = 128700, top = 136000)
##xlim(left=-5,right=5)
#x = [0.01 * i for i in -50000:50000]

otspmax = log_otsp.time[length(log_otsp.time)]

plot(log_otsp.time, log_otsp.bstobj, color = "red", mec = "black", mfc = "white", label = "OTSP", lw = 1.4, ls = "dashed", marker = "D", ms = 2, mew = 1.0)
plot(log_obsp3.time .+ otspmax, log_obsp3.bstobj, color = "orange", mec = "black", mfc = "white", label = "OBSP (OTSP sol.)", lw = 1.4, ls = "dashed", marker = "D", ms = 2, mew = 1.0)
#
plot(log_obsp1.time, log_obsp1.bstobj, color = "blue", mec = "gray", mfc = "white", label = "OBSP (no start)", lw = 1.4, ls = "dashed", marker = "D", ms = 2, mew = 1.0)
plot(log_obsp2.time, log_obsp2.bstobj, color = "lightgreen", mec = "gray", mfc = "white", label = "OBSP (DCOPF sol.)", lw = 1.4, ls = "dashed", marker = "D", ms = 2, mew = 1.0)

#plot(u_buses, pw(σ_vec), color = "black", mec = "black", mfc = "white", label = "\$\\sigma\$", lw = 1, ls = "dashed", marker = "o", ms = 2.4, mew = 1)

legend(loc = "upper right", fancybox=false, edgecolor="black")
savefig(string("73bus.pdf"), format = :pdf)

log_otsp
