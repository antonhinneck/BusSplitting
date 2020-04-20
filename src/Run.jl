using JuMP, Gurobi
using MathOptInterface ## Cbc, GLPK, HTTP, JSON
include("C:/Users/Anton Hinneck/juliaPackages/GitHub/PowerGrids.jl/src/PowerGrids.jl")

datasources = PowerGrids.datasets()
case = PowerGrids.readDataset(datasources[38])

for i in 1:length(data.generators)
    data.line_capacity[i] = 0.6 * data.line_capacity[i]
end

include("dcopf.jl")
solve_DCOPF(case)

include("dcopf_otsp.jl")
solve_DCOPF_OTSP(case)

PowerGrids.splitBus!(case, 1)

include("dcopf_obsp.jl")
solve_DCOPF_OBSP(case)

PowerGrids.splitBus!(case, 2)
PowerGrids.splitBus!(case, 3)
PowerGrids.splitBus!(case, 4)
PowerGrids.splitBus!(case, 5)

solve_DCOPF_OBSP(case)
