using JuMP, GLPK
using MathOptInterface ## Cbc, GLPK, HTTP, JSON
include("C:/Users/Anton Hinneck/juliaPackages/GitHub/PowerGrids.jl/src/PowerGrids.jl")

datasources = PowerGrids.datasets()
data = PowerGrids.readDataset(datasources[5])

include("dcopf.jl")
solve_DCOPF(data)

include("dcopf_otsp.jl")
solve_DCOPF_OTSP(data)
