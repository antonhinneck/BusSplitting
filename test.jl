# pglib_opf_cases(verbose = true)
select_pglib_case(11)
println(CASE_DIR)

using CSV, DataFrames

CSV.read(string(CASE_DIR,"/bus.csv"))



loadDataset()

geotags = false
if size(bus_df, 2) == 15
    geotags = true
end

if typeof(global_df[1,1]) <: Number
    base_mva = convert(Float64,global_df[1,1])
else
    base_mva = nothing
end

if typeof(global_df[1,2]) <: Number
    objective = global_df[1,2]
else
    objective = nothing
end

if string(global_df[1,3]) != "NaN"
    bibtex = global_df[1,3]
else
    bibtex = nothing
end

# Build JsonModel
#----------------
is_extended_generator = false
if size(gen_df, 2) > 10
    is_extended_generator = true
end



for i in 1:size(bus_df, 1)
    if geotags
        push!(buses_input, bus(bus_df[i, :]...))
    else
        push!(buses_input, bus(bus_df[i, :]..., 0.0, 0.0))
    end
end
