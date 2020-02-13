function GenerateSplits!(W, dv; t = 0, Wv_last = Vector{Int8}())
    if sum(Wv_last) == dv
        push!(W, Wv_last)
    else
        for i in 1:(dv-t)
            Wv = copy(Wv_last)
            push!(Wv, i)
            if t == 0
                GenerateSplits!(W, dv; t = t + 1, Wv_last = Wv)
            elseif Wv_last[t] >= i
                GenerateSplits!(W, dv; t = t + 1, Wv_last = Wv)
            end
        end
    end
end

function ω(Wv::Vector{Int8}, dv::Int8)
    ω = Array{Int8, 1}(undef, dv)
    for i in 1:dv ω[i] = 0 end
    for i in 1:length(Wv) ω[Wv[i]] += 1 end
    return ω
end

function GenerateCombinations!(output, array, k::T where T <: Integer; t = 0, origin = Array{Int8, 1}(undef, k))

    if t == k
        push!(output, origin)
    elseif t < k
        for i in array
            cnt = 1
            if t == 0
                new_array = copy(array)
                deleteat!(new_array, cnt)
                new_origin = copy(origin)
                new_origin[t + 1] = i
                GenerateCombinations!(output, new_array, k, t = t + 1, origin = new_origin)
            else
                if origin[t] <= i
                    new_array = copy(array)
                    deleteat!(new_array, cnt)
                    new_origin = copy(origin)
                    new_origin[t + 1] = i
                    GenerateCombinations!(output, new_array, k, t = t + 1, origin = new_origin)
                end
            end
            cnt += 1
        end
    end
end

#function assignments

dv = 1
W = Vector{Vector{Int8}}()
GenerateSplits!(W, dv)
W


time_ns()
ct = Vector{Float64}()
delta = Vector{Int16}()
permuts = Vector{Int16}()
for i in 1:16
    W = Vector{Vector{Int8}}()
    start = time_ns()
    GenerateSplits!(W, i)
    elapsed = time_ns()
    push!(permuts, length(W))
    push!(ct, (elapsed - start)/10e9)
    push!(delta, i)
    println(i)
end

permuts

output = Vector{Vector{Int8}}()

array = [i for i in 1:dv]

GenerateCombinations!(output, array, 3)
output

Ω = [ω(Wv, Int8(dv)) for Wv in W]

W
Ω
