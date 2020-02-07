dv = 5
W = Vector{Vector{Int8}}()

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

#function assignments

GenerateSplits!(W, dv)

Ω = [ω(Wv, Int8(dv)) for Wv in W]

W
Ω
