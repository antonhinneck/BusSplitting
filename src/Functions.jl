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
                enum!(W, dv; t = t + 1, Wv_last = Wv)
            elseif Wv_last[t] >= i
                enum!(W, dv; t = t + 1, Wv_last = Wv)
            end
        end
    end
end

GenerateSplits!(W, dv)
W
