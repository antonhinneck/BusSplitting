# function GenerateSplits!(W, dv; t = 0, Wv_last = Vector{Int8}())
#     if sum(Wv_last) == dv
#         push!(W, Wv_last)
#     else
#         for i in 1:(dv-t)
#             Wv = copy(Wv_last)
#             push!(Wv, i)
#             if t == 0
#                 GenerateSplits!(W, dv; t = t + 1, Wv_last = Wv)
#             elseif Wv_last[t] >= i
#                 GenerateSplits!(W, dv; t = t + 1, Wv_last = Wv)
#             end
#         end
#     end
# end

# function r(Pi::Vector{I} where I <: Integer, dv::I where I <: Integer)
#     r = zeros(eltype(Pi), dv)
#     for i in 1:length(Pi) r[Pi[i]] += 1 end
#     Ni = 1
#     for i in 1:length(r)
#         #print(r[i])
#         if r[i] > 1
#             #println(1 / factorial(big(r[i])))
#             Ni *= 1 / factorial(big(r[i]))
#         end
#     end
#     return Ni
# end

# function GenerateCombinations!(output, array, k::T where T <: Integer; t = 0, origin = Array{Int8, 1}(undef, k))
#
#     if t == k
#         push!(output, origin)
#     elseif t < k
#         for i in array
#             cnt = 1
#             if t == 0
#                 new_array = copy(array)
#                 deleteat!(new_array, cnt)
#                 new_origin = copy(origin)
#                 new_origin[t + 1] = i
#                 GenerateCombinations!(output, new_array, k, t = t + 1, origin = new_origin)
#             else
#                 if origin[t] <= i
#                     new_array = copy(array)
#                     deleteat!(new_array, cnt)
#                     new_origin = copy(origin)
#                     new_origin[t + 1] = i
#                     GenerateCombinations!(output, new_array, k, t = t + 1, origin = new_origin)
#                 end
#             end
#             cnt += 1
#         end
#     end
# end
#
# #function assignments
#
# dv = 4
# W = Vector{Vector{Int8}}()
# a = @time GenerateSplits!(W, dv)
# ω(W[1], Int8(dv))
