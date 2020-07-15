# Integer Partitioning p(n)
# ZS1 from Fast algorithms for integer partitioning:
# https://pdfs.semanticscholar.org/9613/c1666b5e48a5035141c8927ade99a9de450e.pdf
function zs1(partitions::Vector{Vector{Int64}}, n::Int64)

    # Initialization
    x = ones(n)
    m = 1
    h = 1
    x[1] = n
    push!(partitions, x[1:m])

    # Main procedure
    while x[1] != 1

        if x[h] == 2
            m += 1
            x[h] = 1
            h -= 1
        else
            r = x[h] - 1
            t = m - h + 1
            x[h] = r
            while t >= r
                h += 1
                x[h] = r
                t -= r
            end
            if t == 0
                m = h
            else
                m = h + 1
            end
            if t > 1
                h += 1
                x[h] = t
            end
        end
        push!(partitions, x[1:m])
    end
end

# Calculating r based on integer partition
function r(Pi::Vector{I} where I <: Integer, dv::I where I <: Integer)
    r = convert(eltype(Pi), 1)
    l = 0
    c = 1
    cng = false
    for i in 1:length(Pi)
        v = Pi[i]
        if v == l
            c += 1
            cng = true
        else
            if cng == 1
                r *= 1 / factorial(c)
                c = 1
                cng = false
            end
        end
        l = v
    end
    if c > 1
        r *= 1 / factorial(c)
    end
    return r
end

# Assign unique buses to partitions
function N(arr, dv)
    N = 0
    for i in 1:length(arr)
        Ni = binomial(dv, Int64(arr[i][1]))
        sum = arr[i][1]
        for j in 1:length(arr[i]) - 1
            Ni *= binomial(dv - sum, Int64(arr[i][j + 1]))
            sum += arr[i][j + 1]
        end
        Ni *= r(arr[i], dv)
        N += Ni
    end
    return N
end

dv = 10
P = Vector{Vector{Int64}}()
@time zs1(P, dv)
@time N(P, dv)
P

Ns = Vector{Int64}()
for i in 1:10
    P = Vector{Vector{Int64}}()
    @time zs1(P, i)
    @time push!(Ns, N(P, i))
end

for i in 1:length(Ns)
    print(string(i," & "))
end
for i in 1:length(Ns)
    print(string(Ns[i]," & "))
end

p = Vector{Int64}()
for i in 1:12
    W = Vector{Vector{Int8}}()
    GenerateSplits!(W, i)
    push!(p, length(W))
end
for i in 1:length(p)
    print(string(p[i]," & "))
end
for i in 1:length(p)
    print(string(i," & "))
end

N(W, dv)

partitions = Vector{Vector{Int64}}()
zs1(partitions, 2)
partitions
