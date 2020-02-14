using PyPlot
using Colors

cd(@__DIR__)

fig = figure(figsize=(8, 3))
rc("font", family = "serif", style = "italic", size = 14)
rc("text", usetex = true)
rc("lines", linewidth = 1)

ax = fig.add_axes([0.1,0.15,0.89,0.825])
grid(linewidth = 0.2, linestyle = (0, (10, 10)), color = "lightgray")
ax.tick_params(direction="in",top=true,right=true,width=1.4)

#ax.set_axisbelow(true)
ax.set_yscale("log")
xlabel("\$\\delta_{v}\$")
ylabel("\$t[s]\$")
ylim(bottom = 10e-9, top = 10e0)
##xlim(left=-5,right=5)
#x = [0.01 * i for i in -50000:50000]

for i in 1:length(permuts)
    ax.annotate(string(permuts[i]), (delta[i] - 0.05, ct[i] * 0.16))
end

plot(delta, ct, color = "black", mec = "black", mfc = "white", label = "Split Generation", lw = 1.5, ls = "dotted", marker = "D", ms = 3.4, mew = 1.5)

#plot(u_buses, pw(σ_vec), color = "black", mec = "black", mfc = "white", label = "\$\\sigma\$", lw = 1, ls = "dashed", marker = "o", ms = 2.4, mew = 1)

legend(loc = "upper left", fancybox=false, edgecolor="black")
savefig(string("computation_time.pdf"), format = :pdf)
