using PyPlot
using Colors

fns_t = [7, 7, 10, 11, 13, 13, 13, 19, 19, 19, 30, 30, 30, 30, 30, 56, 308, 355, 368, 600]
fns_o = [103984, 103648, 103151, 101096, 99335, 98896, 96679, 96269, 95873, 95115, 95060, 94257, 94041, 93950, 93716, 93687, 93359, 93310, 93207, 93207]

fs_t = [0, 5, 6, 6, 10, 20, 20, 27, 43, 43, 43, 43, 146, 600]
fs_o = [96488, 96488, 96394, 95010, 93869, 93754, 93660, 93554, 93308, 93268, 93259, 93231, 93222, 93222]

fp_t = [0, 5, 17, 17, 31, 35, 47, 47, 47, 120, 273, 282, 289, 308, 310, 420, 520]
fp_o = [93139, 93139, 93133, 93123,  93097, 93097, 93094, 93092, 93080, 93076, 93076, 93070, 93064, 93059, 93058, 93057, 93057]

fotsp_t = [0, 0, 0, 0, 1, 3, 3, 3, 5, 6, 25, 35, 79]
fotsp_o = [105236, 100588, 100495, 99821, 97789, 94824, 94227, 93557, 93377, 93364, 93164, 93144, 93139]

cd(@__DIR__)

fig = figure(figsize=(8, 3))
rc("font", family = "serif", style = "italic", size = 14)
rc("text", usetex = true)
rc("lines", linewidth = 1)

ax = fig.add_axes([0.1,0.15,0.89,0.825])
grid(linewidth = 0.2, linestyle = (0, (10, 10)), color = "lightgray")
ax.tick_params(direction="in",top=true,right=true,width=1.4)

#ax.set_axisbelow(true)
#ax.set_yscale("log")
ylabel("\$z^{*}\$")
xlabel("\$t[s]\$")
ylim(bottom = 93000, top = 94000)
##xlim(left=-5,right=5)
#x = [0.01 * i for i in -50000:50000]

plot(fotsp_t, fotsp_o, color = "red", mec = "black", mfc = "white", label = "otsp", lw = 1.4, ls = "dashed", marker = "D", ms = 3, mew = 1.5)
plot(fp_t .+ 80, fp_o, color = "orange", mec = "black", mfc = "white", label = "obsp (otsp sol.)", lw = 1.4, ls = "dashed", marker = "D", ms = 3, mew = 1.5)

plot(fns_t, fns_o, color = "blue", mec = "gray", mfc = "white", label = "obsp (no start)", lw = 1.4, ls = "dashed", marker = "D", ms = 3, mew = 1.5)
plot(fs_t, fs_o, color = "lightgreen", mec = "gray", mfc = "white", label = "obsp (lp sol.)", lw = 1.4, ls = "dashed", marker = "D", ms = 3, mew = 1.5)

#plot(u_buses, pw(σ_vec), color = "black", mec = "black", mfc = "white", label = "\$\\sigma\$", lw = 1, ls = "dashed", marker = "o", ms = 2.4, mew = 1)

legend(loc = "upper right", fancybox=false, edgecolor="black")
savefig(string("plot.pdf"), format = :pdf)
