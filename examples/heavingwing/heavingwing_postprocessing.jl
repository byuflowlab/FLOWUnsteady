#=##############################################################################
# DESCRIPTION
    Postprocessing heaving wing simulation
=###############################################################################

import CSV
import DataFrames: DataFrame
import FLOWUnsteady: cross, dot, norm, gt, plt, @L_str

import PyCall
animation = PyCall.pyimport("matplotlib.animation")

# Beautify PyPlot plots
uns.formatpyplot()


# ------------- POST-PROCESSING ------------------------------------------------
println("\nPostprocessing...\n")

# NOTE: Here we will create a time-marching animation of the loading
#       distribution

fig = plt.figure(figsize=[7*2, 5*2 * 0.8]*2/3)
axs = fig.subplots(2, 2)
dt = ttot/nsteps

# Read CSV file with monitor outputs (CL and CD vs t)
csvdata = CSV.read(joinpath(save_path, run_name*"-backwing_convergence.csv"), DataFrame)

# Read initial vtk file
file_prefix = run_name*"_BackWing_vlm"
points, cells, cell_types, data = gt.read_vtk(file_prefix*".4.vtk"; path=save_path)

# Get loading from vtk data and plot
spanposition = uns.calc_Xspan(points, cells, data) * 2/b_back
cls = uns.calc_wingLdist(points, cells, data) ./ (qinf*b_back/ar_back)
cds = uns.calc_wingDdist(points, cells, data) ./ (qinf*b_back/ar_back)

# ---------- Format CL vs t plot
ax = axs[1]

xlims = [0, 0.225]
ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.05:xlims[end])
ax.set_xlabel("Simulation time (s)")

ylims = [-0.2, 0.6]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.2:ylims[end])
ax.set_ylabel(L"Lift coefficient $C_L$")

line1, = ax.plot(csvdata.T, csvdata.CL, "-", color="steelblue")
line2, = ax.plot(zeros(2), ylims, ":", color="black", linewidth=0.75, alpha=0.8)


# ---------- Format CD vs t plot
ax = axs[3]

xlims = [0, 0.225]
ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.05:xlims[end])
ax.set_xlabel("Simulation time (s)")

ylims = [-0.02, 0.06]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.02:ylims[end])
ax.set_ylabel(L"Drag coefficient $C_D$")

line3, = ax.plot(csvdata.T, csvdata.CD, "-", color="darkred")
line4, = ax.plot(zeros(2), ylims, ":", color="black", linewidth=0.75, alpha=0.8)

# ---------- Format lift distribution plot
ax = axs[2]

xlims = [-1, 1]
ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.5:xlims[end])
ax.set_xlabel(L"Span position $2y/b$")

ylims = [-0.5, 1.0]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.5:ylims[end])
ax.set_ylabel(L"Sectional lift $c_\ell$")

line5, = ax.plot(spanposition, cls, "-", color="steelblue")

ax.set_title("Spanwise lift distribution", color="gray")

# ---------- Format drag distribution plot
ax = axs[4]

xlims = [-1, 1]
ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.5:xlims[end])
ax.set_xlabel(L"Span position $2y/b$")

ylims = [-0.5, 1.0]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.5:ylims[end])
ax.set_ylabel(L"Sectional drag $c_d$")

line6, = ax.plot(spanposition, cds, "-", color="darkred")

ax.set_title("Spanwise drag distribution", color="gray")

for ax in axs
    ax.spines["right"].set_visible(false)
    ax.spines["top"].set_visible(false)
end

fig.suptitle("BACK WING", fontsize=16)
fig.tight_layout()

# ---------- Animate the plot
function animate(i)

    # Update time line in CL vs t plot
    line2.set_xdata(i*dt * ones(2))

    # Update time line in CD vs t plot
    line4.set_xdata(i*dt * ones(2))

    if i >= 3
        # Read initial vtk file
        points, cells, cell_types, data = gt.read_vtk(file_prefix*".$(i).vtk"; path=save_path)

        # Get loading from vtk data
        cls = uns.calc_wingLdist(points, cells, data) ./ (qinf*b_back/ar_back)
        cds = uns.calc_wingDdist(points, cells, data) ./ (qinf*b_back/ar_back)

        # Update wing loading plots
        line5.set_ydata(cls)
        line6.set_ydata(cds)
    end

    return (line1, line2)
end

ani = animation.FuncAnimation(fig, animate, 0:200, interval=40)

# Save the animation as a gif
writer = animation.PillowWriter(fps=20, bitrate=1800)
ani.save(joinpath(save_path, run_name*"-animation.gif"), writer=writer)
