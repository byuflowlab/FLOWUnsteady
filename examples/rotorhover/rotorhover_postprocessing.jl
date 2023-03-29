#=##############################################################################
# DESCRIPTION
    Postprocessing DJI 9443 simulations and comparison to experimental data
=###############################################################################

import FLOWUnsteady as uns
import PyPlot as plt
import CSV
import DataFrames: DataFrame
import Printf: @printf
import PyPlot: @L_str

uns.formatpyplot()

println("\nPostprocessing...\n")

# Path to where simulations are stored
sims_path = "/home/edoalvar/Dropbox/WhisperAero/LabNotebook/202303/data"

# Simulations to plot
sims_to_plot = [ # run_name, style, color, alpha, label
                ("rotorhover-example-high00", "-", "dodgerblue", 1.0, "rVPM - high fidelity")
                ("rotorhover-example-midhigh00", "--", "dodgerblue", 0.5, "rVPM - mid-high fidelity")
                ("rotorhover-example-midlow01", ":", "dodgerblue", 0.5, "rVPM - mid-low fidelity")
              ]

################################################################################
#   CT history
################################################################################

fig = plt.figure(figsize=[7*1.5, 5*0.75] * 2/3)
ax = fig.gca()

xlims, dx = [[0, 10], 2]
ylims, dy = [[0.04, 0.13], 0.02]

# ------------ PLOT EXPERIMENTAL (Zawodny & Boyd 2016) -------------------------
CTexp     = 0.07193                         # Experimental average CT
CTstdexp  = 0.00177                         # Experimental CT std

ax.plot(xlims, CTexp*ones(2), ":", color="black", label="Experimental")
ax.fill_between(xlims, CTexp*ones(2) .+ CTstdexp,
                       CTexp*ones(2) .- CTstdexp, color="black", alpha=0.1)


# ------------ PLOT URANS SIMULATION (Schenk 2020) -----------------------------
data_path = joinpath(uns.examples_path, "..", "docs", "resources", "data")

data = CSV.read(joinpath(data_path, "dji9443-Austins-RANS_Thrust_new01.csv"), DataFrame)

ax.plot(data[!, 1]*5400/60, data[!, 2]/(1.071778*(5400/60)^2*240e-3^4), "-",
                   color="darkred", alpha=1.0, linewidth=1.0, label="URANS")

# ------------ PLOT FLOWUnsteady SIMULATIONS -----------------------------------
nrotors = 1                 # Number of rotors
coli    = 1                 # Column to plot (1==CT, 2==CQ, 3==eta)

nsteps_per_rev = Dict()

for (run_name, stl, clr, alpha, lbl) in sims_to_plot

    simdata = CSV.read(joinpath(sims_path, run_name, "singlerotor_convergence.csv"), DataFrame)

    for i in 1:nrotors
        ax.plot(simdata[2:end, 1]./360, simdata[2:end, 3 + (i-1)*4 + 1+coli], stl;
                alpha=alpha, label=lbl, color=clr, linewidth=1.0)
    end

    # Calculate nsteps_per_rev
    nsteps_per_rev[run_name] = ceil(Int, 360 / (simdata[2, 1] - simdata[1, 1]))

end

# ----------- FORMAT -------------------------------------------------------
ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:dx:xlims[2])
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:dy:ylims[2])

ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), frameon=false, fontsize=10)

ax.set_xlabel("Number of revolutions")
ax.set_ylabel(L"Thrust coefficient $C_T$")
ax.set_title(L"$C_T = \frac{T}{\rho n^2 d^4}$", color="gray")

ax.spines["right"].set_visible(false)
ax.spines["top"].set_visible(false)


fig.tight_layout()

# Save plot
fig.savefig("dji9443-CTcomparison.png", dpi=300, transparent=true)




################################################################################
#   Blade loading
################################################################################

rotor_axis = [-1.0, 0.0, 0.0]       # Rotor centerline axis
R          = 0.12                   # (m) rotor radius

# Generate statistics (mean and deviation of loading)
for (run_name, _, _, _, _) in sims_to_plot

    read_path = joinpath(sims_path, run_name)
    save_path = read_path*"-statistics"

    # Process outputs between revolutions 8 and 9
    nums      = range(8, 9; length = nsteps_per_rev[run_name]+1)
    nums      = ceil.(Int, nums * nsteps_per_rev[run_name])

    uns.postprocess_statistics(read_path, save_path, nums;
                                        cyl_axial_dir = rotor_axis,
                                        prompt = false)
end

# Start plotting
fig = plt.figure(figsize=[7*1.5, 5] * 2/3)
ax = fig.gca()

# Plot y=0
ax.plot([0, 1], zeros(2), ":k", linewidth=1)

# Plot URANS simulation
data = CSV.read(joinpath(data_path, "dji9443-Austins-RANS_loading.csv"),
                                                        DataFrame; skipto=1)
ax.plot(data[!, 1], data[!, 2], "o:", label="URANS", markersize=5,
                                    color="darkred", alpha=1.0, linewidth=2.0)

# Read and plot FLOWUnsteady mean blade loading and deviation
for (run_name, stl, clr, alpha, lbl) in sims_to_plot

    read_path = joinpath(sims_path, run_name*"-statistics")

    (rs, Gamma,
        Np, Tp) = uns.postprocess_bladeloading(read_path;
                                                O           = zeros(3),
                                                rotor_axis  = rotor_axis,
                                                filename    = "singlerotor_Rotor_Blade1_vlm-statistics.vtk",
                                                fieldsuff   = "-mean"
                                                )


    ax.plot(rs/R, Np, stl; alpha=alpha, label=lbl, color=clr, linewidth=2.0)
end

# Plot quasi-steady solution (BEMT)
read_path = joinpath(sims_path, "rotorhover-example-high00-BEMT")

(rs, Gamma,
    Np, Tp) = uns.postprocess_bladeloading(read_path;
                                            O           = zeros(3),
                                            rotor_axis  = rotor_axis,
                                            filename    = "singlerotor_Rotor_Blade1_vlm.2.vtk",
                                            fieldsuff   = ""
                                            )

ax.plot(rs/R, Np, "-"; alpha=0.7, label="BEMT (quasi-steady)", color="tab:orange", linewidth=1.0)

# Format plot
xlims, dx = [[0, 1], 0.2]
ylims, dy = [[-1, 20], 5]

ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:dx:xlims[2])
ax.set_xlabel(L"Radial position $r/R$")

ax.set_ylim(ylims)
ax.set_yticks(0:dy:ylims[2])
ax.set_ylabel(L"Loading ($\mathrm{N/m}$)")

ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), frameon=false, fontsize=10)

ax.spines["right"].set_visible(false)
ax.spines["top"].set_visible(false)

fig.tight_layout()

# Save plot
fig.savefig("dji9443-loadingcomparison.png", dpi=300, transparent=true)
