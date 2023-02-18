#=##############################################################################
# DESCRIPTION
    AOA sweep on 45deg swept-back wing
=###############################################################################
import FLOWUnsteady: cross, dot, norm, plt, @L_str

AOAs        = [0, 2.1, 4.2, 6.3, 8.4, 10.5, 12] # (deg) angles of attack
Xac         = [0.25*b/ar, 0, 0]             # (m) aerodynamic center for moment calculation

# Results are stored in these arrays
CLs, CDs    = [], []                        # Lift and drag at each angle of attack
rolls, pitchs, yaws = [], [], []            # Rolling, pitching, and yawing moment

ls, ds      = [], []                        # Load and drag distributions
spanposs    = []                            # Spanwise positions for load distributions


# ----------------- AOA SWEEP --------------------------------------------------
for AOA in AOAs

    println("\n\n Running AOA = $(AOA) deg")

    # ------------- RUN SIMULATION ----------------

    # Freestream function
    Vinf(X, t) = magVinf*[cosd(AOA), 0.0, sind(AOA)]

    # Define wing monitor with new reference freestream direction
    Shat = [0, 1, 0]                            # Spanwise direction
    Dhat = [cosd(AOA), 0.0, sind(AOA)]          # Direction of drag
    Lhat = cross(Dhat, Shat)                    # Direction of lift

    # Generate wing monitor
    monitor = uns.generate_monitor_wing(wing, Vinf, b, ar,
                                                rho, qinf, nsteps;
                                                calc_aerodynamicforce_fun=calc_aerodynamicforce_fun,
                                                L_dir=Lhat,
                                                D_dir=Dhat,
                                                save_path=nothing,
                                                disp_plot=false
                                                )

    # Run simulation
    pfield = uns.run_simulation(simulation, nsteps;
                                    # SIMULATION OPTIONS
                                    Vinf=Vinf,
                                    # SOLVERS OPTIONS
                                    p_per_step=p_per_step,
                                    max_particles=max_particles,
                                    sigma_vlm_solver=sigma_vlm_solver,
                                    sigma_vlm_surf=sigma_vlm_surf,
                                    sigma_rotor_surf=sigma_vlm_surf,
                                    sigma_vpm_overwrite=sigma_vpm_overwrite,
                                    extra_runtime_function=monitor,
                                    # OUTPUT OPTIONS
                                    save_path=nothing,
                                    v_lvl=1, verbose_nsteps=60
                                    )

    # ------------- POST-PROCESSING ---------------

    # Integrate total lift and drag
    L = sum(wing.sol["L"])
    D = sum(wing.sol["D"])

    # Lift and drag coefficients
    CL = norm(L) / (qinf*b^2/ar)
    CD = norm(D) / (qinf*b^2/ar)

    # Control point of each element
    Xs = [vlm.getControlPoint(wing, i) for i in 1:vlm.get_m(wing)]

    # Force of each element
    Fs = wing.sol["Ftot"]

    # Integrate the total moment with respect to aerodynamic center
    M = sum( cross(X - Xac, F) for (X, F) in zip(Xs, Fs) )

    # Integrated moment decomposed into rolling, pitching, and yawing moments
    lhat = Dhat                   # Rolling direction
    mhat = Shat                   # Pitching direction
    nhat = Lhat                   # Yawing direction

    roll = dot(M, lhat)
    pitch = dot(M, mhat)
    yaw = dot(M, nhat)

    # Sectional loading (in vector form) at each control point
    fs = wing.sol["ftot"]

    # Decompose vectors into lift and drag distribution
    l = [ dot(f, Lhat) for f in fs ]
    d = [ dot(f, Dhat) for f in fs ]

    # Span position of each control point
    spanpos = [ dot(X, Shat) / (b/2) for X in Xs ]

    # Store results
    push!(CLs, CL)
    push!(CDs, CD)

    push!(rolls, roll)
    push!(pitchs, pitch)
    push!(yaws, yaw)

    push!(spanposs, spanpos)
    push!(ls, l)
    push!(ds, d)

end


# ----------------- COMPARISON TO EXPERIMENTAL DATA ----------------------------

# --------- Load distribution
nondim = 0.5*rho*magVinf^2*b/ar   # Normalization factor

cls = ls / nondim
cds = ds / nondim

fig4 = plt.figure(figsize=[7*2, 5*1*0.8]*2/3)
axs = fig4.subplots(1, 2)

ANNOT_SIZE = 10

for (axi, (ax, vals_exp)) in enumerate(zip(axs, [cls_exp, cds_exp]))

    first = true

    for (AOA, pos, cl, cd) in zip(AOAs, spanposs, cls, cds)
        rowi = findfirst(a -> a==AOA, alphas_exp)
        if rowi != nothing && AOA in (axi==1 ? [2.1, 4.2, 6.3, 8.4] : [4.2, 6.3, 8.4])

            # Filter out NaNs
            ys = vals_exp[rowi, :]
            xs = [val for (vali, val) in enumerate(y2b_exp) if !isnan(ys[vali])]
            ys = [val for (vali, val) in enumerate(ys) if !isnan(ys[vali])]

            # Plot experimental
            for f in [-1, 1]
                ax.plot(f*xs, ys, "o--k",
                            label=("Experimental"^(f==1))^first,
                            linewidth=0.5, markersize=5, alpha=1.0)
            end

            # Plot FLOWUnsteady
            ax.plot(pos, axi==1 ? cl : cd, "-", label="FLOWUnsteady"^first,
                            color="steelblue", markersize=8, linewidth=1)

            first = false
        end

    end

    xlims = [0, 1]
    ax.set_xlim(xlims)
    ax.set_xticks(xlims[1]:0.2:xlims[end])
    ax.set_xlabel(L"Span position $2y/b$")

    if axi==1
        ylims = [0, 0.6]
        ax.set_ylim(ylims)
        ax.set_yticks(ylims[1]:0.2:ylims[end])
        ax.set_ylabel(L"Sectional lift $c_\ell$")

        ax.legend(loc="best", frameon=false, fontsize=6)

        ax.annotate(L"\alpha=8.4^\circ", [0.38, 0.53], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6)
        ax.annotate(L"\alpha=6.3^\circ", [0.38, 0.402], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6)
        ax.annotate(L"\alpha=4.2^\circ", [0.38, 0.275], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6)
        ax.annotate(L"\alpha=2.1^\circ", [0.38, 0.145], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6)
    else
        ylims = [-0.04, 0.12]
        ax.set_ylim(ylims)
        ax.set_yticks(ylims[1]:0.04:ylims[end])
        ax.set_ylabel(L"Sectional drag $c_d$")


        ax.annotate(L"\alpha=8.4^\circ", [0.25, 0.030], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6, rotation=-10)
        ax.annotate(L"\alpha=4.2^\circ", [0.25, -0.005], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6, rotation=-5)

        ax.annotate(L"\alpha=6.3^\circ", [0.5, 0.035], xycoords="data", fontsize=ANNOT_SIZE, color="black", alpha=0.6)

        ax.annotate("", [0.4, 0.0145], xycoords="data",
                    xytext=[0.5, 0.035], textcoords="data",
                    arrowprops=Dict(:facecolor=>"black", :linewidth=>0, :alpha=>0.4,
                                    :shrink=>0, :width=>1.0, :headwidth=>5.0, :headlength=>7))
    end

    ax.spines["right"].set_visible(false)
    ax.spines["top"].set_visible(false)
end

fig4.tight_layout()

# --------- Integrated forces: lift and drag

fig5 = plt.figure(figsize=[7*2, 5*1*0.75]*2/3)
axs = fig5.subplots(1, 2)

ax = axs[1]
ax.plot(alphas_exp, CLs_exp, "-ok", label="Experimental")
ax.plot(AOAs, CLs, ":^", label="FLOWUnsteady", color="steelblue", markersize=8)

ylims = [0, 0.8]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.2:ylims[end])
ax.set_ylabel(L"Lift coefficient $C_L$")

ax.legend(loc="lower right", frameon=false, fontsize=10)

ax = axs[2]
ax.plot(alphas_exp, CDs_exp, "-ok", label="Experimental")
ax.plot(AOAs, CDs, ":^", label="FLOWUnsteady", color="steelblue", markersize=8)

ylims = [0, 0.04]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.01:ylims[end])
ax.set_ylabel(L"Drag coefficient $C_D$")

for ax in axs
    xlims = [0, 12]
    xticks = xlims[1]:2:xlims[2]
    ax.set_xlim(xlims)
    ax.set_xticks(xticks)
    ax.set_xticklabels(["$val"*L"^\circ" for val in xticks])
    ax.set_xlabel(L"Angle of attack $\alpha$")

    ax.spines["right"].set_visible(false)
    ax.spines["top"].set_visible(false)
end

fig5.tight_layout()

# --------- Integrated moment: Pitching moment
nondim = 0.5*rho*magVinf^2*b*(b/ar)^2 # Normalization factor

Cls = rolls / nondim
Cms = pitchs / nondim
Cns = yaws / nondim

fig6 = plt.figure(figsize=[7*1, 5*1*0.75]*2/3)
ax = fig6.gca()

ax.plot(AOAs, Cms, ":o", label="FLOWUnsteady", color="steelblue", markersize=8)

xlims = [0, 16]
xticks = xlims[1]:2:xlims[2]
ax.set_xlim(xlims)
ax.set_xticks(xticks)
ax.set_xticklabels(["$val"*L"^\circ" for val in xticks])
ax.set_xlabel(L"Angle of attack $\alpha$")

ylims = [-1.2, 0.2]
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.2:ylims[end])
ax.set_ylabel(L"Pitching moment $C_m$")

ax.spines["right"].set_visible(false)
ax.spines["top"].set_visible(false)

ax.legend(loc="best", frameon=false, fontsize=10)

fig6.tight_layout()


# --------- Save figures
if save_outputs
    fig4.savefig(joinpath(fig_path, "$(run_name)-sweep-loading.png"),
                                                dpi=300, transparent=true)
    fig5.savefig(joinpath(fig_path, "$(run_name)-sweep-CLCD.png"),
                                                dpi=300, transparent=true)
    fig6.savefig(joinpath(fig_path, "$(run_name)-sweep-Cm.png"),
                                                dpi=300, transparent=true)
end
