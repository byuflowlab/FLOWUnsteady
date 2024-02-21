#=##############################################################################
# DESCRIPTION
    Advance ratio sweep on APC 10 x7 propeller
=###############################################################################

case_name       = "propeller-Jsweep-example"# Name of this sweep case
save_path       = case_name                 # Where to save this sweep

Js              = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.775] # Advance ratios Vinf/(nD)

# Create path where to save sweep
uns.gt.create_path(save_path, true)

# ----------------- J SWEEP ----------------------------------------------------
for J in Js

    println("\n\n Running J = $(J)")

    magVinf         = J*RPM/60*(2*R)
    Vinf(X, t)      = magVinf*[cosd(AOA), sind(AOA), 0] # (m/s) freestream velocity vector

    # ------------- 1) VEHICLE DEFINITION ---------
    println("\tGenerating geometry...")

    # Generate rotor
    rotor = uns.generate_rotor(rotor_file; pitch=pitch,
                                            n=n, CW=CW, blade_r=r,
                                            altReD=[RPM, J, mu/rho],
                                            xfoil=xfoil,
                                            ncrit=ncrit,
                                            data_path=data_path,
                                            verbose=false,
                                            verbose_xfoil=false,
                                            plot_disc=false
                                            );


    # Generate vehicle
    system = vlm.WingSystem()                   # System of all FLOWVLM objects
    vlm.addwing(system, "Rotor", rotor)

    rotors = [rotor];                           # Defining this rotor as its own system
    rotor_systems = (rotors, );                 # All systems of rotors

    wake_system = vlm.WingSystem()              # System that will shed a VPM wake
                                                # NOTE: Do NOT include rotor when using the quasi-steady solver
    if VehicleType != uns.QVLMVehicle
        vlm.addwing(wake_system, "Rotor", rotor)
    end

    vehicle = VehicleType(  system;
                            rotor_systems=rotor_systems,
                            wake_system=wake_system
                            );

    # ------------- 2) MANEUVER DEFINITION --------
    # No changes

    # ------------- 3) SIMULATION DEFINITION ------
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                     Vinit=Vinit, Winit=Winit);

    # ------------- 4) MONITOR DEFINITION ---------
    monitor_rotor = uns.generate_monitor_rotors(rotors, J, rho, RPM, nsteps;
                                                t_scale=RPM/60,
                                                t_lbl="Revolutions",
                                                save_path=save_path,
                                                run_name="J$(ceil(Int, J*100))",
                                                disp_conv=false,
                                                figname="rotor monitor J=$(J)",
                                                )

    # ------------- 5) RUN SIMULATION -------------
    println("\tRunning simulation...")

    uns.run_simulation(simulation, nsteps;
                        # ----- SIMULATION OPTIONS -------------
                        Vinf=Vinf,
                        rho=rho, mu=mu, sound_spd=speedofsound,
                        # ----- SOLVERS OPTIONS ----------------
                        p_per_step=p_per_step,
                        max_particles=max_particles,
                        vpm_viscous=vpm_viscous,
                        sigma_vlm_surf=sigma_rotor_surf,
                        sigma_rotor_surf=sigma_rotor_surf,
                        sigma_rotor_self=sigma_rotor_self,
                        sigma_vpm_overwrite=sigma_vpm_overwrite,
                        vlm_rlx=vlm_rlx,
                        shed_unsteady=shed_unsteady,
                        shed_starting=shed_starting,
                        extra_runtime_function=monitor_rotor,
                        # ----- OUTPUT OPTIONS ------------------
                        save_path=nothing,
                        v_lvl=1, verbose_nsteps=24
                        );
end

# ----------------- 6) POSTPROCESSING ------------------------------------------

import FLOWUnsteady: mean, plt, @L_str
import CSV
import DataFrames: DataFrame

fig = plt.figure(figsize=[7*2, 5*1*0.9]*2/3)
axs = fig.subplots(1, 2)

push!(axs, axs[1].twinx())

# Experimental thrust, torque, and efficiency (McCrink & Gregory, Figs. 16 and 20)
CTexp = [
            0.06281240640287514 0.12013245033112582;
            0.12466305035109301 0.11682119205298012;
            0.1857549336084395  0.11390728476821191;
            0.242344171187061   0.10940397350993376;
            0.2996921694565542  0.10450331125827812;
            0.35405171553129877 0.09748344370860926;
            0.4061266597890112  0.09231788079470196;
            0.4559586009517784  0.08569536423841057;
            0.5027821225331957  0.07854304635761584;
            0.54812639355719    0.06913907284768207;
            0.5912093580485209  0.059735099337748274;
            0.6327947685447102  0.04953642384105954;
            0.6705996871776098  0.04026490066225161;
            0.7076242137841524  0.03311258278145693;
            0.7424024093979833  0.024768211920529748;
            0.774915970581384   0.016688741721854278;
            0.8051532496921695  0.009801324503311254;
            0.8323521581417018  0.00476821192052973;
        ]

CQexp = [
            0.06219000739580638 0.008780152429555517;
            0.12549988716369898 0.00886238663303177;
            0.18496350136632578 0.008830104278618528;
            0.24362675985530302 0.00901658581926976;
            0.2992119333885983  0.009119787465562586;
            0.3524858619464615  0.009181362570969565;
            0.40345927056358044 0.009117977615958853;
            0.4544420635860513  0.008981675831357767;
            0.5008037072869915  0.008751704275043515;
            0.5472002073506723  0.008250898780250742;
            0.5905199630880067  0.007656396701158079;
            0.6307777214216932  0.006853614448409228;
            0.6694790090961709  0.006144609217720435;
            0.7066372324047991  0.005425214109676885;
            0.74147884822065    0.004705859220513412;
            0.7732396978221479  0.003924057816874497;
            0.8034413955057631  0.0032568668151786006;
            0.8313278263259383  0.0025792993424213068;
            0.8576457208228785  0.0020892591011974007;
        ]

etaexp = [
            0.06562756357670235 0.14010959803117318;
            0.12863002461033646 0.26625723021851;
            0.18867924528301894 0.3852826012379745;
            0.2462674323215751  0.47325760310239395;
            0.30237899917965555 0.5479987471101498;
            0.3550451189499591  0.5993275561190244;
            0.40771123872026266 0.6557472742188082;
            0.45594749794913886 0.692829055112238;
            0.504183757178015   0.7176926541874858;
            0.5489745693191141  0.7318711909911252;
            0.5913043478260871  0.7302720859124467;
            0.6326497128794095  0.7271473786262956;
            0.6710418375717804  0.6985731374449995;
            0.7079573420836753  0.6852741293161309;
            0.7419196062346187  0.6246346782012081;
            0.7739130434782611  0.5293803863077036;
            0.8039376538146024  0.38882034454470865;
            0.8319934372436424  0.23910000745767757;
]

nrevs_to_average = 1.0                  # Number of revolutions to average
nsteps_to_average = VehicleType==uns.QVLMVehicle ? 0 : ceil(Int, nrevs_to_average*nsteps_per_rev)

for (axi, (ax, exp, iden)) in enumerate(zip(axs,
                                            [CTexp, etaexp, CQexp],
                                            [:CT_1, :eta_1, :CQ_1]))

    vals = []

    # Process monitor's CSV ouput
    for J in Js

        # Read CSV
        filename = "J$(ceil(Int, J*100))_convergence.csv"
        data = CSV.read(joinpath(save_path, filename), DataFrame)

        # Average the last few revolutions
        ave = mean(getproperty(data, iden)[end-nsteps_to_average:end])

        push!(vals, ave)
    end

    # Plot experimental
    ax.plot(exp[:, 1], exp[:, 2], "--.k", label="Experimental", linewidth=1)

    # Plot FLOWUnsteady
    ax.plot(Js, vals, "--o", label="FLOWUnsteady", alpha=0.9,
        color=axi==3 ? "darkred" : "steelblue", markersize=6, linewidth=1, clip_on=false)

end


xlims = [0, 1]
for (axi, ax) in enumerate(axs)
    ax.set_xlim(xlims)
    ax.set_xticks(xlims[1]:0.2:xlims[end])
    ax.set_xlabel(L"Advance ratio $J$")

    if axi==2
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)
    end
    if axi==1
        ax.legend(loc="best", frameon=false, fontsize=8)
    end
end

ax = axs[1]
ax.set_ylabel(L"Thrust $C_T$")
ax.set_ylim([0, 0.15])
ax.set_yticks(0:0.03:0.15)
ax.annotate(L"C_T", [0.2, 0.13], xycoords="data", fontsize=10, color="steelblue", alpha=0.8)
ax.annotate(L"C_Q", [0.15, 0.055], xycoords="data", fontsize=10, color="darkred", alpha=0.8)
ax.set_title(L"$C_T = \frac{T}{\rho n^2 d^4}$, $C_Q = \frac{Q}{\rho n^2 d^5}$", color="gray")

ax = axs[3]
ax.set_ylabel(L"Torque $C_Q$")
ax.set_ylim([0, 0.03])
ax.set_yticks(0:0.006:0.03)

ax = axs[2]
ax.set_title(L"$\eta = \frac{T u_\infty}{2\pi n Q}$", color="gray")
ax.set_ylabel(L"Propulsive efficiency $\eta$")
ax.set_ylim([0, 0.8])

fig.tight_layout()

fig.savefig(joinpath(save_path, case_name*".png"), dpi=300, transparent=true)
