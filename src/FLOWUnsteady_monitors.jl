#=##############################################################################
# DESCRIPTION
    Plotting and outputting-processing functions for monitoring vehicle and
    component performance. The functions in this module generate
    monitor-functions that can be passed to the simulation engine through the
    optional arguments `extra_runtime_function`.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################


"""
    `generate_monitor_rotors( rotors::Array{vlm.Rotor, 1}, J_ref::Real,
rho_ref::Real, RPM_ref::Real, nsteps_sim::Int)`

Generate monitor of load distribution and performance of a rotor system.
`J_ref` and `rho_ref` are the reference advance ratio and air density used for
calculating propulsive efficiency and coefficients. `RPM_ref` is the reference
RPM used to estimate the age of the wake. `nsteps_sim` is the number of time
steps by the end of the simulation (used for generating the color gradient).
"""
function generate_monitor_rotors( rotors::Array{vlm.Rotor, 1},
                                    J_ref::Real, rho_ref::Real, RPM_ref::Real,
                                    nsteps_sim::Int;
                                    t_scale=1.0,                    # Time scaling factor
                                    t_lbl="Simulation time (s)",    # Time-axis label
                                    # OUTPUT OPTIONS
                                    save_path=nothing,
                                    run_name="rotor",
                                    figname="monitor_rotor",
                                    disp_conv=true,
                                    conv_suff="_convergence.csv",
                                    nsteps_savefig=10,
                                    colors="rgbcmy"^100,
                                    stls="o^*.px"^100, )

    fcalls = 0                  # Number of function calls

    # Name of convergence file
    if save_path!=nothing
        fname = joinpath(save_path, run_name*conv_suff)
    end

    # Call figure
    if disp_conv
        fig = figure(figname, figsize=(7*3,5*2))
        axs = fig.subplots(2, 3)
        axs = [axs[1], axs[3], axs[5], axs[2], axs[4], axs[6]]
    end

    # Function for run_vpm! to call on each iteration
    function extra_runtime_function(sim::Simulation{V, M, R},
                                    PFIELD::vpm.ParticleField,
                                    T::Real, DT::Real; optargs...
                                   ) where{V<:AbstractVLMVehicle, M, R}

        # rotors = vcat(sim.vehicle.rotor_systems...)
        angle = T*360*RPM_ref/60
        t_scaled = T*t_scale

        if fcalls==0
            # Format subplots
            if disp_conv
                ax = axs[1]
                ax.title.set_text("Circulation Distribution")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Circulation $\Gamma$ (m$^2$/s)")
                ax.grid(true, color="0.8", linestyle="--")
                ax = axs[2]
                ax.title.set_text("Plane-of-rotation Normal Force")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Normal Force $N_p$ (N/m)")
                ax.grid(true, color="0.8", linestyle="--")
                ax = axs[3]
                ax.title.set_text("Plane-of-rotation Tangential Force")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Tangential Force $T_p$ (N/m)")
                ax.grid(true, color="0.8", linestyle="--")
                ax = axs[4]
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Thrust $C_T$")
                ax.grid(true, color="0.8", linestyle="--")
                ax = axs[5]
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Torque $C_Q$")
                ax.grid(true, color="0.8", linestyle="--")
                ax = axs[6]
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Propulsive efficiency $\eta$")
                ax.grid(true, color="0.8", linestyle="--")
                fig.tight_layout()
            end

            # Convergence file header
            if save_path!=nothing
                f = open(fname, "w")
                print(f, "ref age (deg),T,DT")
                for (i, rotor) in enumerate(rotors)
                    print(f, ",RPM_$i,CT_$i,CQ_$i,eta_$i")
                end
                print(f, "\n")
                close(f)
            end
        end

        # Write rotor position and time on convergence file
        if save_path!=nothing
            f = open(fname, "a")
            print(f, angle, ",", T, ",", DT)
        end


        # Plot circulation and loads distributions
        if disp_conv

            cratio = PFIELD.nt/nsteps_sim
            cratio = cratio > 1 ? 1 : cratio
            clr = fcalls==0 && false ? (0,0,0) : (1-cratio, 0, cratio)
            stl = fcalls==0 && false ? "o" : "-"
            alpha = fcalls==0 && false ? 1 : 0.5

            # Circulation distribution
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, [vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B]...)
            end
            axs[1].plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Np distribution
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Np"]["field_data"]...)
            end
            axs[2].plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Tp distribution
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Tp"]["field_data"]...)
            end
            axs[3].plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)
        end

        # Plot performance parameters
        for (i,rotor) in enumerate(rotors)
            CT, CQ = vlm.calc_thrust_torque_coeffs(rotor, rho_ref)
            eta = J_ref*CT/(2*pi*CQ)

            if disp_conv
                axs[4].plot([t_scaled], [CT], "$(stls[i])", alpha=alpha, color=clr)
                axs[5].plot([t_scaled], [CQ], "$(stls[i])", alpha=alpha, color=clr)
                axs[6].plot([t_scaled], [eta], "$(stls[i])", alpha=alpha, color=clr)
            end

            if save_path!=nothing
                print(f, ",", rotor.RPM, ",", CT, ",", CQ, ",", eta)
            end
        end

        if disp_conv
            # Save figure
            if fcalls%nsteps_savefig==0 && fcalls!=0 && save_path!=nothing
                fig.savefig(joinpath(save_path, run_name*"_convergence.png"),
                                                    transparent=false, dpi=300)
            end
        end

        # Close convergence file
        if save_path!=nothing
            print(f, "\n")
            close(f)
        end

        fcalls += 1

        return false
    end

    return extra_runtime_function
end



"""
    `generate_monitor_wing(wing, Vinf::Function, b_ref::Real, ar_ref::Real,
rho_ref::Real, qinf_ref::Real, nsteps_sim::Int)`

Generate monitor of load distribution and performance of a wing.
"""
function generate_monitor_wing(wing, Vinf::Function, b_ref::Real, ar_ref::Real,
                                rho_ref::Real, qinf_ref::Real, nsteps_sim::Int;
                                lencrit_f=0.5,      # Factor for critical length to ignore horseshoe forces
                                L_dir=[0,0,1],      # Direction of lift component
                                D_dir=[-1,0,0],     # Direction of drag component
                                # OUTPUT OPTIONS
                                out_Lwing=nothing,
                                out_Dwing=nothing,
                                out_CLwing=nothing,
                                out_CDwing=nothing,
                                save_path=nothing,
                                run_name="wing",
                                figname="monitor_wing",
                                disp_plot=true,
                                title_lbl="",
                                CL_lbl=L"Lift Coefficient $C_L$",
                                CD_lbl=L"Drag Coefficient $C_D$",
                                y2b_i=2,
                                y2b_ref=nothing,
                                ClCL_ref=nothing, CdCD_ref=nothing,
                                CL_ref=nothing, CD_ref=nothing,
                                ref_lbl="Reference", ref_stl="ok",
                                conv_suff="_convergence.csv",
                                figsize_factor=5/6,
                                nsteps_plot=10,
                                nsteps_savefig=10)

    fcalls = 0                  # Number of function calls

    # y2b = 2*wing._ym/b_ref          # Initial span position
    y2b = 2*[vlm.getControlPoint(wing, i)[y2b_i] for i in 1:vlm.get_m(wing)]/b_ref
    prev_wing = nothing

    # Critical length to ignore horseshoe forces
    meanchord = b_ref/ar_ref
    lencrit = lencrit_f*meanchord/vlm.get_m(wing)

    # Name of convergence file
    if save_path!=nothing
        fname = joinpath(save_path, run_name*conv_suff)
    end

    if disp_plot
        fig1 = figure(figname, figsize=[7*2, 5*2]*figsize_factor)
        fig1.suptitle(title_lbl)
        axs1 = fig1.subplots(2, 2)
        axs1 = axs1[axs1[1], axs1[3], axs1[2], axs1[4]]
        ax = axs1[1]
        # xlim([-1,1])
        ax.set_xlabel(L"$\frac{2y}{b}$")
        ax.set_ylabel(L"$\frac{Cl}{CL}$")
        ax.title.set_text("Spanwise lift distribution")

        ax = axs1[2]
        # xlim([-1,1])
        ax.set_xlabel(L"$\frac{2y}{b}$")
        ax.set_ylabel(L"$\frac{Cd}{CD}$")
        ax.title.set_text("Spanwise drag distribution")

        ax = axs1[3]
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(CL_lbl)

        ax = axs1[4]
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(CD_lbl)

        fig1.tight_layout()

        fig2 = figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
        axs2 = fig2.subplots(1, 2)
        ax = axs2[1]
        ax.set_xlabel(L"$\frac{2y}{b}$")
        ax.set_ylabel(L"Circulation $\Gamma$")
        ax = axs2[2]
        ax.set_xlabel(L"$\frac{2y}{b}$")
        ax.set_ylabel(L"Effective velocity $V_\infty$")

        fig2.tight_layout()
    end

    function extra_runtime_function(sim, PFIELD, T, DT; optargs...)

        aux = PFIELD.nt/nsteps_sim
        clr = (1-aux, 0, aux)

        if fcalls==0
            # Convergence file header
            if save_path!=nothing
                f = open(fname, "w")
                print(f, "T,CL,CD\n")
                close(f)
            end
        end

        if PFIELD.nt>2

            # Force at each VLM element
            Ftot = calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                                            rho_ref; t=PFIELD.t,
                                                            lencrit=lencrit)
            L, D, S = decompose(Ftot, L_dir, D_dir)
            vlm._addsolution(wing, "L", L)
            vlm._addsolution(wing, "D", D)
            vlm._addsolution(wing, "S", S)

            # Force per unit span at each VLM element
            ftot = calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                        rho_ref; t=PFIELD.t, per_unit_span=true,
                                        lencrit=lencrit)
            l, d, s = decompose(ftot, L_dir, D_dir)
            vlm._addsolution(wing, "l", l)
            vlm._addsolution(wing, "d", d)
            vlm._addsolution(wing, "s", s)

            # Lift of the wing
            Lwing = norm(sum(L))
            CLwing = Lwing/(qinf_ref*b_ref^2/ar_ref)
            ClCL = norm.(l) / (Lwing/b_ref)

            # Drag of the wing
            Dwing = norm(sum(D))
            CDwing = Dwing/(qinf_ref*b_ref^2/ar_ref)
            CdCD = [sign(dot(this_d, [1,0,0])) for this_d in d].*norm.(d) / (Dwing/b_ref) # Preserves the sign of drag

            vlm._addsolution(wing, "Cl/CL", ClCL)
            vlm._addsolution(wing, "Cd/CD", CdCD)

            if out_Lwing!=nothing; push!(out_Lwing, Lwing); end;
            if out_Dwing!=nothing; push!(out_Dwing, Dwing); end;
            if out_CLwing!=nothing; push!(out_CLwing, CLwing); end;
            if out_CDwing!=nothing; push!(out_CDwing, CDwing); end;

            if PFIELD.nt%nsteps_plot==0 && disp_plot

                ax = axs1[1]
                if y2b_ref!=nothing && ClCL_ref!=nothing
                    ax.plot(y2b_ref, ClCL_ref, ref_stl, label=ref_lbl)
                end
                ax.plot(y2b, ClCL, "-", alpha=0.5, color=clr)

                ax = axs1[2]
                if y2b_ref!=nothing && CdCD_ref!=nothing
                    ax.plot(y2b_ref, CdCD_ref, ref_stl, label=ref_lbl)
                end
                ax.plot(y2b, CdCD, "-", alpha=0.5, color=clr)

                ax = axs1[3]
                if CL_ref!=nothing
                    ax.plot([0, T], CL_ref*ones(2), ":k", label=ref_lbl)
                end
                ax.plot([T], [CLwing], "o", alpha=0.5, color=clr)

                ax = axs1[4]
                if CD_ref!=nothing
                    ax.plot([0, T], CD_ref*ones(2), ":k", label=ref_lbl)
                end
                ax.plot([T], [CDwing], "o", alpha=0.5, color=clr)

                ax = axs2[1]
                ax.plot(y2b, wing.sol["Gamma"], "-", alpha=0.5, color=clr)
                ax = axs2[2]
                if "Vkin" in keys(wing.sol)
                    ax.plot(y2b, norm.(wing.sol["Vkin"]), "-", alpha=0.5, color=[clr[1], 1, clr[3]])
                end
                if "Vvpm" in keys(wing.sol)
                    ax.plot(y2b, norm.(wing.sol["Vvpm"]), "-", alpha=0.5, color=clr)
                elseif "Vind" in keys(wing.sol)
                    ax.plot(y2b, norm.(wing.sol["Vind"]), "-", alpha=0.5, color=clr)
                end
                ax.plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)],
                                                            "-k", alpha=0.5)

                if save_path!=nothing
                    # Save figures
                    if PFIELD.nt%nsteps_savefig==0
                        fig1.savefig(joinpath(save_path, run_name*"_convergence.png"),
                                                    transparent=false, dpi=300)

                        fig2.savefig(joinpath(save_path, run_name*"_convergence2.png"),
                                                    transparent=false, dpi=300)
                    end
                end
            end

            if save_path != nothing
                # Write rotor position and time on convergence file
                f = open(fname, "a")
                print(f, T, ",", CLwing, ",", CDwing, "\n")
                close(f)
            end
        end

        prev_wing = deepcopy(wing)
        fcalls += 1
        return false
    end
end


function generate_monitor_statevariables(; figname="monitor_statevariables",
                                           save_path=nothing,
                                           run_name="",
                                           nsteps_savefig=10)

    fig = figure(figname, figsize=[7*2, 5*1])
    axs = fig.subplots(1, 3)
    ax = axs[1]
    ax.set_xlabel("Simulation time")
    ax.set_ylabel("Velocity")
    Vlbls = [L"V_x", L"V_y", L"V_z"]
    ax = axs[2]
    ax.set_xlabel("Simulation time")
    ax.set_ylabel(L"Angular velocity ($^\circ/t$)")
    Wlbls = [L"\Omega_x", L"\Omega_y", L"\Omega_z"]
    ax = axs[3]
    ax.set_xlabel("Simulation time")
    ax.set_ylabel(L"$O$ position")
    Olbls = [L"O_x", L"O_y", L"O_z"]

    fig.tight_layout()


    function extra_runtime_function(sim, PFIELD, T, DT; optargs...)
        for j in 1:3
            axs[1].plot(sim.t, sim.vehicle.V[j], ".", label=Vlbls[j], alpha=0.8,
                                                            color=clrs[j])
            axs[2].plot(sim.t, sim.vehicle.W[j], ".", label=Wlbls[j], alpha=0.8,
                                                            color=clrs[j])
            axs[3].plot(sim.t, sim.vehicle.system.O[j], ".", label=Olbls[j], alpha=0.8,
                                                            color=clrs[j])
        end
        if sim.nt==0
            for j in 1:2
                axs[j].legend(loc="best", frameon=false)
            end
        end

        if save_path!=nothing
            # Save figures
            if PFIELD.nt%nsteps_savefig==0
                fig.savefig(joinpath(save_path, run_name*"statevariables.png"),
                                                    transparent=false, dpi=300)
            end
        end

        return false
    end

    return extra_runtime_function
end


function generate_monitor_enstrophy(; save_path=nothing, run_name="",
                                     disp_plot=true,
                                     figname="monitor_enstrophy",
                                     nsteps_savefig=10,
                                     nsteps_plot=10)

    if disp_plot
        fig = figure(figname, figsize=[7*1, 5*1])
        ax = fig.gca()
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(L"Enstrophy ($\mathrm{m}^3/\mathrm{s}^2$)")
    end

    enstrophy = []
    ts = []

    function extra_runtime_function(sim, PFIELD, T, DT;
                                                    vprintln=(args...)->nothing)

        vpm.monitor_enstrophy(PFIELD, T, DT;
                                save_path=save_path, run_name=run_name,
                                vprintln=vprintln, out=enstrophy)

        if PFIELD.nt != 0
            push!(ts, T)

            if disp_plot
                if PFIELD.nt%nsteps_plot == 0
                    ax.plot(ts, enstrophy[end-length(ts)+1:end], ".k")
                    ts = []
                end

                if save_path!=nothing && PFIELD.nt%nsteps_savefig==0
                    fig.savefig(joinpath(save_path, run_name*"enstrophy.png"),
                                                    transparent=false, dpi=300)
                end
            end
        end

        return false
    end

    return extra_runtime_function
end


function generate_monitor_Cd(; save_path=nothing, run_name="",
                                     disp_plot=true,
                                     figname="monitor_Cd",
                                     nsteps_savefig=10,
                                     nsteps_plot=10,
                                     ylims=[1e-5, 1e0])

    if disp_plot
        fig = figure(figname, figsize=[7*2, 5*1])
        axs = fig.subplots(1, 2)

        ax = axs[1]
        ax.set_ylim(ylims)
        ax.set_yscale("log")
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(L"Mean $\Vert C_d \Vert$")
        ax = axs[2]
        ax.set_xlabel("Simulation time")
        ax.set_ylabel(L"Ratio of $C_d$-zeroes"*
                      L" $\frac{n_\mathrm{zeroes}}{n_\mathrm{particles}}$")
    end

    meanCds, stdCds, zeroCds, ts, out  = [], [], [], [], []

    function extra_runtime_function(sim, PFIELD, T, DT;
                                                    vprintln=(args...)->nothing)

        vpm.monitor_Cd(PFIELD, T, DT;
                                save_path=save_path, run_name=run_name,
                                vprintln=vprintln, out=out)

        if PFIELD.nt != 0

            t, rationzero, mean, stddev, skew, kurt, minC, maxC = out[end]
            push!(ts, T)
            push!(meanCds, mean)
            push!(stdCds, stddev)
            push!(zeroCds, rationzero)

            if disp_plot
                if PFIELD.nt%nsteps_plot == 0

                    this_meanCds = view(meanCds, length(meanCds)-length(ts)+1, length(meanCds))
                    this_stdCds = view(stdCds, length(stdCds)-length(ts)+1,length(stdCds))

                    ax = axs[1]
                    ax.plot(ts, this_meanCds, ".k")
                    if nsteps_plot > 1
                        ax.fill_between(ts, this_meanCds .+ this_stdCds,
                                            this_meanCds .- this_stdCds;
                                            alpha=0.1, color="tab:blue")
                    end

                    ax = axs[2]
                    ax.plot(ts, zeroCds[end-length(ts)+1:end], ".k")

                    ts = []
                end

                if save_path!=nothing && PFIELD.nt%nsteps_savefig==0
                    fig.savefig(joinpath(save_path, run_name*"Chistory.png"),
                                                    transparent=false, dpi=300)
                end
            end

        end

        return false
    end

    return extra_runtime_function
end
