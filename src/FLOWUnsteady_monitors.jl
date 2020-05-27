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

    # Function for run_vpm! to call on each iteration
    function extra_runtime_function(sim::Simulation{V, M, R},
                                    PFIELD::vpm.ParticleField,
                                    T::Real, DT::Real
                                   ) where{V<:AbstractVLMVehicle, M, R}

        # rotors = vcat(sim.vehicle.rotor_systems...)
        angle = T*360*RPM_ref/60
        t_scaled = T*t_scale


        # Call figure
        if disp_conv; fig = figure(figname, figsize=(7*3,5*2)); end;

        if fcalls==0
            # Format subplots
            if disp_conv
                subplot(231)
                title("Circulation Distribution")
                xlabel("Element index")
                ylabel(L"Circulation $\Gamma$ (m$^2$/s)")
                grid(true, color="0.8", linestyle="--")
                subplot(232)
                title("Plane-of-rotation Normal Force")
                xlabel("Element index")
                ylabel(L"Normal Force $N_p$ (N/m)")
                grid(true, color="0.8", linestyle="--")
                subplot(233)
                title("Plane-of-rotation Tangential Force")
                xlabel("Element index")
                ylabel(L"Tangential Force $T_p$ (N/m)")
                grid(true, color="0.8", linestyle="--")
                subplot(234)
                xlabel(t_lbl)
                ylabel(L"Thrust $C_T$")
                grid(true, color="0.8", linestyle="--")
                subplot(235)
                xlabel(t_lbl)
                ylabel(L"Torque $C_Q$")
                grid(true, color="0.8", linestyle="--")
                subplot(236)
                xlabel(t_lbl)
                ylabel(L"Propulsive efficiency $\eta$")
                grid(true, color="0.8", linestyle="--")
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
            subplot(231)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, [vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B]...)
            end
            plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Np distribution
            subplot(232)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Np"]["field_data"]...)
            end
            plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Tp distribution
            subplot(233)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Tp"]["field_data"]...)
            end
            plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)
        end

        # Plot performance parameters
        for (i,rotor) in enumerate(rotors)
            CT, CQ = vlm.calc_thrust_torque_coeffs(rotor, rho_ref)
            eta = J_ref*CT/(2*pi*CQ)

            if disp_conv
                subplot(234)
                plot([t_scaled], [CT], "$(stls[i])", alpha=alpha, color=clr)
                subplot(235)
                plot([t_scaled], [CQ], "$(stls[i])", alpha=alpha, color=clr)
                subplot(236)
                plot([t_scaled], [eta], "$(stls[i])", alpha=alpha, color=clr)
            end

            if save_path!=nothing
                print(f, ",", rotor.RPM, ",", CT, ",", CQ, ",", eta)
            end
        end

        if disp_conv
            # Save figure
            if fcalls%nsteps_savefig==0 && fcalls!=0 && save_path!=nothing
                savefig(joinpath(save_path, run_name*"_convergence.png"),
                                                            transparent=false)
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
                                save_path=nothing,
                                run_name="wing",
                                figname="monitor_wing",
                                disp_plot=true,
                                CL_lbl=L"Lift Coefficient $C_L$",
                                CD_lbl=L"Drag Coefficient $C_D$",
                                y2b_i=2,
                                y2b_ref=nothing,
                                ClCL_ref=nothing, CdCD_ref=nothing,
                                CL_ref=nothing, CD_ref=nothing,
                                ref_lbl="Reference", ref_stl="ok",
                                conv_suff="_convergence.csv",
                                figsize_factor=5/6,
                                nsteps_plot=1,
                                nsteps_savefig=10)

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

    function extra_runtime_function(sim, PFIELD, T, DT)

        aux = PFIELD.nt/nsteps_sim
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && (disp_plot || save_path!=nothing)
            figure(figname, figsize=[7*2, 5*2]*figsize_factor)
            subplot(221)
            # xlim([-1,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cl}{CL}$")
            title("Spanwise lift distribution")

            subplot(222)
            # xlim([-1,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cd}{CD}$")
            title("Spanwise drag distribution")

            subplot(223)
            xlabel("Simulation time (s)")
            ylabel(CL_lbl)

            subplot(224)
            xlabel("Simulation time (s)")
            ylabel(CD_lbl)

            figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
            subplot(121)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Circulation $\Gamma$")
            subplot(122)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Effective velocity $V_\infty$")

            # Convergence file header
            if save_path!=nothing
                f = open(fname, "w")
                print(f, "T,CL,CD\n")
                close(f)
            end
        end

        if PFIELD.nt>2 && PFIELD.nt%nsteps_plot==0 && (disp_plot || save_path!=nothing)
            figure(figname)

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

            subplot(221)
            if y2b_ref!=nothing && ClCL_ref!=nothing
                plot(y2b_ref, ClCL_ref, ref_stl, label=ref_lbl)
            end
            plot(y2b, ClCL, "-", alpha=0.5, color=clr)

            subplot(222)
            if y2b_ref!=nothing && CdCD_ref!=nothing
                plot(y2b_ref, CdCD_ref, ref_stl, label=ref_lbl)
            end
            plot(y2b, CdCD, "-", alpha=0.5, color=clr)

            subplot(223)
            if CL_ref!=nothing
                plot([0, T], CL_ref*ones(2), ":k", label=ref_lbl)
            end
            plot([T], [CLwing], "o", alpha=0.5, color=clr)

            subplot(224)
            if CD_ref!=nothing
                plot([0, T], CD_ref*ones(2), ":k", label=ref_lbl)
            end
            plot([T], [CDwing], "o", alpha=0.5, color=clr)

            figure(figname*"_2")
            subplot(121)
            plot(y2b, wing.sol["Gamma"], "-", alpha=0.5, color=clr)
            subplot(122)
            if "Vkin" in keys(wing.sol)
                plot(y2b, norm.(wing.sol["Vkin"]), "-", alpha=0.5, color=[clr[1], 1, clr[3]])
            end
            if "Vvpm" in keys(wing.sol)
                plot(y2b, norm.(wing.sol["Vvpm"]), "-", alpha=0.5, color=clr)
            elseif "Vind" in keys(wing.sol)
                plot(y2b, norm.(wing.sol["Vind"]), "-", alpha=0.5, color=clr)
            end
            plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)],
                                                        "-k", alpha=0.5)

            if save_path!=nothing

                # Write rotor position and time on convergence file
                f = open(fname, "a")
                print(f, T, ",", CLwing, ",", CDwing, "\n")
                close(f)

                # Save figures
                if PFIELD.nt%nsteps_savefig==0
                    figure(figname)
                    savefig(joinpath(save_path, run_name*"_convergence.png"),
                                                            transparent=false)
                    figure(figname*"_2")
                    savefig(joinpath(save_path, run_name*"_convergence2.png"),
                                                            transparent=false)
                end
            end
        end

        prev_wing = deepcopy(wing)
        return false
    end
end


function generate_monitor_statevariables(; figname="monitor_statevariables")

    figure(figname, figsize=[7*2, 5*1])
    subplot(131)
    xlabel("Simulation time")
    ylabel("Velocity")
    Vlbls = [L"V_x", L"V_y", L"V_z"]
    subplot(132)
    xlabel("Simulation time")
    ylabel(L"Angular velocity ($^\circ/t$)")
    Wlbls = [L"\Omega_x", L"\Omega_y", L"\Omega_z"]
    subplot(133)
    xlabel("Simulation time")
    ylabel(L"$\mathop{O}$ position")
    Olbls = [L"O_x", L"O_y", L"O_z"]


    function extra_runtime_function(sim, PFIELD, T, DT)
        for j in 1:3
            subplot(131)
            plot(sim.t, sim.vehicle.V[j], ".", label=Vlbls[j], alpha=0.8,
                                                            color=clrs[j])
            subplot(132)
            plot(sim.t, sim.vehicle.W[j], ".", label=Wlbls[j], alpha=0.8,
                                                            color=clrs[j])
            subplot(133)
            plot(sim.t, sim.vehicle.system.O[j], ".", label=Olbls[j], alpha=0.8,
                                                            color=clrs[j])
        end
        if sim.nt==0
            for j in 1:2
                subplot(130+j)
                legend(loc="best", frameon=false)
            end
        end

        return false
    end

    return extra_runtime_function
end
