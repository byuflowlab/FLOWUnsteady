#=##############################################################################
# DESCRIPTION
    Plotting and outputting-processing functions for monitoring vehicle and
    component performance. The functions in this module generate
    monitor-functions that can be passed to the simulation engine through the
    optional arguments `extra_runtime_function`.

# ABOUT
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################


"""
    generate_monitor_rotors(rotors::Array{vlm.Rotor}, J_ref::Real,
                                rho_ref::Real, RPM_ref::Real, nsteps_sim::Int;
                                save_path=nothing)

Generate a rotor monitor plotting the aerodynamic performance and blade loading
at every time step.

The aerodynamic performance consists of thrust coefficient
\$C_T=\\frac{T}{\\rho n^2 d^4}\$, torque coefficient
\$C_Q = \\frac{Q}{\\rho n^2 d^5}\$, and propulsive efficiency
\$\\eta = \\frac{T u_\\infty}{2\\pi n Q}\$.

* `J_ref` and `rho_ref` are the reference advance ratio and air density used \
for calculating propulsive efficiency and coefficients. The advance ratio used \
here is defined as \$J=\\frac{u_\\infty}{n d}\$ with \
\$n = \\frac{\\mathrm{RPM}}{60}\$.
* `RPM_ref` is the reference RPM used to estimate the age of the wake.
* `nsteps_sim` is the number of time steps by the end of the simulation (used \
for generating the color gradient).
* Use `save_path` to indicate a directory where to save the plots. If so, it \
will also generate a CSV file with \$C_T\$, \$C_Q\$, and \$\\eta\$.
"""
function generate_monitor_rotors( rotors::Array{vlm.Rotor, 1},
                                    J_ref::Real, rho_ref::Real, RPM_ref::Real,
                                    nsteps_sim::Int;
                                    t_scale=1.0,                    # Time scaling factor
                                    t_lbl="Simulation time (s)",    # Time-axis label
                                    # OUTPUT OPTIONS
                                    out_figs=[],
                                    out_figaxs=[],
                                    save_path=nothing,
                                    run_name="rotor",
                                    figname="monitor_rotor",
                                    disp_conv=true,
                                    conv_suff="_convergence.csv",
                                    save_init_plots=true,
                                    figsize_factor=5/6,
                                    nsteps_plot=1,
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
        formatpyplot()
        fig = plt.figure(figname, figsize=[7*3, 5*2]*figsize_factor)
        axs = fig.subplots(2, 3)
        axs = [axs[6], axs[2], axs[4], axs[1], axs[3], axs[5]]

        push!(out_figs, fig)
        push!(out_figaxs, axs)
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
                ax.set_title("Circulation distribution", color="gray")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Circulation $\Gamma$ (m$^2$/s)")

                ax = axs[2]
                ax.set_title("Normal force distribution", color="gray")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Normal load $N_p$ (N/m)")

                ax = axs[3]
                ax.set_title("Tangential force distribution", color="gray")
                ax.set_xlabel("Element index")
                ax.set_ylabel(L"Tangential load $T_p$ (N/m)")

                ax = axs[4]
                ax.set_title(L"$C_T = \frac{T}{\rho n^2 d^4}$", color="gray")
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Thrust $C_T$")

                ax = axs[5]
                ax.set_title(L"$C_Q = \frac{Q}{\rho n^2 d^5}$", color="gray")
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Torque $C_Q$")

                ax = axs[6]
                ax.set_title(L"$\eta = \frac{T u_\infty}{2\pi n Q}$", color="gray")
                ax.set_xlabel(t_lbl)
                ax.set_ylabel(L"Propulsive efficiency $\eta$")


                for ax in axs
                    ax.spines["right"].set_visible(false)
                    ax.spines["top"].set_visible(false)
                    # ax.grid(true, color="0.8", linestyle="--")
                end

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

            # Save initialization plots (including polars)
            if save_init_plots && save_path!=nothing
                for fi in plt.get_fignums()
                    this_fig = plt.figure(fi)
                    this_fig.savefig(joinpath(save_path, run_name*"_initplot$(fi).png"),
                                                            transparent=false, dpi=300)
                end
            end
        end

        # Write rotor position and time on convergence file
        if save_path!=nothing
            f = open(fname, "a")
            print(f, angle, ",", T, ",", DT)
        end


        # Plot circulation and loads distributions
        if  PFIELD.nt%nsteps_plot==0 && disp_conv

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
        for (i, rotor) in enumerate(rotors)
            CT, CQ = vlm.calc_thrust_torque_coeffs(rotor, rho_ref)
            eta = J_ref*CT/(2*pi*CQ)

            if PFIELD.nt%nsteps_plot==0 && disp_conv
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
            if PFIELD.nt%nsteps_savefig==0 && fcalls!=0 && save_path!=nothing
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
    generate_monitor_wing(wing::Union{vlm.Wing, vlm.WingSystem},
                            Vinf::Function, b_ref::Real, ar_ref::Real,
                            rho_ref::Real, qinf_ref::Real, nsteps_sim::Int;
                            L_dir=[0,0,1],      # Direction of lift component
                            D_dir=[1,0,0],      # Direction of drag component
                            calc_aerodynamicforce_fun=FLOWUnsteady.generate_calc_aerodynamicforce())

Generate a wing monitor computing and plotting the aerodynamic force and
wing loading at every time step.

The aerodynamic force is integrated, decomposed, and reported as overall lift
coefficient \$C_L = \\frac{L}{\\frac{1}{2}\\rho u_\\infty^2 b c}\$ and drag
coefficient \${C_D = \\frac{D}{\\frac{1}{2}\\rho u_\\infty^2 b c}}\$.
The wing loading is reported as the sectional lift and drag coefficients defined
as \$c_\\ell = \\frac{\\ell}{\\frac{1}{2}\\rho u_\\infty^2 c}\$ and
\$c_d = \\frac{d}{\\frac{1}{2}\\rho u_\\infty^2 c}\$, respectively.

The aerodynamic force is calculated through the function
`calc_aerodynamicforce_fun`, which is a user-defined function. The function
can also be automatically generated through,
[`generate_calc_aerodynamicforce`](@ref) which defaults to incluiding the
Kutta-Joukowski force, parasitic drag (calculated from a NACA 0012 airfoil
polar), and unsteady-circulation force.

* `b_ref`       : Reference span length.
* `ar_ref`      : Reference aspect ratio, used to calculate the equivalent \
                    chord \$c = \\frac{b}{\\mathrm{ar}}\$.
* `rho_ref`     : Reference density.
* `qinf_ref`    : Reference dynamic pressure \
                    \$q_\\infty = \\frac{1}{2}\\rho u_\\infty^2\$.
* `nsteps_sim`  : the number of time steps by the end of the simulation (used \
                    for generating the color gradient).
* Use `save_path` to indicate a directory where to save the plots. If so, it \
will also generate a CSV file with \$C_L\$ and \$C_D\$.

Here is an example of this monitor:
![image](http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/wing-example_convergence.png)
"""
function generate_monitor_wing(wing, Vinf::Function, b_ref::Real, ar_ref::Real,
                                rho_ref::Real, qinf_ref::Real, nsteps_sim::Int;
                                lencrit_f=0.5,      # Factor for critical length to ignore horseshoe forces
                                L_dir=[0,0,1],      # Direction of lift component
                                D_dir=[1,0,0],      # Direction of drag component
                                include_trailingboundvortex=false,
                                calc_aerodynamicforce_fun=generate_calc_aerodynamicforce(),
                                # OUTPUT OPTIONS
                                out_Lwing=nothing,
                                out_Dwing=nothing,
                                out_CLwing=nothing,
                                out_CDwing=nothing,
                                out_figs=[],
                                out_figaxs=[],
                                save_path=nothing,
                                run_name="wing",
                                figname="monitor_wing",
                                disp_plot=true,
                                title_lbl="",
                                CL_lbl=L"Lift coefficient $C_L$",
                                CD_lbl=L"Drag coefficient $C_D$",
                                cl_lbl=L"Sectional lift $c_\ell$",
                                cd_lbl=L"Sectional drag $c_d$",
                                y2b_lbl=L"Span position $\frac{2y}{b}$",
                                cl_ttl="Spanwise lift distribution",
                                cd_ttl="Spanwise drag distribution",
                                X_offset=t->zeros(3),
                                S_proj=t->[0, 1, 0],
                                conv_suff="_convergence.csv",
                                figsize_factor=5/6,
                                nsteps_plot=1,
                                nsteps_savefig=10,
                                debug=false)

    fcalls = 0                  # Number of function calls
    prev_wing = nothing

    # Critical length to ignore horseshoe forces
    meanchord = b_ref/ar_ref
    lencrit = lencrit_f*meanchord/vlm.get_m(wing)

    # Name of convergence file
    if save_path!=nothing
        fname = joinpath(save_path, run_name*conv_suff)
    end

    if disp_plot
        formatpyplot()
        fig1 = plt.figure(figname, figsize=[7*2, 5*2]*figsize_factor)
        fig1.suptitle(title_lbl)
        axs1 = fig1.subplots(2, 2)
        axs1 = [axs1[2], axs1[4], axs1[1], axs1[3]]
        ax = axs1[1]
        # xlim([-1,1])
        ax.set_xlabel(y2b_lbl)
        ax.set_ylabel(cl_lbl)
        ax.set_title(cl_ttl, color="gray")
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        ax = axs1[2]
        # xlim([-1,1])
        ax.set_xlabel(y2b_lbl)
        ax.set_ylabel(cd_lbl)
        ax.set_title(cd_ttl, color="gray")
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        ax = axs1[3]
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(CL_lbl)
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        ax = axs1[4]
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(CD_lbl)
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        fig1.tight_layout()

        fig2 = plt.figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
        fig2.suptitle(title_lbl)
        axs2 = fig2.subplots(1, 2)

        ax = axs2[1]
        ax.set_xlabel(y2b_lbl)
        ax.set_ylabel(L"Circulation $\Gamma$")
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        ax = axs2[2]
        ax.set_xlabel(y2b_lbl)
        ax.set_ylabel(L"Effective velocity $V_\infty$")
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        fig2.tight_layout()

        push!(out_figs, fig1)
        push!(out_figs, fig2)
        push!(out_figaxs, axs1)
        push!(out_figaxs, axs2)
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

            t = PFIELD.t
            y2b = 2*[dot(vlm.getControlPoint(wing, i) .- X_offset(t), S_proj(t))
                                                for i in 1:vlm.get_m(wing)]/b_ref

            Lhat = L_dir isa Function ? L_dir(PFIELD.t) : L_dir
            Dhat = D_dir isa Function ? D_dir(PFIELD.t) : D_dir

            # Force at each VLM element
            Ftot = calc_aerodynamicforce_fun(wing, prev_wing, PFIELD, Vinf, DT,
                                                            rho_ref; t=t,
                                                            spandir=cross(Lhat, Dhat),
                                                            lencrit=lencrit,
                                                            include_trailingboundvortex=include_trailingboundvortex,
                                                            debug=debug)
            L, D, S = decompose(Ftot, Lhat, Dhat)
            vlm._addsolution(wing, "L", L)
            vlm._addsolution(wing, "D", D)
            vlm._addsolution(wing, "S", S)

            # Force per unit span at each VLM element
            ftot = calc_aerodynamicforce_fun(wing, prev_wing, PFIELD, Vinf, DT,
                                        rho_ref; t=PFIELD.t, per_unit_span=true,
                                        spandir=cross(Lhat, Dhat),
                                        lencrit=lencrit,
                                        include_trailingboundvortex=include_trailingboundvortex,
                                        debug=debug)
            l, d, s = decompose(ftot, Lhat, Dhat)

            # Lift of the wing
            Lwing = sum(L)
            Lwing = sign(dot(Lwing, Lhat))*norm(Lwing)
            CLwing = Lwing/(qinf_ref*b_ref^2/ar_ref)
            cl = broadcast(x -> sign(dot(x, Lhat)), l) .* norm.(l) / (qinf_ref*b_ref/ar_ref)

            # Drag of the wing
            Dwing = sum(D)
            Dwing = sign(dot(Dwing, Dhat))*norm(Dwing)
            CDwing = Dwing/(qinf_ref*b_ref^2/ar_ref)
            cd = broadcast(x -> sign(dot(x, Dhat)), d) .* norm.(d) / (qinf_ref*b_ref/ar_ref)

            # vlm._addsolution(wing, "cl", cl)
            # vlm._addsolution(wing, "cd", cd)

            if out_Lwing!=nothing; push!(out_Lwing, Lwing); end;
            if out_Dwing!=nothing; push!(out_Dwing, Dwing); end;
            if out_CLwing!=nothing; push!(out_CLwing, CLwing); end;
            if out_CDwing!=nothing; push!(out_CDwing, CDwing); end;

            if PFIELD.nt%nsteps_plot==0 && disp_plot

                ax = axs1[1]
                ax.plot(y2b, cl, "-", alpha=0.5, color=clr)

                ax = axs1[2]
                ax.plot(y2b, cd, "-", alpha=0.5, color=clr)

                ax = axs1[3]
                ax.plot([T], [CLwing], "o", alpha=0.5, color=clr)

                ax = axs1[4]
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

"""
    generate_monitor_statevariables(; save_path=nothing)

Generate a monitor plotting the state variables of the vehicle at every time
step. The state variables are vehicle velocity, vehicle angular velocity, and
vehicle position.

Use `save_path` to indicate a directory where to save the plots.

Here is an example of this monitor on a vehicle flying a circular path:
![image](http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/tetheredwing-example-statemonitor.png)
"""
function generate_monitor_statevariables(; figname="monitor_statevariables",
                                            out_figs=[],
                                            out_figaxs=[],
                                            save_path=nothing,
                                            run_name="",
                                            nsteps_savefig=10)

    formatpyplot()
    fig = plt.figure(figname, figsize=[7*2, 5*1])
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

    for ax in axs
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)
    end

    fig.suptitle("Vehicle state variables", color="gray")
    fig.tight_layout()

    push!(out_figs, fig)
    push!(out_figaxs, axs)


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
            for ax in axs
                ax.legend(loc="best", frameon=false)
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

"""
    generate_monitor_enstrophy(; save_path=nothing)

Generate a monitor plotting the global enstrophy of the flow at every
time step (computed through the particle field). This is calculated by
integrating the local enstrophy defined as ξ = ω⋅ω / 2.

Enstrophy is approximated as 0.5*Σ( Γ𝑝⋅ω(x𝑝) ). This is consistent with
Winckelamns' 1995 CTR report, "Some Progress in LES using the 3-D VPM".

Use `save_path` to indicate a directory where to save the plots. If so, it
will also generate a CSV file with ξ.
"""
function generate_monitor_enstrophy(; save_path=nothing, run_name="",
                                        out_figs=[],
                                        out_figaxs=[],
                                        disp_plot=true,
                                        figname="monitor_enstrophy",
                                        figsize_factor=5/6,
                                        nsteps_savefig=10,
                                        nsteps_plot=1)

    if disp_plot
        formatpyplot()
        fig = plt.figure(figname, figsize=[7*1, 5*1]*figsize_factor)
        ax = fig.gca()

        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(L"Enstrophy ($\mathrm{m}^3/\mathrm{s}^2$)")

        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)

        fig.tight_layout()

        push!(out_figs, fig)
        push!(out_figaxs, ax)
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

"""
    generate_monitor_Cd(; save_path=nothing)

Generate a monitor plotting the mean value of the SFS model coefficient \$C_d\$
across the particle field at every time step. It also plots the ratio of \$C_d\$
values that were clipped to zero (not included in the mean).

Use `save_path` to indicate a directory where to save the plots. If so, it
will also generate a CSV file with the statistics of \$C_d\$ (particles whose
coefficients have been clipped are ignored).
"""
function generate_monitor_Cd(; save_path=nothing, run_name="",
                                        out_figs=[],
                                        out_figaxs=[],
                                        disp_plot=true,
                                        figname="monitor_Cd",
                                        figsize_factor=5/6,
                                        nsteps_plot=1,
                                        nsteps_savefig=10,
                                        ylims=[1e-5, 1e0])

    if disp_plot
        formatpyplot()
        fig = plt.figure(figname, figsize=[7*2, 5*1]*figsize_factor)
        axs = fig.subplots(1, 2)

        ax = axs[1]
        ax.set_ylim(ylims)
        ax.set_yscale("log")
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel(L"Mean $\Vert C_d \Vert$")

        ax = axs[2]
        ax.set_xlabel("Simulation time")
        ax.set_ylim([0, 1])
        ax.set_ylabel(L"Ratio of $C_d$-zeroes"*
                        L" $\frac{n_\mathrm{zeroes}}{n_\mathrm{particles}}$")

        for ax in axs
            ax.spines["right"].set_visible(false)
            ax.spines["top"].set_visible(false)
        end

        fig.tight_layout()

        push!(out_figs, fig)
        push!(out_figaxs, axs)
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

                    this_meanCds = view(meanCds, (length(meanCds)-length(ts)+1):length(meanCds))
                    this_stdCds = view(stdCds, (length(stdCds)-length(ts)+1):length(stdCds))

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

"""
    concatenate(monitors::Array{Function})
    concatenate(monitors::NTuple{N, Function})
    concatenate(monitor1, monitor2, ...)

Concatenates a collection of monitors into a pipeline, returning one monitor of
the form
```julia
monitor(args...; optargs...) =
    monitors[1](args...; optargs...) || monitors[2](args...; optargs...) || ...
```
"""
function concatenate(monitors...)

    monitor(args...; optargs...) = !prod(!f(args...; optargs...) for f in monitors)

    return monitor
end

concatenate(monitors) = concatenate(monitors...)
