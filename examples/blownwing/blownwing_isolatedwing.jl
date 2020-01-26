#=##############################################################################
# DESCRIPTION
    Testing of an isolated, planar, 45-deg swept-back wing (Bertin's planar wing
    in Example 7.2, pp. 343 of Bertin's Aerodynamics for Engineers). Validated
    with experimental data from Weber and Brebner (1958), Low-speed tests on
    45-deg swept-back wings, part I, Tables 3 and 4.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Dec 2019
  * License   : MIT
=###############################################################################

"""
    Test FLOWVLM solver on kinematics of an isolated, planar, swept wing.
"""
function isolatedwing(; xfoil=true,
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="isolatedwing",
                        prompt=true,
                        verbose=true, v_lvl=0
                        )

    # ------------ PARAMETERS --------------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Defining parameters..."); end;

    # Rotor parameters
    rotor_file = "apc10x7.csv"          # Rotor geometry
    data_path = uns.def_data_path       # Path to rotor database
    pitch = 0.0                         # (deg) collective pitch of blades
    n_r = 10                            # Number of blade elements
    # xfoil = false                     # Whether to run XFOIL
    pos_bs = [-0.5, 0.5]                # Position on the semispan of every rotor
    CWs = [false, false]                # Clock-wise rotation of every rotor


    # Read radius of this rotor and number of blades
    R, B = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]

    # Simulation parameters
    J = 0.6                             # Advance ratio Vinf/(nD)
    ReD07 = 1.5e6                       # Diameter-based Reynolds at 70% span
    ReD = ReD07/0.7                     # Diameter-based Reynolds
    rho = 1.225                         # (kg/m^3) air density
    mu = 1.81e-5                        # (kg/ms) air dynamic viscosity
    nu = mu/rho
    sound_spd = 343                     # (m/s) speed of sound
    RPM = uns.calc_RPM(ReD, J, R, 2*R, nu) # RPM
    magVinf = J*RPM/60*2*R              # (m/s) freestream velocity
    Minf = magVinf / sound_spd
    Mtip = 2*pi*RPM/60*R / sound_spd
    Rob = 0.075                         # Ratio of rotor radius over span
    b = R/Rob                           # (m) wing span
    Reb = magVinf*b/nu

    # magVinf = 163*0.3048              # (m/s) freestream
    rhoinf = rho                        # (kg/m^3) air density
    alpha = 4.2                         # (deg) angle of attack
    qinf = 0.5*rhoinf*magVinf^2         # (Pa) static pressure

    # Wing geometry
    twist = 0.0                         # (deg) root twist
    lambda = 45.0                       # (deg) sweep
    gamma = 0.0                         # (deg) Dihedral
    # b = 98*0.0254                     # (m) span
    ar = 5.0                            # Aspect ratio
    tr = 1.0                            # Taper ratio
    n_w = 4*2^3                         # Number of horseshoes
    r_w = 3.0                           # Expansion ratio of horeshoes
    n_rot = 1.75                        # Factor for HS density behind rotors

    chord = b/ar
    Rec = magVinf*chord/nu

    # Freestream function
    # Vinf(X, t) = magVinf*[cos(alpha*pi/180), 0.0, sin(alpha*pi/180)]
    # Vinf(X, t) = 1e-12*ones(3)      # (Don't make this zero or things will break)
    # Here I had to give it an initial freestream or the unsteady shedding would
    # in the first step
    Vinf(X, t) = t==0 ? magVinf*[1,0,0] : 1e-12*ones(3)


    # Solver parameters
    nrevs = 10                          # Number of revolutions in simulation
    nsteps_per_rev = 72                 # Time steps per revolution
    # p_per_step = 2                    # Sheds per time step
    p_per_step = 1
    ttot = nrevs/(RPM/60)               # (s) total simulation time
    nsteps = nrevs*nsteps_per_rev       # Number of time steps
    lambda_vpm = 2.125                  # Core overlap
    overwrite_sigma = lambda_vpm * 2*pi*R/(nsteps_per_rev*p_per_step) # Smoothing core size
    surf_sigma = R/10                   # Smoothing radius of lifting surface
    # vlm_sigma = surf_sigma              # Smoothing radius of VLM
    vlm_sigma = -1                      # NOTE: keep this disabled or the VLM will go awry
    shed_unsteady = true                # Shed particles from unsteady loading
                                        # Max particles for memory pre-allocation
    max_particles = ((2*n_r+1)*B + 2*n_w+1)*nrevs*nsteps_per_rev*p_per_step
    plot_disc = true                    # Plot blade discretization for debugging
    wake_coupled = true                 # Coupled VLM solver with VPM
    vlm_rlx = 0.75                      # VLM relaxation (deactivated with -1)
    # vlm_rlx = 0.0
    vlm_init = true                     # Initialize with the VLM semi-infinite wake solution

    if verbose
        println("\t"^(v_lvl+2)*"J:\t\t$(J)")
        println("\t"^(v_lvl+2)*"ReD07:\t\t$(ReD07)")
        println("\t"^(v_lvl+2)*"RPM:\t\t$(ceil(Int, RPM))")
        println("\t"^(v_lvl+2)*"Mtip:\t\t$(round(Mtip, 3))")
        println("\t"^(v_lvl+2)*"Minf:\t\t$(round(Minf, 3))")
        println("\t"^(v_lvl+2)*"R/b:\t\t$(round(Rob, 3))")
        println("\t"^(v_lvl+2)*"b:\t\t$(round(b, 2)) m")
        println("\t"^(v_lvl+2)*"c:\t\t$(round(chord, 2)) m")
        println("\t"^(v_lvl+2)*"Reb:\t\t$(ceil(Int, Reb))")
        println("\t"^(v_lvl+2)*"Rec:\t\t$(ceil(Int, Rec))")
        println("\t"^(v_lvl+2)*"Solution time: "*
            "$(round(magVinf*nrevs/(RPM/60)/b, 3)) spans ($nrevs revolutions)")
        println("\t"^(v_lvl+2)*"vlm_fsgm:\t\t$(round(vlm_sigma/b, 6))")
    end

    # ------------ GENERATING GEOMETRY -----------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;

    # # Generate rotor
    # rotor = uns.generate_rotor(rotor_file; pitch=pitch,
    #                                         n=n_r, CW=CW, ReD=ReD,
    #                                         verbose=verbose, xfoil=xfoil,
    #                                         data_path=data_path,
    #                                         plot_disc=plot_disc,
    #                                         verbose=verbose, v_lvl=v_lvl+1)

    # Wing discretization
    _pos_bs = unique(abs.(pos_bs))
    if length(pos_bs)!=0 && length(pos_bs)!=1 && length(pos_bs)/length(_pos_bs)!=2
        error("Logic error: Position of rotors is expected to be symmetric.")
    end
    refinement = []
    for pos_b in _pos_bs
        # Add wing section before rotor
        pos = pos_b - R/(b/2)                   # Position along semi span
        if pos > 0                              # Length of segment
            len = pos - (length(refinement)!=0 ? refinement[end][1] : 0)
            nfrac = len                         # fraction of horseshoes
            if length(refinement) != 0
                push!(refinement, [len/2, nfrac/2, r_w])
                push!(refinement, [len/2, nfrac/2, 1/r_w])
            else
                push!(refinement, [len, nfrac, 1/r_w])
            end
        end

        # Add rotor section
        pos = min(pos_b + R/(b/2), 1.0)
        len = pos - (length(refinement)!=0 ? refinement[end][1] : 0)
        nfrac = len
        push!(refinement, [len, n_rot*nfrac, 1.01])
    end
    if length(refinement) == 0
        push!(refinement, [1.0, 1.0, 1/r_w])
    end
    # Add tip
    sumc = sum(ref[1] for ref in refinement)
    if abs(sumc-1.0) > 0.0001
        len = abs(sumc-1)
        nfrac = 1.5*len
        push!(refinement, [len/2, nfrac/2, r_w])
        push!(refinement, [len/2, nfrac/2, 1/r_w])
    end

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist, lambda, gamma;
                                                n=n_w, refinement=refinement)

    # Pitch wing to corresponding angle of attack
    O = zeros(3)                                    # Coordinate system origin
    Oaxis = gt.rotation_matrix2(0.0, -alpha, 0.0)   # Coordinate system axes
    vlm.setcoordsystem(wing, O, Oaxis)


    # ------------- SIMULATION SETUP -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    # ----- VEHICLE DEFINITION
    # System of all FLOWVLM objects
    system = vlm.WingSystem()
    vlm.addwing(system, run_name*"_wing", wing)
    # vlm.addwing(system, run_name*"_rotor", rotor)

    # Systems of rotors
    # rotors = vlm.Rotor[rotor]   # Defining this rotor as its own system
    rotor_systems = ()

    # System solved through VLM solver
    vlm_system = system

    # Wake-shedding system
    wake_system = system

    # FVS's Vehicle object
    vehicle = uns.VLMVehicle(   system;
                                vlm_system=vlm_system,
                                rotor_systems=rotor_systems,
                                wake_system=wake_system
                             )

    # ----- MANEUVER DEFINITION
    RPM_fun(t) = 1.0                # RPM (normalized by reference RPM) as a
                                    # function of normalized time

    angle = ()                      # Angle of each tilting system (none in this case)
    # sysRPM = (RPM_fun, )          # RPM of each rotor system
    sysRPM = ()
    Vvehicle(t) = [-1,0,0]          # Translational velocity of vehicle over Vref
    anglevehicle(t) = zeros(3)      # (deg) angle of the vehicle

    Vref = magVinf                  # (m/s) aircraft velocity during cruise
    # RPMref = RPM                  # Reference RPM
    RPMref = 0.0

    # FVS's Maneuver object
    maneuver = uns.KinematicManeuver(angle, sysRPM, Vvehicle, anglevehicle)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; vis_nsteps=nsteps)

    # ----- SIMULATION DEFINITION
    Vinit = Vref*Vvehicle(0)       # Initial vehicle velocity
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot; Vinit=Vinit)

    # monitor = generate_monitor_rotor(J, rho, RPM, nsteps; save_path=save_path,
    #                                                         run_name=run_name)
    monitor = generate_monitor_wing(wing, b, ar, nsteps, Vinf, rhoinf, qinf,
                                                        magVinf, wake_coupled)

    # ------------- RUN SIMULATION ---------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      surf_sigma=surf_sigma,
                                      vlm_init=vlm_init,
                                      vlm_rlx=vlm_rlx,
                                      max_particles=max_particles,
                                      wake_coupled=wake_coupled,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl+1,
                                      save_code=splitdir(@__FILE__)[1],
                                      save_horseshoes=false
                                      )
    return pfield, wing#, rotor
end

function generate_monitor_wing(wing, b, ar, nsteps, Vinf, rhoinf, qinf, magVinf,
                                wake_coupled;
                                figname="monitor_wing", nsteps_plot=1,
                                disp_plot=true, figsize_factor=5/6,
                                save_path=nothing, run_name="wing",
                                nsteps_savefig=10,
                                extra_plots=true)

    # ------------- SIMULATION MONITOR -----------------------------------------
    y2b = 2*wing._ym/b

    # Weber's lift distribution data (Table 3)
    web_2yb = [0.0, 0.041, 0.082, 0.163, 0.245, 0.367, 0.510, 0.653, 0.898, 0.949]
    web_Cl = [0.235, 0.241, 0.248, 0.253, 0.251, 0.251, 0.251, 0.246, 0.192, 0.171]
    web_CL = 0.238
    web_ClCL = web_Cl/web_CL

    # Weber's drag distribution data (Table 3)
    web_Cd = [0.059, 0.025, 0.016, 0.009, 0.007, 0.006, 0.006, 0.004, -0.002, -0.007]
    web_CD = 0.005
    web_CdCD = web_Cd/web_CD

    prev_wing = nothing

    # Critical length to ignore horseshoe forces
    meanchord = b/ar
    lencrit = 0.5*meanchord/vlm.get_m(wing)

    # Name of convergence file
    if save_path!=nothing
        fname = joinpath(save_path, run_name*"_convergence.csv")
    end

    function extra_runtime_function(sim, PFIELD, T, DT)

        aux = PFIELD.nt/nsteps
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && (disp_plot || save_path!=nothing)
            figure(figname, figsize=[7*2, 5*2]*figsize_factor)
            subplot(221)
            xlim([-1,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cl}{CL}$")
            title("Spanwise lift distribution")

            subplot(222)
            xlim([-1,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cd}{CD}$")
            title("Spanwise drag distribution")

            subplot(223)
            xlabel("Simulation time (s)")
            ylabel(L"Lift Coefficient $C_L$")

            subplot(224)
            xlabel("Simulation time (s)")
            ylabel(L"Drag Coefficient $C_D$")

            figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
            subplot(121)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Circulation $\Gamma$")
            subplot(122)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Effective velocity $V_\infty$")

            if extra_plots
                for num in 1:3 # (1==ApA, 2==AB, 3==BBp)
                    figure(figname*"_3_$num", figsize=[7*3, 5*3]*figsize_factor)
                    suptitle("Velocitiy at $(num==1 ? "ApA" : num==2 ? "AB" : "BBp")")
                    for i in 7:9; subplot(330+i); xlabel(L"$\frac{2y}{b}$"); end
                    subplot(331)
                    ylabel(L"$V_\mathrm{VPM}$ Velocity")
                    subplot(334)
                    ylabel(L"$V_\mathrm{VLM}$ Velocity")
                    subplot(337)
                    ylabel(L"$V_\mathrm{kin}$ and $V_\infty$ Velocity")
                    subplot(331)
                    title(L"$x$-component")
                    subplot(332)
                    title(L"$y$-component")
                    subplot(333)
                    title(L"$z$-component")
                end
                figure(figname*"_4", figsize=[7*3, 5*1]*figsize_factor)
                for num in 1:3
                    subplot(130+num)
                    title("Length at $(num==1 ? "ApA" : num==2 ? "AB" : "BBp")")
                    xlabel(L"$\frac{2y}{b}$")
                    if num==1; ylabel("Bound-vortex length"); end;
                end
            end

            # Convergence file header
            if save_path!=nothing
                f = open(fname, "w")
                print(f, "T,CL,CD\n")
                close(f)
            end
        end

        if PFIELD.nt!=0 && PFIELD.nt%nsteps_plot==0 && (disp_plot || save_path!=nothing)
            figure(figname)

            # Force at each VLM element
            Ftot = uns.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                                            rhoinf; t=PFIELD.t,
                                                            lencrit=lencrit)
            L, D, S = uns.decompose(Ftot, [0,0,1], [-1,0,0])
            vlm._addsolution(wing, "L", L)
            vlm._addsolution(wing, "D", D)
            vlm._addsolution(wing, "S", S)

            # Force per unit span at each VLM element
            Vout, lenout = extra_plots ? ([], []) : (nothing, nothing)
            ftot = uns.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                        rhoinf; t=PFIELD.t, per_unit_span=true,
                                        Vout=Vout, lenout=lenout,
                                        lencrit=lencrit)
            l, d, s = uns.decompose(ftot, [0,0,1], [-1,0,0])

            # Lift of the wing
            Lwing = norm(sum(L))
            CLwing = Lwing/(qinf*b^2/ar)
            ClCL = norm.(l) / (Lwing/b)

            # Drag of the wing
            Dwing = norm(sum(D))
            CDwing = Dwing/(qinf*b^2/ar)
            CdCD = [sign(dot(this_d, [1,0,0])) for this_d in d].*norm.(d) / (Dwing/b) # Preserves the sign of drag

            vlm._addsolution(wing, "Cl/CL", ClCL)
            vlm._addsolution(wing, "Cd/CD", CdCD)

            subplot(221)
            plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
            plot(-web_2yb, web_ClCL, "ok")
            plot(y2b, ClCL, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(222)
            plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
            plot(-web_2yb, web_CdCD, "ok")
            plot(y2b, CdCD, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(223)
            plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CLwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(224)
            plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CDwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            figure(figname*"_2")
            subplot(121)
            plot(y2b, wing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
            if wake_coupled && PFIELD.nt!=0
                subplot(122)
                plot(y2b, norm.(wing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                plot(y2b, norm.(wing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)],
                                                            "-k", label="FLOWVLM", alpha=0.5)
            end

            if extra_plots
                m = vlm.get_m(wing)
                for num in 1:3               # (1==ApA, 2==AB, 3==BBp)
                    figure(figname*"_3_$num")
                    for Vi in 1:3           # (1==Vvpm, 2==Vvlm, 3==Vinf && Vkin)
                        for xi in 1:3       # (1==Vx, 2==Vy, 3==Vz)
                            subplot(330 + (Vi-1)*3 + xi)
                            if Vi!=3
                                plot(y2b, [Vout[(i-1)*3 + num][Vi][xi] for i in 1:m], color=clr, alpha=0.5)
                            else
                                plot(y2b, [Vout[(i-1)*3 + num][Vi][xi] for i in 1:m], "k", alpha=0.5)
                                plot(y2b, [Vout[(i-1)*3 + num][Vi+1][xi] for i in 1:m], color=clr, alpha=0.5)
                            end
                        end
                    end
                end
                figure(figname*"_4")
                for num in 1:3
                    subplot(130+num)
                    plot(y2b, [lenout[(i-1)*3 + num] for i in 1:m], color=clr, alpha=0.5)
                end
            end
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
