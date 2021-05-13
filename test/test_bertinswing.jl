#=##############################################################################
# DESCRIPTION
    Testing of an isolated, planar, 45-deg swept-back wing (Bertin's planar wing
    in Example 7.2, pp. 343 of Bertin's Aerodynamics for Engineers). Validated
    with experimental data from Weber and Brebner (1958), Low-speed tests on
    45-deg swept-back wings, part I, Tables 3 and 4.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

dot(X1, X2) = X1[1]*X2[1] + X1[2]*X2[2] + X1[3]*X2[3]
norm(X) = sqrt(dot(X, X))

"""
    Test FLOWVLM solver with an isolated, planar, swept wing.
"""
function bertin_VLM(;   # TEST OPTIONS
                        tol=0.025,
                        wake_coupled=true,
                        shed_unsteady=true,
                        nsteps=200,
                        vlm_fsgm=-1,
                        surf_fsgm=0.05,
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="bertins",
                        prompt=true,
                        verbose=true, verbose2=true, v_lvl=1,
                        disp_plot=true, figsize_factor=5/6,
                        sim_optargs...
                        )

    if verbose; println("\t"^(v_lvl)*"Running Bertin's wing test..."); end;

    # ------------- GENERATE BERTIN'S WING -------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;
    # Experimental conditions
    magVinf = 163*0.3048            # (m/s) freestream
    rhoinf = 9.093/10^1             # (kg/m^3) air density
    alpha = 4.2                     # (deg) angle of attack
    qinf = 0.5*rhoinf*magVinf^2     # (Pa) static pressure

    # Geometry
    twist = 0.0                     # (deg) root twist
    lambda = 45.0                   # (deg) sweep
    gamma = 0.0                     # (deg) Dihedral
    b = 98*0.0254                   # (m) span
    ar = 5.0                        # Aspect ratio
    tr = 1.0                        # Taper ratio

    # Discretization
    n = 4*2^4                       # Number of horseshoes
    r = 12.0                        # Geometric expansion
    central = false                 # Central expansion

    # Freestream function
    Vinf(X, t) = magVinf*[cos(alpha*pi/180), 0.0, sin(alpha*pi/180)]

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist, lambda, gamma;
                                                    n=n, r=r, central=central)


    # ------------- SIMULATION SETUP -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    wake_len = 2*b              # (m) length to develop the wake
    lambda_vpm = 2.0            # target core overlap of vpm wake

    # Simulation options
    telapsed = wake_len/magVinf # (s) total time to perform maneuver
    # nsteps = 2000             # Number of time steps
    Vcruise = 0.0               # (m/s) aircraft velocity during cruise (dummy)
    RPMh_w = 0.0                # Rotor RPM during hover (dummy)

    # Solver options
    p_per_step = 1              # Number of particle sheds per time steps (dummy)
    overwrite_sigma = lambda_vpm * magVinf * (telapsed/nsteps)/p_per_step # Smoothing core size
    # vlm_sigma = -1            # VLM regularization core size (deactivated with -1)
    vlm_sigma = vlm_fsgm*b
    surf_sigma = surf_fsgm*b    # Smoothing radius of lifting surface on VPM
    # wake_coupled = true       # Coupled VPM wake with VLM solution
    # shed_unsteady = true        # Whether to shed unsteady-loading wake
    # shed_unsteady = false
    vlm_init = true             # Initialize with the VLM semi-infinite wake solution

    # Maneuver definition (dummy)
    Vaircraft(t) = zeros(3)     # Translational velocity of system
    angle_wing(t) = zeros(3)    # Angle of the system

    angle = ()                  # Angle of each tilting system
    RPM = ()                    # RPM of each rotor system
    Vvehicle = Vaircraft        # Velocity of the vehicle
    anglevehicle = angle_wing   # Angle of the vehicle

    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

    # System definitions
    system = vlm.WingSystem()   # System of all FLOWVLM objects
    vlm.addwing(system, "BertinsWing", wing)

    vlm_system = system         # System solved through VLM solver
    wake_system = system        # System that will shed a VPM wake

    # Vehicle definition
    vehicle = uns.VLMVehicle(   system;
                                vlm_system=vlm_system,
                                wake_system=wake_system
                             )

    if verbose
        println("\t"^(v_lvl+1)*"Core overlap:\t\t$(lambda_vpm)")
        println("\t"^(v_lvl+1)*"Core size:\t\t$(round(overwrite_sigma/b, digits=3))*b")
        println("\t"^(v_lvl+1)*"Time step translation:\t$(round(magVinf * (telapsed/nsteps)/b, digits=3))*b")
    end


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

    figname = "monitor_$(save_path)"

    fig1 = figure(figname, figsize=[7*2, 5*2]*figsize_factor)
    axs1 = fig1.subplots(2, 2)

    fig2 = figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
    axs2 = fig2.subplots(1, 2)

    function monitor(sim, PFIELD, T, DT; nsteps_plot=1)

        aux = PFIELD.nt/nsteps
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && disp_plot

            ax = axs1[1]
            ax.set_xlim([0,1])
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"$\frac{Cl}{CL}$")
            ax.title.set_text("Spanwise lift distribution")

            ax = axs1[2]
            ax.set_xlim([0,1])
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"$\frac{Cd}{CD}$")
            ax.title.set_text("Spanwise drag distribution")

            ax = axs1[3]
            ax.set_xlabel("Simulation time (s)")
            ax.set_ylabel(L"Lift Coefficient $C_L$")

            ax = axs1[4]
            ax.set_xlabel("Simulation time (s)")
            ax.set_ylabel(L"Drag Coefficient $C_D$")

            ax = axs2[1]
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"Circulation $\Gamma$")
            ax = axs2[2]
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"Effective velocity $V_\infty$")
        end

        if PFIELD.nt%nsteps_plot==0 && disp_plot

            vlm.calculate_field(wing, "Ftot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
            vlm.calculate_field(wing, "CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
            vlm.calculate_field(wing, "Cftot/CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
            ClCL1 = wing.sol["Cl/CL"]
            CdCD1 = wing.sol["Cd/CD"]

            info = vlm.fields_summary(wing)
            CL = info["CL"]
            CD = info["CD"]

            ax = axs1[1]
            ax.plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
            ax.plot(y2b, ClCL1, "-", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[2]
            ax.plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
            ax.plot(y2b, CdCD1, "-", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[3]
            ax.plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
            ax.plot([T], [CL], "o", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[4]
            ax.plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
            ax.plot([T], [CD], "o", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs2[1]
            ax.plot(y2b, wing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
            if wake_coupled && PFIELD.nt!=0
                ax = axs2[2]
                ax.plot(y2b, norm.(wing.sol["Vkin"]), "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                ax.plot(y2b, norm.(wing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                ax.plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)]/magVinf,
                                                            "-k", label="FLOWVLM", alpha=0.5)
            end
        end

        return false
    end


    # ------------- RUN SIMULATION ---------------------------------------------
    # Simulation setup
    Vref = Vcruise                  # Reference velocity
    RPMref = RPMh_w                 # Reference RPM
    ttot = telapsed                 # Total time to perform maneuver
    Vinit = Vref*Vaircraft(0)       # Initial vehicle velocity
                                    # Maximum number of particles
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                                    Vinit=Vinit)

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
                                      max_particles=max_particles,
                                      wake_coupled=wake_coupled,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose2, v_lvl=v_lvl+1,
                                      save_horseshoes=!wake_coupled,
                                      sim_optargs...
                                      )


    # ------------- POST-PROCESSING --------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Postprocessing..."); end;
    # Simulation fift and drag
    vlm.calculate_field(wing, "CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
    info = vlm.fields_summary(wing)
    CLsim = info["CL"]
    CDsim = info["CD"]

    # Weber's experimental lift and drag (Table 4)
    CLexp = 0.238
    CDexp = 0.005

    # Error
    CLerr = abs(CLexp-CLsim)/CLexp
    CDerr = abs(CDexp-CDsim)/CDexp

    res = CLerr<tol && (CDerr<tol || true)

    if verbose
        t = "\t"^(v_lvl+1)
        @printf "%0s%10s\t%-11s\t%-11s\t%-11s\n"    t "PARAMETER"   "Experimental"  "Simulation"    "Error %"
        @printf "%0s%10s\t%11.5e\t%11.5e\t%11.5e\n" t "CL"          CLexp           CLsim           100*CLerr
        @printf "%0s%10s\t%11.5e\t%11.5e\t%11.5e\n" t "CD"          CDexp           CDsim           100*CDerr
        println("\t"^(v_lvl+1)*"TEST RESULT:\t$res")
    end

    return res
end


"""
    Test FLOWVLM solver on kinematics of an isolated, planar, swept wing.
"""
function bertin_kinematic(;   # TEST OPTIONS
                        tol=0.025,
                        wake_coupled=true,
                        nsteps=150,
                        vlm_fsgm=-1,
                        surf_fsgm=0.046,
                        p_per_step = 1,
                        vlm_rlx = -1,
                        VehicleType=uns.VLMVehicle,
                        shed_unsteady=true,
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="bertins",
                        prompt=true,
                        verbose=true, verbose2=true, v_lvl=1,
                        disp_plot=true, figsize_factor=5/6,
                        sim_optargs...
                        )

    if verbose; println("\t"^(v_lvl)*"Running Bertin's wing test..."); end;

    # ------------- GENERATE BERTIN'S WING -------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;
    # Experimental conditions
    magVinf = 163*0.3048            # (m/s) freestream
    rhoinf = 9.093/10^1             # (kg/m^3) air density
    alpha = 4.2                     # (deg) angle of attack
    qinf = 0.5*rhoinf*magVinf^2     # (Pa) static pressure

    # Geometry
    twist = 0.0                     # (deg) root twist
    lambda = 45.0                   # (deg) sweep
    gamma = 0.0                     # (deg) Dihedral
    b = 98*0.0254                   # (m) span
    ar = 5.0                        # Aspect ratio
    tr = 1.0                        # Taper ratio

    # Discretization
    n = 4*2^4                       # Number of horseshoes
    r = 12.0                        # Geometric expansion
    central = false                 # Central expansion

    # Freestream function
    # Vinf(X, t) = magVinf*[cos(alpha*pi/180), 0.0, sin(alpha*pi/180)]
    # Vinf(X, t) = 1e-12*ones(3)      # (Don't make this zero or things will break)
    # Here I had to give it an initial freestream or the unsteady shedding would
    # in the first step
    Vinf(X, t) = t==0 ? magVinf*[1,0,0] : 1e-12*ones(3)

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist, lambda, gamma;
                                                    n=n, r=r, central=central)

    # Pitch wing to corresponding angle of attack
    O = zeros(3)                    # Coordinate system origin
    Oaxis = gt.rotation_matrix2(0.0, -alpha, 0.0) # Coordinate system axes
    vlm.setcoordsystem(wing, O, Oaxis)


    # ------------- SIMULATION SETUP -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    wake_len = 2*b              # (m) length to develop the wake
    lambda_vpm = 2.0            # target core overlap of vpm wake

    # Simulation options
    telapsed = wake_len/magVinf # (s) total time to perform maneuver
    # nsteps = 2000             # Number of time steps
    Vcruise = magVinf           # (m/s) aircraft velocity during cruise
    RPMh_w = 0.0                # Rotor RPM during hover (dummy)

    # Solver options
    # p_per_step = 1              # Number of particle sheds per time steps
    overwrite_sigma = lambda_vpm * magVinf * (telapsed/nsteps)/p_per_step # Smoothing core size
    # vlm_sigma = -1            # VLM regularization core size (deactivated with -1)
    vlm_sigma = vlm_fsgm*b
    surf_sigma = surf_fsgm*b    # Smoothing radius of lifting surface on VPM
    # wake_coupled = true       # Coupled VPM wake with VLM solution
    # shed_unsteady = true        # Whether to shed unsteady-loading wake
    # shed_unsteady = false
    # vlm_rlx = -1                # VLM relaxation (deactivated with -1)
    vlm_init = true             # Initialize with the VLM semi-infinite wake solution

    # Maneuver definition
    Vaircraft(t) = [-1,0,0]     # Translational velocity of system
    angle_wing(t) = zeros(3)    # Angle of the system

    angle = ()                  # Angle of each tilting system
    RPM = ()                    # RPM of each rotor system
    Vvehicle = Vaircraft        # Velocity of the vehicle
    anglevehicle = angle_wing   # Angle of the vehicle

    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

    # System definitions
    system = vlm.WingSystem()   # System of all FLOWVLM objects
    vlm.addwing(system, "BertinsWing", wing)

    vlm_system = system         # System solved through VLM solver
    wake_system = system        # System that will shed a VPM wake

    # Vehicle definition
    vehicle = VehicleType(      system;
                                vlm_system=vlm_system,
                                wake_system=wake_system
                             )

    if verbose
        println("\t"^(v_lvl+1)*"Core overlap:\t\t$(lambda_vpm)")
        println("\t"^(v_lvl+1)*"Core size:\t\t$(round(overwrite_sigma/b, digits=3))*b")
        println("\t"^(v_lvl+1)*"Time step translation:\t$(round(magVinf * (telapsed/nsteps)/b, digits=3))*b")
    end


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

    figname = "monitor_$(save_path)"

    fig1 = figure(figname, figsize=[7*2, 5*2]*figsize_factor)
    axs1 = fig1.subplots(2, 2)

    fig2 = figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
    axs2 = fig2.subplots(1, 2)

    function monitor(sim, PFIELD, T, DT; nsteps_plot=1)

        aux = PFIELD.nt/nsteps
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && disp_plot
            ax = axs1[1]
            ax.set_xlim([0,1])
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"$\frac{Cl}{CL}$")
            ax.title.set_text("Spanwise lift distribution")

            ax = axs1[2]
            ax.set_xlim([0,1])
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"$\frac{Cd}{CD}$")
            ax.title.set_text("Spanwise drag distribution")

            ax = axs1[3]
            ax.set_xlabel("Simulation time (s)")
            ax.set_ylabel(L"Lift Coefficient $C_L$")

            ax = axs1[4]
            ax.set_xlabel("Simulation time (s)")
            ax.set_ylabel(L"Drag Coefficient $C_D$")

            ax = axs2[1]
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"Circulation $\Gamma$")
            ax = axs2[2]
            ax.set_xlabel(L"$\frac{2y}{b}$")
            ax.set_ylabel(L"Effective velocity $V_\infty$")
        end

        if PFIELD.nt!=0 && PFIELD.nt%nsteps_plot==0 && disp_plot

            # Force at each VLM element
            Ftot = uns.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                                            rhoinf; t=PFIELD.t)
            L, D, S = uns.decompose(Ftot, [0,0,1], [-1,0,0])
            vlm._addsolution(wing, "L", L)
            vlm._addsolution(wing, "D", D)
            vlm._addsolution(wing, "S", S)

            # Force per unit span at each VLM element
            ftot = uns.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                        rhoinf; t=PFIELD.t, per_unit_span=true)
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

            ax = axs1[1]
            ax.plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
            ax.plot(y2b, ClCL, "-", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[2]
            ax.plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
            ax.plot(y2b, CdCD, "-", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[3]
            ax.plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
            ax.plot([T], [CLwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs1[4]
            ax.plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
            ax.plot([T], [CDwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            ax = axs2[1]
            ax.plot(y2b, wing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
            if wake_coupled && PFIELD.nt!=0
                ax = axs2[2]
                ax.plot(y2b, norm.(wing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                if VehicleType==uns.VLMVehicle
                    ax.plot(y2b, norm.(wing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                end
                ax.plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)],
                                                            "-k", label="FLOWVLM", alpha=0.5)
            end
        end

        prev_wing = deepcopy(wing)

        return false
    end


    # ------------- RUN SIMULATION ---------------------------------------------
    # Simulation setup
    Vref = Vcruise                  # Reference velocity
    RPMref = RPMh_w                 # Reference RPM
    ttot = telapsed                 # Total time to perform maneuver
    Vinit = Vref*Vaircraft(0)       # Initial vehicle velocity
                                    # Maximum number of particles
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                                    Vinit=Vinit)

    if verbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      surf_sigma=surf_sigma,
                                      vlm_rlx=vlm_rlx,
                                      vlm_init=vlm_init,
                                      max_particles=max_particles,
                                      wake_coupled=wake_coupled,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose2, v_lvl=v_lvl+1,
                                      save_horseshoes=!wake_coupled,
                                      sim_optargs...
                                      )


    # ------------- POST-PROCESSING --------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Postprocessing..."); end;
    # Simulation fift and drag
    vlm.calculate_field(wing, "CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
    info = vlm.fields_summary(wing)
    CLsim = info["CL"]
    CDsim = info["CD"]

    # Weber's experimental lift and drag (Table 4)
    CLexp = 0.238
    CDexp = 0.005

    # Error
    CLerr = abs(CLexp-CLsim)/CLexp
    CDerr = abs(CDexp-CDsim)/CDexp

    res = CLerr<tol && (CDerr<tol || true)

    if verbose
        t = "\t"^(v_lvl+1)
        @printf "%0s%10s\t%-11s\t%-11s\t%-11s\n"    t "PARAMETER"   "Experimental"  "Simulation"    "Error %"
        @printf "%0s%10s\t%11.5e\t%11.5e\t%11.5e\n" t "CL"          CLexp           CLsim           100*CLerr
        @printf "%0s%10s\t%11.5e\t%11.5e\t%11.5e\n" t "CD"          CDexp           CDsim           100*CDerr
        println("\t"^(v_lvl+1)*"TEST RESULT:\t$res")
    end

    return res
end
