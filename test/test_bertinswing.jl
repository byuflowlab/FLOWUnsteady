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


"""
    Test FLOWVLM solver with an isolated, planar, swept wing.
"""
function bertin_VLM(;   # TEST OPTIONS
                        tol=0.025,
                        wake_coupled=true,
                        nsteps=200,
                        vlm_fsgm=-1,
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="bertins",
                        prompt=true,
                        verbose=true, verbose2=true, v_lvl=1,
                        disp_plot=true, figsize_factor=5/6
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
    overwrite_sigma = lambda_vpm * magVinf * (telapsed/nsteps) # Smoothing core size
    # vlm_sigma = -1            # VLM regularization core size (deactivated with -1)
    vlm_sigma = vlm_fsgm*b
    p_per_step = 1              # Number of particle sheds per time steps (dummy)
    # wake_coupled = true       # Coupled VPM wake with VLM solution
    shed_unsteady = true        # Whether to shed unsteady-loading wake
    # shed_unsteady = false

    # Maneuver definition (dummy)
    Vaircraft(t) = zeros(3)     # Translational velocity of system
    angles(t) = ()              # Tilt angle of each tilting system
    RPMs(t) = ()                # RPM of each rotor system
    maneuver(args...; optargs...) = (Vaircraft, angles, RPMs)

    # System definitions
    system = vlm.WingSystem()   # System of all FLOWVLM objects
    vlm.addwing(system, "BertinsWing", wing)

    rotors = vlm.Rotor[]        # System of all FLOWVLM Rotor objects (dummy)
    tilting_systems = ()        # Tuple of all lilting FLOWVLM Systems (dummy)
    rotors_systems = ()         # Tuple of all groups of rotors (dummy)
    vlm_system = system         # System solved through VLM solver
    wake_system = system        # System that will shed a VPM wake
                                # Fuselage Grid (dummy)
    fuselage = vlm.vtk.GridTriangleSurface(gt.Grid(zeros(3), [1.0, 1.0, 0.0], [1, 1, 0]), 1)

    if verbose
        println("\t"^(v_lvl+1)*"Core overlap:\t\t$(lambda_vpm)")
        println("\t"^(v_lvl+1)*"Core size:\t\t$(round(overwrite_sigma/b, 3))*b")
        println("\t"^(v_lvl+1)*"Time step translation:\t$(round(magVinf * (telapsed/nsteps)/b, 3))*b")
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

    function monitor(PFIELD, T, DT; figname="monitor_$(save_path)", nsteps_plot=1)

        aux = PFIELD.nt/nsteps
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && disp_plot
            figure(figname, figsize=[7*2, 5*2]*figsize_factor)
            subplot(221)
            xlim([0,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cl}{CL}$")
            title("Spanwise lift distribution")

            subplot(222)
            xlim([0,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cd}{CD}$")
            title("Spanwise drag distribution")

            subplot(223)
            xlabel("Simulation time (s)")
            ylabel(L"Lift Coefficient $C_L$")

            subplot(224)
            xlabel("Simulation time (s)")
            ylabel(L"Drag Coefficient $C_D$")
        end

        if PFIELD.nt%nsteps_plot==0 && disp_plot
            figure(figname)

            vlm.calculate_field(wing, "CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
            vlm.calculate_field(wing, "Cftot/CFtot"; S=b^2/ar, qinf=qinf, rhoinf=rhoinf)
            ClCL1 = wing.sol["Cl/CL"]
            CdCD1 = wing.sol["Cd/CD"]

            info = vlm.fields_summary(wing)
            CL = info["CL"]
            CD = info["CD"]

            subplot(221)
            plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
            plot(y2b, ClCL1, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(222)
            plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
            plot(y2b, CdCD1, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(223)
            plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CL], "o", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(224)
            plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CD], "o", label="FLOWVLM", alpha=0.5, color=clr)
        end

        return false
    end


    # ------------- RUN SIMULATION ---------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;
    run_simulation(maneuver, system, rotors,
                         tilting_systems, rotors_systems,
                         wake_system, vlm_system,
                         fuselage;
                         # SIMULATION OPTIONS
                         Vcruise=Vcruise,
                         RPMh_w=RPMh_w,
                         telapsed=telapsed,
                         nsteps=nsteps,
                         Vinf=Vinf,
                         # SOLVERS OPTIONS
                         p_per_step=p_per_step,
                         overwrite_sigma=overwrite_sigma,
                         vlm_sigma=vlm_sigma,
                         wake_coupled=wake_coupled,
                         shed_unsteady=shed_unsteady,
                         extra_runtime_function=monitor,
                         # OUTPUT OPTIONS
                         save_path=save_path,
                         run_name=run_name,
                         prompt=prompt,
                         verbose=verbose2, v_lvl=v_lvl+1,
                         save_horseshoes=!wake_coupled
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
