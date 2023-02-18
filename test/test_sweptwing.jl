#=##############################################################################
# DESCRIPTION
    45deg swept-back wing at an angle of attack of 4.2deg. This wing has an
    aspect ratio of 5.0, a RAE 101 airfoil section with 12% thickness, and no
    dihedral, twist, nor taper. This test case matches the experimental setup
    of Weber, J., and Brebner, G., “Low-Speed Tests on 45-deg Swept-Back Wings,
    Part I,” Tech. rep., 1951. The same case is used in a VLM calculation in
    Bertin's Aerodynamics for Engineers, Example 7.2, pp. 343.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

function run_sweptwing(testname;
                        # -------- TEST SETTINGS -------------------------------
                        kinematic   = false,        # If true, freestream is zero and aircraft moves
                        nsteps      = 150,          # Number of time steps
                        p_per_step  = 1,            # Number of particle sheds per time steps
                        fsgm_vlm    = -1,           # VLM-on-VLM smoothing factor
                        fsgm_surf   = 0.05,         # VLM-on-VPM smoothing factor
                        wake_coupled= true,         # Couple VPM wake and VLM solution
                        VehicleType = uns.UVLMVehicle, # Vehicle type
                        tol_CL      = 0.025,        # CL error tolerance
                        tol_CD      = 0.100,        # CD error tolerance
                        # -------- VERBOSE SETTINGS ----------------------------
                        verbose=true, simverbose=false, v_lvl=0,
                        prompt=true,
                        # -------- OUTPUT OPTIONS ------------------------------
                        save_path=nothing,
                        run_name="test-sweptwing",
                        disp_plot=false,
                        optargs...
                        )

    if verbose; println("\t"^(v_lvl)*testname); end;

    # ----------------- SIMULATION PARAMETERS ----------------------------------
    AOA             = 4.2                           # (deg) angle of attack
    magVinf         = 163*0.3048                    # (m/s) freestream velocity
    rho             = 0.93                          # (kg/m^3) air density
    qinf            = 0.5*rho*magVinf^2             # (Pa) static pressure

                                                    # Freestream and vehicle velocity
    if kinematic        # Case: No freestream, aircraft moves through air
        freestream  = 1e-12*[1, 0, 0]
        Vaircraft   = magVinf
    else                # Case: Freestream, aircraft static
        freestream  = magVinf*[1, 0, 0]
        Vaircraft   = 0.0
    end
    Vinf(X, t)      = t==0 ? magVinf*[1, 0, 0] : freestream # Freestream over time and space

    # ----------------- GEOMETRY PARAMETERS ------------------------------------
    b               = 98*0.0254                     # (m) span length
    ar              = 5.0                           # Aspect ratio b/c_tip
    tr              = 1.0                           # Taper ratio c_tip/c_root
    twist_root      = 0.0                           # (deg) twist at root
    twist_tip       = 0.0                           # (deg) twist at tip
    lambda          = 45.0                          # (deg) sweep
    gamma           = 0.0                           # (deg) dihedral

    # Discretization
    n               = 64                            # Number of horseshoes
    r               = 12.0                          # Geometric expansion
    central         = false                         # Central expansion

    # ----------------- SOLVER PARAMETERS --------------------------------------
    # Time parameters
    wakelength      = 2*b               # (m) length of wake that is resolved
    ttot            = wakelength/magVinf# (s) time in which to perform maneuver

    # VLM and VPM parameters
    vpm_lambda      = 2.0               # VPM core overlap
    sigma_vpm_overwrite = vpm_lambda * magVinf * (ttot/nsteps)/p_per_step # Smoothing core size
    sigma_vlm_solver= fsgm_vlm*b        # VLM-on-VLM smoothing radius (deactivated with <0)
    sigma_vlm_surf  = fsgm_surf*b       # VLM-on-VPM smoothing radius
    vlm_init        = true              # Initialize with VLM semi-infinite wake solution

    # ----------------- GENERATE GEOMETRY --------------------------------------
    if simverbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist_root, lambda, gamma;
                            twist_tip=twist_tip, n=n, r=r, central=central)

    # Pitch wing to corresponding angle of attack
    O = zeros(3)                        # Coordinate system origin
    Oaxis = gt.rotation_matrix2(0.0, -AOA, 0.0) # Coordinate system axes
    vlm.setcoordsystem(wing, O, Oaxis)

    # ------------- SIMULATION SETUP -------------------------------------------
    if simverbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    # Vehicle definition
    system = vlm.WingSystem()   # System of all FLOWVLM objects
    vlm.addwing(system, "Wing", wing)

    vlm_system = system         # System solved through VLM solver
    wake_system = system        # System that will shed a VPM wake

    vehicle = VehicleType(   system;
                                vlm_system=vlm_system,
                                wake_system=wake_system
                             )

    # Maneuver definition
    # NOTE: The velocity defined here is non-dimensional
    Vvehicle(t) = kinematic ? [-1, 0, 0] : zeros(3) # Translational velocity of vehicle over time
    anglevehicle(t) = zeros(3)  # Angle of the vehicle over time

    angle = ()                  # Angle of each tilting system (none)
    RPM = ()                    # RPM of each rotor system (none)

    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

    # Simulation definition
    Vref = Vaircraft                # Reference velocity to scale maneuver
    RPMref = 0.0                    # Reference RPM to scale maneuver
    Vinit = Vref*Vvehicle(0)        # Initial vehicle velocity
                                    # Maximum number of particles
    max_particles = ceil(Int, (nsteps+1)*(vlm.get_m(vehicle.vlm_system)*(p_per_step+1) + p_per_step))

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot; Vinit=Vinit)


    # ------------- DEFINE MONITORS --------------------------------------------

    # Generate function that calculates aerodynamic forces
    # NOTE: We exclude skin friction since we want to compare to the experimental
    #       data reported in Weber 1951 that was measured with pressure taps
    calc_aerodynamicforce_fun = uns.generate_calc_aerodynamicforce(;
                                        add_parasiticdrag=true,
                                        add_skinfriction=false,
                                        airfoilpolar="xf-rae101-il-1000000.csv"
                                        )

    monitor_wing = uns.generate_monitor_wing(wing, Vinf, b, ar,
                                                rho, qinf, nsteps;
                                                calc_aerodynamicforce_fun=calc_aerodynamicforce_fun,
                                                L_dir=[0,0,1],      # Direction of lift component
                                                D_dir=[1,0,0],      # Direction of drag component
                                                save_path=save_path,
                                                run_name=run_name,
                                                figname=testname,
                                                disp_plot=disp_plot,
                                                figsize_factor=5/6,
                                                nsteps_plot=1,
                                                nsteps_savefig=1
                                                )

    # ------------- RUN SIMULATION ---------------------------------------------
    if simverbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;

    pfield = uns.run_simulation(simulation, nsteps;
                                    # SIMULATION OPTIONS
                                    Vinf=Vinf,
                                    # SOLVERS OPTIONS
                                    p_per_step=p_per_step,
                                    vlm_init=vlm_init,
                                    wake_coupled=wake_coupled,
                                    sigma_vlm_solver=sigma_vlm_solver,
                                    sigma_vlm_surf=sigma_vlm_surf,
                                    sigma_rotor_surf=sigma_vlm_surf,
                                    sigma_vpm_overwrite=sigma_vpm_overwrite,
                                    max_particles=max_particles,
                                    extra_runtime_function=monitor_wing,
                                    # OUTPUT OPTIONS
                                    save_path=save_path,
                                    run_name=run_name,
                                    prompt=prompt,
                                    raisewarnings=false,
                                    verbose=simverbose, v_lvl=v_lvl+1,
                                    save_horseshoes=!wake_coupled,
                                    optargs...
                                    )


    # ------------- POST-PROCESSING --------------------------------------------
    if simverbose; println("\n"*"\t"^(v_lvl+1)*"Postprocessing..."*"\n"); end;

    # Total lift and drag
    L = sum(wing.sol["L"])
    D = sum(wing.sol["D"])

    # Lift and drag coefficients
    CL = uns.norm(L) / (qinf*b^2/ar)
    CD = uns.norm(D) / (qinf*b^2/ar)

    # Weber's experimental lift and drag (Table 4)
    CLexp = 0.238
    CDexp = 0.005

    # Error
    CLerr = abs(CLexp-CL)/CLexp
    CDerr = abs(CDexp-CD)/CDexp

    res = CLerr<tol_CL && CDerr<tol_CD

    if verbose
        t = "\t"^(v_lvl+1)
        @printf "%0s%10s\t%-11s\t%-11s\t%7s\n"    t "PARAMETER"   "Experimental"  "  FLOWUnsteady"    "Error"
        @printf "%0s%10s\t%11.4f\t%11.5f\t%7.2f ﹪\n" t "CL"          CLexp           CL              100*CLerr
        @printf "%0s%10s\t%11.4f\t%11.5f\t%7.2f ﹪\n" t "CD"          CDexp           CD              100*CDerr
        println("\t"^(v_lvl+1)*"TEST RESULT:\t$res\n")
    end

    return res

end
