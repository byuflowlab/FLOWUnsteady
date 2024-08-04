#=##############################################################################
# DESCRIPTION
    Simulation of a DJI 9450 rotor in hover in ground effect (two-bladed rotor, 9.4 inches
    diameter).

    This example replicates the experiment described in Zawodny & Boyd (2016),
    "Acoustic Characterization and Prediction of Representative,
    Small-scale Rotary-wing Unmanned Aircraft System Components."

# AUTHORSHIP
  * Author          : Ryan Anderson
  * Email           : rymanderson@gmail.com
  * Created         : Jul 2024
  * Last updated    : Jul 2024
  * License         : MIT
=###############################################################################

import Statistics: mean
import FLOWMath: Akima
const I = [1.0 0 0;0 1 0;0 0 1]
#=
Use the following parameters to obtain the desired fidelity

---- MID-LOW FIDELITY ---
n               = 20                        # Number of blade elements per blade
nsteps_per_rev  = 36                        # Time steps per revolution
p_per_step      = 4                         # Sheds per time step
sigma_rotor_surf= R/10                      # Rotor-on-VPM smoothing radius
vpm_integration = vpm.euler                 # VPM time integration scheme
vpm_SFS         = vpm.SFS_none              # VPM LES subfilter-scale model
shed_starting   = false                     # Whether to shed starting vortex
suppress_fountain    = true                 # Suppress hub fountain effect
sigmafactor_vpmonvlm = 1.0                  # Shrink particles by this factor when
                                            #  calculating VPM-on-VLM/Rotor induced velocities

---- MID-HIGH FIDELITY ---
n               = 50
nsteps_per_rev  = 72
p_per_step      = 2
sigma_rotor_surf= R/10
sigmafactor_vpmonvlm = 1.0
shed_starting   = false
suppress_fountain    = true
vpm_integration = vpm.rungekutta3
vpm_SFS         = vpm.SFS_none

---- HIGH FIDELITY -----
n               = 50
nsteps_per_rev  = 360
p_per_step      = 2
sigma_rotor_surf= R/80
sigmafactor_vpmonvlm = 5.5
shed_starting   = true
suppress_fountain    = false
vpm_integration = vpm.rungekutta3
vpm_SFS         = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive;
                                    alpha=0.999, maxC=1.0,
                                    clippings=[vpm.clipping_backscatter])
=#

import FLOWUnsteady as uns
using FLOWUnsteady.StaticArrays
import FLOWVLM as vlm
import FLOWVPM as vpm

paraview        = false # Whether to visualize with Paraview

# Uncomment this to have the folder named after this file instead
# save_path     = String(split(@__FILE__, ".")[1])
# run_name      = "singlerotor"
# paraview      = false
# ----------------- GEOMETRY PARAMETERS ----------------------------------------

# NOTE: If `xfoil=true`, XFOIL will be run to generate the airfoil polars used
#       by blade elements before starting the simulation. XFOIL is run
#       on the airfoil contours found in `rotor_file` at the corresponding
#       local Reynolds and Mach numbers along the blade.
#       Alternatively, the user can provide pre-computer airfoil polars using
#       `xfoil=false` and providing the polar files through `rotor_file`.
#       `read_polar` is the function that will be used to parse polar files. Use
#       `vlm.ap.read_polar` for files that are direct outputs of XFOIL (e.g., as
#       downloaded from www.airfoiltools.com). Use `vlm.ap.read_polar2` for CSV
#       files.

# Discretization

# NOTE: Here a geometric expansion of 1/10 means that the spacing between the
#       tip elements is 1/10 of the spacing between the hub elements. Refine the
#       discretization towards the blade tip like this in order to better
#       resolve the tip vortex.


# ----------------- SIMULATION PARAMETERS --------------------------------------

# Operating conditions
function run_quadrotor(RPM=6000.0,J=0.0001;
        run_name = "rotorground-example-20240801-07",      # Name of this simulation
        mirror          = false,
        x_ground        = 1,
        lR              = 2.7,
        use_actuator_line = true,
        no_tip_correction = false,
        n               = 40,                        # Number of blade elements per blade
        r               = 1/10,                      # Geometric expansion of elements
        # Rotor geometry
        rotor_file      = "DJI9450.csv",             # Rotor geometry
        data_path       = uns.def_data_path,         # Path to rotor database
        pitch           = 0.0,                       # (deg) collective pitch of blades
        CW              = false,                     # Clock-wise rotation
        xfoil           = false,                     # Whether to run XFOIL
        read_polar      = vlm.ap.read_polar2,        # What polar reader to use
        nrevs           = 10,                        # Number of revolutions in simulation
        nsteps_per_rev  = 144,                       # Time steps per revolution
    )
    save_path       = run_name                  # Where to save this simulation
    # RPM             = 5400                      # RPM
    # J               = 0.0001                    # Advance ratio Vinf/(nD)
    AOA             = 0                         # (deg) Angle of attack (incidence angle)

    rho             = 1.071778                  # (kg/m^3) air density
    mu              = 1.85508e-5                # (kg/ms) air dynamic viscosity
    speedofsound    = 342.35                    # (m/s) speed of sound

    # NOTE: For cases with zero freestream velocity, it is recommended that a
    #       negligible small velocity is used instead of zero in order to avoid
    #       potential numerical instabilities (hence, J here is negligible small
    #       instead of zero)

    # Read radius of this rotor and number of blades
    R, B            = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]

    magVinf         = J*RPM/60*(2*R)
    Vinf(X, t)      = magVinf*[cos(AOA*pi/180), sin(AOA*pi/180), 0]  # (m/s) freestream velocity vector

    ReD             = 2*pi*RPM/60*R * rho/mu * 2*R      # Diameter-based Reynolds number
    Matip           = 2*pi*RPM/60 * R / speedofsound    # Tip Mach number

    println("""
        RPM:    $(RPM)
        Vinf:   $(Vinf(zeros(3), 0)) m/s
        Matip:  $(round(Matip, digits=3))
        ReD:    $(round(ReD, digits=0))
    """)

    # ----------------- SOLVER PARAMETERS ------------------------------------------

    # Aerodynamic solver
    VehicleType     = uns.UVLMVehicle           # Unsteady solver
    # VehicleType     = uns.QVLMVehicle         # Quasi-steady solver
    const_solution  = VehicleType==uns.QVLMVehicle  # Whether to assume that the
                                                    # solution is constant or not
    # Time parameters
    nsteps          = const_solution ? 2 : nrevs*nsteps_per_rev # Number of time steps
    ttot            = nsteps/nsteps_per_rev / (RPM/60)       # (s) total simulation time

    # VPM particle shedding
    p_per_step      = 1                         # Sheds per time step
    shed_starting   = false                     # Whether to shed starting vortex
    shed_unsteady   = true                      # Whether to shed vorticity from unsteady loading
    unsteady_shedcrit = 0.001                   # Shed unsteady loading whenever circulation
                                                #  fluctuates by more than this ratio
    max_particles   = ((2*n+1)*B)*nsteps*p_per_step + 1 # Maximum number of particles

    # Regularization
    sigma_rotor_surf= R/10                      # Rotor-on-VPM smoothing radius
    lambda_vpm      = 2.125                     # VPM core overlap
                                                # VPM smoothing radius
    sigma_vpm_overwrite = lambda_vpm * 2*pi*R/(nsteps_per_rev*p_per_step)
    sigmafactor_vpmonvlm= 1.0                   # Shrink particles by this factor when
                                                #  calculating VPM-on-VLM/Rotor induced velocities

    # Rotor solver
    vlm_rlx         = 0.5                       # VLM relaxation <-- this also applied to rotors <-- no it doesn't
    hubtiploss_correction = no_tip_correction ? vlm.hubtiploss_nocorrection : ((0.4, 5, 0.1, 0.05), (2, 1, 0.25, 0.05)) # Hub and tip correction

    # VPM solver
    # vpm_integration = vpm.euler                 # VPM temporal integration scheme
    vpm_integration = vpm.rungekutta3

    vpm_viscous     = vpm.Inviscid()            # VPM viscous diffusion scheme
    # vpm_viscous   = vpm.CoreSpreading(-1, -1, vpm.zeta_fmm; beta=100.0, itmax=20, tol=1e-1)

    # vpm_SFS         = vpm.SFS_none              # VPM LES subfilter-scale model
    # vpm_SFS       = vpm.SFS_Cd_twolevel_nobackscatter
    # vpm_SFS       = vpm.SFS_Cd_threelevel_nobackscatter
    # vpm_SFS       = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive;
    #                                   alpha=0.999, maxC=1.0,
    #                                   clippings=[vpm.clipping_backscatter])
    # vpm_SFS       = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive;
    #                                   alpha=0.999, rlxf=0.005, minC=0, maxC=1
    #                                   clippings=[vpm.clipping_backscatter],
    #                                   controls=[vpm.control_sigmasensor],
    #                                   )
    vpm_SFS         = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive;
                                       alpha=0.999, maxC=1.0,
                                       clippings=[vpm.clipping_backscatter])

    # NOTE: In most practical situations, open rotors operate at a Reynolds number
    #       high enough that viscous diffusion in the wake is actually negligible.
    #       Hence, it does not make much of a difference whether we run the
    #       simulation with viscous diffusion enabled or not. On the other hand,
    #       such high Reynolds numbers mean that the wake quickly becomes turbulent
    #       and it is crucial to use a subfilter-scale (SFS) model to accurately
    #       capture the turbulent decay of the wake (turbulent diffusion).

    if VehicleType == uns.QVLMVehicle
        # Mute warnings regarding potential colinear vortex filaments. This is
        # needed since the quasi-steady solver will probe induced velocities at the
        # lifting line of the blade
        uns.vlm.VLMSolver._mute_warning(true)
    end



    # ----------------- WAKE TREATMENT ---------------------------------------------
    # NOTE: It is known in the CFD community that rotor simulations with an
    #       impulsive RPM start (*i.e.*, 0 to RPM in the first time step, as opposed
    #       to gradually ramping up the RPM) leads to the hub "fountain effect",
    #       with the root wake reversing the flow near the hub.
    #       The fountain eventually goes away as the wake develops, but this happens
    #       very slowly, which delays the convergence of the simulation to a steady
    #       state. To accelerate convergence, here we define a wake treatment
    #       procedure that suppresses the hub wake for the first three revolutions,
    #       avoiding the fountain effect altogether.
    #       This is especially helpful in low and mid-fidelity simulations.

    suppress_fountain   = true                  # Toggle

    # Supress wake shedding on blade elements inboard of this r/R radial station
    no_shedding_Rthreshold = suppress_fountain ? 0.35 : 0.0

    # Supress wake shedding for this many time steps
    no_shedding_nstepsthreshold = 3*nsteps_per_rev

    omit_shedding = []          # Index of blade elements to supress wake shedding

    # Function to suppress or activate wake shedding
    function wake_treatment_supress(sim, args...; optargs...)

        # Case: start of simulation -> suppress shedding
        if sim.nt == 1

            # Identify blade elements on which to suppress shedding
            for i in 1:vlm.get_m(rotor)
                HS = vlm.getHorseshoe(rotor, i)
                CP = HS[5]

                if uns.vlm.norm(CP - vlm._get_O(rotor)) <= no_shedding_Rthreshold*R
                    push!(omit_shedding, i)
                end
            end
        end

        # Case: sufficient time steps -> enable shedding
        if sim.nt == no_shedding_nstepsthreshold

            # Flag to stop suppressing
            omit_shedding .= -1

        end

        return false
    end

    # try another method
    Pmin = SVector{3}(-R/8, -10*R, -10*R)
    Pmax = SVector{3}(x_ground*R, 10*R, 10*R)
    wake_treatment_box = uns.remove_particles_box(Pmin, Pmax, 1)

    # ----------------- 1) VEHICLE DEFINITION --------------------------------------
    println("Generating geometry...")

    # Generate rotor
    if use_actuator_line
        rotor1 = uns.generate_rotor(rotor_file; pitch=pitch,
                                                n=n, CW=CW, blade_r=r,
                                                altReD=[RPM, J, mu/rho],
                                                xfoil=xfoil,
                                                read_polar=read_polar,
                                                data_path=data_path,
                                                verbose=true,
                                                plot_disc=true
                                                )
        vlm.setcoordsystem(rotor1, [0.0, lR/2*R, lR/2*R], I; user=true)

        rotor2 = uns.generate_rotor(rotor_file; pitch=pitch,
                                                n=n, CW=!CW, blade_r=r,
                                                altReD=[RPM, J, mu/rho],
                                                xfoil=xfoil,
                                                read_polar=read_polar,
                                                data_path=data_path,
                                                verbose=true,
                                                plot_disc=true
                                                )
        vlm.setcoordsystem(rotor2, [0.0, -lR/2*R, lR/2*R], I; user=true)
        rotor3 = deepcopy(rotor1)
        vlm.setcoordsystem(rotor3, [0.0, -lR/2*R, -lR/2*R], I; user=true)
        rotor4 = deepcopy(rotor2)
        vlm.setcoordsystem(rotor4, [0.0, lR/2*R, -lR/2*R], I; user=true)
    else
        r = [0.00624, 0.011928000000000001, 0.017616, 0.023304000000000002, 0.028992, 0.03468, 0.04036800000000001, 0.04605600000000001, 0.051744000000000005, 0.057432000000000004, 0.06312, 0.068808, 0.074496, 0.08018399999999999, 0.085872, 0.09155999999999999, 0.097248, 0.102936, 0.108624, 0.11431199999999998, 0.12]
        chord = [0.015859659042058712, 0.021712836336990532, 0.026471916314454773, 0.030461073429155164, 0.0317971438150289, 0.030501994764606518, 0.02869564736260471, 0.026709524982770225, 0.02491417716322731, 0.02312146444774523, 0.02143149575711966, 0.020040817717748644, 0.018694708765967558, 0.017317579753634096, 0.01608328099021666, 0.014994713447804647, 0.014066063848378024, 0.01317730489959203, 0.012217298822173369, 0.011285945627012788, 0.005859876]
        theta = [15.66104986554918, 18.723913333871426, 19.68216955539137, 19.7955, 19.035941192138505, 18.146870653685674, 16.97244973068665, 15.937117364119175, 15.013842635490459, 13.98485597885306, 13.028560772939581, 12.138592707201528, 11.252774816377263, 10.155109460976346, 9.227215187155704, 8.417084274414362, 7.560389846316106, 7.067530936484768, 6.630681416402278, 6.385145970694592, 5.40892]
        LE_x = -[0.008008028901338822, 0.009373371261750517, 0.010691634057751948, 0.0118083411417151, 0.012276957660372158, 0.011916237936642851, 0.011561576082791573, 0.010990988452101998, 0.010546096584594886, 0.01000168687528428, 0.0094526398949714, 0.009182786644796692, 0.008842820629436492, 0.00842332560336312, 0.008059186232663855, 0.007827501376214652, 0.007741670297121591, 0.007587253545692625, 0.00731281033391528, 0.007140246456237078, 0.00288816]
        LE_z = [-0.0016390575925383636, -0.0008641410800689035, 2.7521759999999995e-5, 0.0007823152971428572, 0.0013627207407049735, 0.001227285398358281, 0.000636376992045275, 6.635751803655285e-5, -0.0004584889446266324, -0.0008575595243038399, -0.001122961413507112, -0.0013143246818426242, -0.0015087701279981286, -0.001778251911079911, -0.0019953256778521777, -0.0021722641545572096, -0.0023491980231992423, -0.002526870008190008, -0.0027065319246519248, -0.002827114402806406, -0.0029155799999999996]

        # airfoil twist correction
        rR_unaltered = [0.0, 0.0857143, 0.185714, 0.371429, 0.714286, 0.942857, 1.0]
        rR_unaltered .*= r[end]-r[1]
        rR_unaltered .+= r[1]
        zero_lift_aoa = [0.376, 0.949, 0.253, 0.354, -0.626, 0.285, -0.328] # maybe doesn't need a twist correction?
        zero_lift_spline = Akima(rR_unaltered, zero_lift_aoa)
        for (i,ri) in enumerate(r)
            zero_aoa = zero_lift_spline(ri)
            # theta[i] -= zero_aoa
        end

        # create blades
        blade1 = vlm.Wing(LE_x[1], r[1], LE_z[1], chord[1], theta[1])
        n = 1
        for i in 2:length(r)
        	vlm.addchord(blade1, LE_x[i], r[i], LE_z[i], chord[i], theta[i], n)
        end
        vlm.save(blade1, "test_blade1"; save_horseshoes=false)
        blade2 = deepcopy(blade1)

        vlm.setcoordsystem(blade1, [0.0,0,0], [0 0 1.0; 0 1 0; -1 0 0])
        vlm.setcoordsystem(blade2, [0.0,0,0], [0 0 -1.0; 0 -1 0; -1 0 0])

        rotor = vlm.WingSystem()
        vlm.addwing(rotor, "blade1", blade1)
        vlm.addwing(rotor, "blade2", blade2)

    end

    println("Generating vehicle...")

    # Generate vehicle
    system = vlm.WingSystem()                   # System of all FLOWVLM objects
    for (i,rotor) in enumerate((rotor1, rotor2, rotor3, rotor4))
        vlm.addwing(system, "Rotor$i", rotor)
    end

    vlm_system = vlm.WingSystem()
    if use_actuator_line
        rotors = [rotor1, rotor2, rotor3, rotor4];                           # Defining this rotor as its own system
        rotor_systems = (rotors, );                 # All systems of rotors
    else
        rotor_systems = NTuple{0, Array{vlm.Rotor, 1}}()
        for (i,rotor) in enumerate((rotor1, rotor2, rotor3, rotor4))
            vlm.addwing(vlm_system, "Rotor$i", rotor)
        end
    end

    wake_system = vlm.WingSystem()              # System that will shed a VPM wake
                                                # NOTE: Do NOT include rotor when using the quasi-steady solver
    if VehicleType != uns.QVLMVehicle
        for (i,rotor) in enumerate((rotor1, rotor2, rotor3, rotor4))
            vlm.addwing(wake_system, "Rotor$i", rotor)
        end
    end

    vehicle = VehicleType(   system;
                                vlm_system=vlm_system,
                                rotor_systems=rotor_systems,
                                wake_system=wake_system
                             );


    # ------------- 2) MANEUVER DEFINITION -----------------------------------------
    # Non-dimensional translational velocity of vehicle over time
    Vvehicle(t) = zeros(3)

    # RPM control input over time (RPM over `RPMref`)
    function RPMcontrol(t)
        tcorner = 0.2
        if t < tcorner
            return t / tcorner
        else
            return 1.0
        end
    end

    angles = ()                                 # Angle of each tilting system (none)
    dt = ttot / nsteps
    #if use_actuator_line
        RPMs = (RPMcontrol, )                       # RPM of each rotor system

        # Angle of the vehicle over time
        anglevehicle(t) = zeros(3)
    #else
    #    RPMs = ()
    #    function anglevehicle(t)
    #        t_actual = t * ttot
    #        t_track = 0.0
    #        i_step = Int(round(t_actual / dt))
    #        angle = 0.0
    #        for i in 0:i_step-1
    #            rpm = RPMcontrol(i/nsteps)
    #            angle += rpm * RPM * 360 / 60 * dt
    #        end
    #        return [-angle,0,0.0]
    #    end
    #end

    maneuver = uns.KinematicManeuver(angles, RPMs, Vvehicle, anglevehicle)


    # ------------- 3) SIMULATION DEFINITION ---------------------------------------

    Vref = 0.0                                  # Reference velocity to scale maneuver by
    RPMref = RPM                                # Reference RPM to scale maneuver by
    Vinit = Vref*Vvehicle(0)                    # Initial vehicle velocity
    Winit = pi/180*(anglevehicle(1e-6) - anglevehicle(0))/(1e-6*ttot)  # Initial angular velocity

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                        Vinit=Vinit, Winit=Winit);

    # define ground
    mirror_X = SVector{3}(x_ground*R,0.0,0.0)
    mirror_normal = SVector{3}(-1.0,0,0)

    # Restart simulation
    # restart_file = nothing

    # NOTE: Uncomment the following line to restart a previous simulation.
    #       Point it to a particle field file (with its full path) at a specific
    #       time step, and `run_simulation` will start this simulation with the
    #       particle field found in the restart simulation.

    # restart_file = "rotorhover-example_pfield.50"
    restart_file = nothing


    # ------------- 4) MONITORS DEFINITIONS ----------------------------------------

    # Generate rotor monitor
    if use_actuator_line
        monitor_rotor = uns.generate_monitor_rotors(rotors, J, rho, RPM, nsteps;
                                                    t_scale=RPM/60,        # Scaling factor for time in plots
                                                    t_lbl="Revolutions",   # Label for time axis
                                                    save_path=save_path,
                                                    run_name=run_name,
                                                    figname="rotor monitor",
                                                    )
    else
        monitor_rotor = uns.generate_monitor_rotors(rotors, J, rho, RPM, Vinf, nsteps;
                                                    N_dir=[0,0,1], T_dir=[1,0,0],
                                                    t_scale=RPM/60,
                                                    t_lbl="Revolutions",
                                                    save_path, run_name, figname="VLM rotor monitor")
    end

    # Generate monitor of flow enstrophy (numerical stability)
    monitor_enstrophy = uns.generate_monitor_enstrophy(;
                                                save_path=save_path,
                                                run_name=run_name,
                                                figname="enstrophy monitor"
                                                )

    # Generate monitor of SFS model coefficient Cd
    monitor_Cd = uns.generate_monitor_Cd(;
                                                save_path=save_path,
                                                run_name=run_name,
                                                figname="Cd monitor"
                                                )
    # Concatenate monitors
    monitors = uns.concatenate(monitor_rotor, monitor_enstrophy, monitor_Cd)


    # ------------- 5) RUN SIMULATION ----------------------------------------------
    println("Running simulation...")

    # Concatenate monitors and wake treatment procedure into one runtime function
    # runtime_function = uns.concatenate(monitors, wake_treatment_supress)
    runtime_function = mirror ? uns.concatenate(monitors, wake_treatment_box) : monitors
    # runtime_function = monitors

    # Run simulation
    uns.run_simulation(simulation, nsteps;
                        # ----- SIMULATION OPTIONS -------------
                        Vinf=Vinf,
                        rho=rho, mu=mu, sound_spd=speedofsound,
                        # ----- SOLVERS OPTIONS ----------------
                        p_per_step=p_per_step,
                        max_particles=max_particles,
                        vpm_integration=vpm_integration,
                        vpm_viscous=vpm_viscous,
                        vpm_SFS=vpm_SFS,
                        sigma_vlm_surf=sigma_rotor_surf,
                        sigma_rotor_surf=sigma_rotor_surf,
                        sigma_vpm_overwrite=sigma_vpm_overwrite,
                        sigmafactor_vpmonvlm=sigmafactor_vpmonvlm,
                        vlm_rlx=vlm_rlx,
                        hubtiploss_correction=hubtiploss_correction,
                        shed_starting=shed_starting,
                        shed_unsteady=shed_unsteady,
                        unsteady_shedcrit=unsteady_shedcrit,
                        omit_shedding=omit_shedding,
                        extra_runtime_function=runtime_function,
                        mirror, mirror_X, mirror_normal,
                        # ----- RESTART OPTIONS -----------------
                        restart_vpmfile=restart_file,
                        # ----- OUTPUT OPTIONS ------------------
                        save_path=save_path,
                        run_name=run_name,
                        save_wopwopin=false,  # <--- Generates input files for PSU-WOPWOP noise analysis
                        save_code=@__FILE__
                        );

end


# ----------------- 6) VISUALIZATION -------------------------------------------
if paraview
    println("Calling Paraview...")

    # Files to open in Paraview
    files = joinpath(save_path, run_name*"_pfield...xmf;")
    for bi in 1:B
        global files
        files *= run_name*"_Rotor_Blade$(bi)_loft...vtk;"
        files *= run_name*"_Rotor_Blade$(bi)_vlm...vtk;"
    end

    # Call Paraview
    run(`paraview --data=$(files)`)

end

# RPM = 6000.0
# J = 0.0001
# run_quadrotor(RPM,J;
#         run_name = "quadrotor-example-20240803-01",      # Name of this simulation
#         mirror          = false,
#         x_ground        = 1,
#         lR              = 2.7,
#         use_actuator_line = true,
#         no_tip_correction = false,
#         nrevs= 30,
#         nsteps_per_rev = 144,
#     )

# ------------- 6) POSTPROCESSING ----------------------------------------------

# Post-process monitor plots
# include(joinpath(uns.examples_path, "rotorhover", "rotorhover_postprocessing.jl"))
