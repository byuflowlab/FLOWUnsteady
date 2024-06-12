#=##############################################################################
# DESCRIPTION
    45° swept-back wing at an angle of attack of 4.2°. This wing has an aspect
    ratio of 5.0, a RAE 101 airfoil section with 12% thickness, and no dihedral,
    twist, nor taper. This test case matches the experimental setup of Weber,
    J., and Brebner, G., “Low-Speed Tests on 45-deg Swept-Back Wings, Part I,”
    Tech. rep., 1951. The same case is used in a VLM calculation in Bertin's
    Aerodynamics for Engineers, Example 7.2, pp. 343.

# AUTHORSHIP
  * Author          : Eduardo J. Alvarez (edoalvarez.com)
  * Email           : Edo.AlvarezR@gmail.com
  * Created         : Feb 2023
  * Last updated    : Feb 2023
  * License         : MIT
=###############################################################################

using Pkg
this_dir = @__DIR__
FLOWUnsteady_dir = normpath(joinpath(this_dir,"../.."))
Pkg.activate(FLOWUnsteady_dir)

using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm

# construct VLM system
function construct_vlm_system(; ns=20, nc=1)
    halfspan = 2.489/2
    aspect_ratio = 5.0
    c0 = 2*halfspan/aspect_ratio
    yle = [0.0, halfspan]
    xle = [0.0, halfspan]
    zle = [0.0, 0.0]
    chord = [c0, c0]
    theta = [0.0, 0.0]
    phi = [0.0, 0.0]

    grid, surface = vlm.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
            mirror=true, spacing_s=vlm.Sine(), spacing_c=vlm.Uniform(), fcore=(c,Δs) -> 0.001
        )

    surfaces = [surface]
    ref = vlm.Reference(1.0, 1.0, 1.0, SVector{3}(halfspan/2+c0/2, 0.0, 0.0), 1.0)
    fs = vlm.Freestream(0.0, 0.0, 0.0, SVector{3}(0.0,0.0,0.0), 1.0)

    system = vlm.steady_analysis(surfaces, ref, fs; symmetric=false)
    system.trailing_vortices .= false

    return system
end

function construct_simulation(time_range, alpha;
        preprocessor=DefaultPreprocessor(),
        integrator=ForwardEuler(),
        ns=20, nc=1, dynamic=false,
        orientation=zero(Quaternion{Float64}),
        velocity=zero(SVector{3,Float64}),
        save_steps=collect(range(0,length(time_range)-1)),
        omit_fields=(),
        vinf = 49.7,
        Quasisteady=false,
        model_kwargs...
    )

    # simulation parameters
    Re = 1.6e6
    freestream=SimpleFreestream(vinf * SVector{3}(-cos(alpha),0,-sin(alpha))) # in North-East-down coordinates

    # construct vehicle
    vlm_system = construct_vlm_system(; ns, nc)
    model = VortexLatticeModel(vlm_system; Quasisteady, max_timesteps=length(time_range), model_kwargs...)
    vehicle = RigidBodyVehicle(model; dynamic, orientation, velocity, model_coordinates=Aerodynamics(), vehicle_coordinates=FlightDynamics())
    controller = PrescribedKinematics()

    # postprocessor
    history = History(vehicle, controller, save_steps; omit_fields)
    paraview = ParaviewOutput(save_steps)
    postprocessor = MultiPostprocessor((history, paraview))

    # create simulation object
    sim = Simulation(vehicle, time_range;
        freestream, controller, preprocessor, integrator, postprocessor
    )

    return sim
end

function run_simulation(time_range, alpha, run_name, path; kwargs...)
    sim = construct_simulation(time_range, alpha; kwargs...)
    simulate!(sim, time_range; run_name, path)
    return sim
end

save_path = normpath(joinpath(this_dir,"results"))
!isdir(save_path) && mkdir(save_path)
model_kwargs = (eta_wake=1.0, overlap_trailing=0.4, FMM=false, Quasisteady=false, ns=10,nc=5, p_per_step_trailing=1, p_per_step_unsteady=1, threshold_unsteady_gamma_min=0.001)
sim = run_simulation(range(0.0,stop=0.1,length=2), deg2rad(4.2), "wing", save_path; model_kwargs...)
# sim = run_simulation(range(0.0,step=0.001,length=151), deg2rad(4.2), "wing", save_path; model_kwargs...)

# import FLOWUnsteady as uns
# import FLOWVLM as vlm

# run_name        = "wing-example"            # Name of this simulation

# save_path       = run_name                  # Where to save this simulation
# paraview        = true                      # Whether to visualize with Paraview


# # ----------------- SIMULATION PARAMETERS --------------------------------------
# AOA             = 4.2                       # (deg) angle of attack
# magVinf         = 49.7                      # (m/s) freestream velocity
# rho             = 0.93                      # (kg/m^3) air density
# qinf            = 0.5*rho*magVinf^2         # (Pa) static pressure

# Vinf(X, t)      = magVinf*[cosd(AOA), 0.0, sind(AOA)]  # Freestream function


# # ----------------- GEOMETRY PARAMETERS ----------------------------------------
# b               = 2.489                     # (m) span length
# ar              = 5.0                       # Aspect ratio b/c_tip
# tr              = 1.0                       # Taper ratio c_tip/c_root
# twist_root      = 0.0                       # (deg) twist at root
# twist_tip       = 0.0                       # (deg) twist at tip
# lambda          = 45.0                      # (deg) sweep
# gamma           = 0.0                       # (deg) dihedral

# # Discretization
# n               = 50                        # Number of spanwise elements per side
# r               = 10.0                      # Geometric expansion of elements
# central         = false                     # Whether expansion is central

# # NOTE: A geometric expansion of 10 that is not central means that the last
# #       element is 10 times wider than the first element. If central, the
# #       middle element is 10 times wider than the peripheral elements.

# # ----------------- SOLVER PARAMETERS ------------------------------------------
# # Time parameters
# wakelength      = 2.75*b                    # (m) length of wake to be resolved
# ttot            = wakelength/magVinf        # (s) total simulation time
# nsteps          = 200                       # Number of time steps

# # VLM and VPM parameters
# p_per_step      = 1                         # Number of particle sheds per time step

# lambda_vpm      = 2.0                       # VPM core overlap
# sigma_vpm_overwrite = lambda_vpm * magVinf * (ttot/nsteps)/p_per_step # Smoothing core size
# sigma_vlm_solver= -1                        # VLM-on-VLM smoothing radius (deactivated with <0)
# sigma_vlm_surf  = 0.05*b                    # VLM-on-VPM smoothing radius

# shed_starting   = true                      # Whether to shed starting vortex
# vlm_rlx         = 0.7                       # VLM relaxation




# # ----------------- 1) VEHICLE DEFINITION --------------------------------------
# println("Generating geometry...")

# # Generate wing
# wing = vlm.simpleWing(b, ar, tr, twist_root, lambda, gamma;
#                         twist_tip=twist_tip, n=n, r=r, central=central);

# # NOTE: `FLOWVLM.simpleWing` is an auxiliary function in FLOWVLM for creating a
# #       VLM wing with constant sweep, dihedral, and taper ratio, and a linear
# #       twist between the root and the wing tips

# println("Generating vehicle...")

# # Generate vehicle
# system = vlm.WingSystem()                   # System of all FLOWVLM objects
# vlm.addwing(system, "Wing", wing)

# vlm_system = system;                        # System solved through VLM solver
# wake_system = system;                       # System that will shed a VPM wake

# vehicle = uns.VLMVehicle(   system;
#                             vlm_system=vlm_system,
#                             wake_system=wake_system
#                          );

# # NOTE: `FLOWUnsteady.VLMVehicle` creates a vehicle made out of multiple VLM and
# #       rotor subsystems. The argument `system` represents the vehicle as a
# #       whole which will be translated and rotated with the kinematics
# #       prescribed by the maneuver. The subsystem `vlm_system` will be solved
# #       with the VLM solver. The subsystems `rotor_systems` are solved with
# #       blade elements (none in this case). The subsystem `wake_system` will
# #       shed the VPM wake.




# # ------------- 2) MANEUVER DEFINITION -----------------------------------------

# Vvehicle(t) = zeros(3)                      # Translational velocity of vehicle over time
# anglevehicle(t) = zeros(3)                  # Angle of the vehicle over time

# angle = ()                                  # Angle of each tilting system (none)
# RPM = ()                                    # RPM of each rotor system (none)

# maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

# # NOTE: `FLOWUnsteady.KinematicManeuver` defines a maneuver with prescribed
# #       kinematics. `Vvehicle` defines the velocity of the vehicle (a vector)
# #       over time. `anglevehicle` defines the attitude of the vehicle over time
# #       (a vector with inclination angles with respect to each axis of the
# #       global coordinate system). `angle` defines the tilting angle of each
# #       tilting system over time (none in this case). `RPM` defines the RPM of
# #       each rotor system over time (none in this case).
# #       Each of these functions receives a nondimensional time `t`, which is the
# #       simulation time normalized by the total time `ttot`, from 0 to
# #       1, beginning to end of simulation. They all return a nondimensional
# #       output that is then scaled up by either a reference velocity (`Vref`) or
# #       a reference RPM (`RPMref`). Defining the kinematics and controls of the
# #       maneuver in this way allows the user to have more control over how fast
# #       to perform the maneuver, since the total time, reference velocity and
# #       RPM are then defined in the simulation parameters shown below.




# # ------------- 3) SIMULATION DEFINITION ---------------------------------------

# Vref = 0.0                                  # Reference velocity to scale maneuver by
# RPMref = 0.0                                # Reference RPM to scale maneuver by
# Vinit = Vref*Vvehicle(0)                    # Initial vehicle velocity
# Winit = pi/180*(anglevehicle(1e-6) - anglevehicle(0))/(1e-6*ttot)  # Initial angular velocity

#                                             # Maximum number of particles
# max_particles = (nsteps+1)*(vlm.get_m(vehicle.vlm_system)*(p_per_step+1) + p_per_step)

# simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
#                                                     Vinit=Vinit, Winit=Winit);




# # ------------- 4) MONITORS DEFINITIONS ----------------------------------------

# # NOTE: Monitors are functions that are called at every time step to perform
# #       some secondary computation after the solution of that time step has been
# #       obtained. In this case, the wing monitor uses the circulation and
# #       induced velocities to compute aerodynamic forces and decompose them
# #       into lift and drag. The monitor then plots these forces at every time
# #       step while also saving them under a CSV file in the simulation folder.

# # Generate function that calculates aerodynamic forces
# # NOTE: We exclude skin friction since we want to compare to the experimental
# #       data reported in Weber 1951 that was measured with pressure taps
# calc_aerodynamicforce_fun = uns.generate_calc_aerodynamicforce(;
#                                     add_parasiticdrag=true,
#                                     add_skinfriction=false,
#                                     airfoilpolar="xf-rae101-il-1000000.csv"
#                                     )

# D_dir = [cosd(AOA), 0.0, sind(AOA)]         # Direction of drag
# L_dir = uns.cross(D_dir, [0,1,0])           # Direction of lift

# figs, figaxs = [], []                       # Figures generated by monitor

# # Generate wing monitor
# monitor_wing = uns.generate_monitor_wing(wing, Vinf, b, ar,
#                                             rho, qinf, nsteps;
#                                             calc_aerodynamicforce_fun=calc_aerodynamicforce_fun,
#                                             L_dir=L_dir,
#                                             D_dir=D_dir,
#                                             out_figs=figs,
#                                             out_figaxs=figaxs,
#                                             save_path=save_path,
#                                             run_name=run_name,
#                                             figname="wing monitor",
#                                             );




# # ------------- 5) RUN SIMULATION ----------------------------------------------
# println("Running simulation...")

# uns.run_simulation(simulation, nsteps;
#                     # ----- SIMULATION OPTIONS -------------
#                     Vinf=Vinf,
#                     rho=rho,
#                     # ----- SOLVERS OPTIONS ----------------
#                     p_per_step=p_per_step,
#                     max_particles=max_particles,
#                     sigma_vlm_solver=sigma_vlm_solver,
#                     sigma_vlm_surf=sigma_vlm_surf,
#                     sigma_rotor_surf=sigma_vlm_surf,
#                     sigma_vpm_overwrite=sigma_vpm_overwrite,
#                     shed_starting=shed_starting,
#                     vlm_rlx=vlm_rlx,
#                     extra_runtime_function=monitor_wing,
#                     # ----- OUTPUT OPTIONS ------------------
#                     save_path=save_path,
#                     run_name=run_name
#                     );




# # ----------------- 6) VISUALIZATION -------------------------------------------
# if paraview
#     println("Calling Paraview...")

#     # Files to open in Paraview
#     files = joinpath(save_path, run_name*"_Wing_vlm...vtk")
#     files *= ";"*run_name*"_pfield...xmf;"

#     # Call Paraview
#     run(`paraview --data=$(files)`)

# end


# # ------------- 6) COMPARISON TO EXPERIMENTAL DATA -----------------------------
# save_outputs    = !true                    # Whether to save outputs or not

# # Where to save figures
# fig_path = save_path

# # Where to save output data (default to re-generating files used in the docs)
# outdata_path = joinpath(uns.examples_path, "..", "docs", "resources", "data")

# # Post-process monitor plots
# include(joinpath(uns.examples_path, "wing", "wing_postprocessing.jl"))


# # --------- Save simulation outputs
# if save_outputs
#     figs[1].savefig(joinpath(fig_path, run_name*"-simmonitor.png"),
#                                                 dpi=300, transparent=true)

#     str = """
#     |           | Experimental  | FLOWUnsteady              | Error |
#     | --------: | :-----------: | :-----------------------: | :---- |
#     | \$C_L\$   | 0.238         | $(round(CL, digits=5))    | $(round(CLerr*100, digits=3))% |
#     | \$C_D\$   | 0.005         | $(round(CD, digits=5))    | $(round(CDerr*100, digits=3))% |
#     """

#     open(joinpath(outdata_path, run_name*"-CLCD.md"), "w") do f
#         println(f, str)
#     end
# end


# # ----------------- ANGLE OF ATTACK SWEEP --------------------------------------
# sweep_aoa = !true                       # Whether to run AOA sweep

# if sweep_aoa
#     println("Running AOA sweep...")
#     include(joinpath(uns.examples_path, "wing", "wing_aoasweep.jl"))
# end
