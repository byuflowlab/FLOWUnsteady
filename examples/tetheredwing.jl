#=##############################################################################
# DESCRIPTION
    Kite flying in a circular path with crosswind while tethered to the ground.
    The kite is the same 45° swept-back wing from the previous example.

# AUTHORSHIP
  * Author          : Eduardo J. Alvarez (edoalvarez.com)
  * Email           : Edo.AlvarezR@gmail.com
  * Created         : Feb 2023
  * Last updated    : Feb 2023
  * License         : MIT

Rewritten by Benjamin Varela in June 2024 for integration into version 5.0.0 of FLOWUnsteady
using weber validaiton
=###############################################################################

using Pkg
this_dir = @__DIR__
FLOWUnsteady_dir = normpath(joinpath(this_dir,".."))
Pkg.activate(FLOWUnsteady_dir)

using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm

using Plots

# construct VLM system (see VortexLattice.jl for info on VLM systems)
function construct_vlm_system(; ns=10, nc=5)
    c0 = 20*0.0254
    halfspan = 44.0/0.9 * 0.0254
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

function construct_simulation(time_range)
    R = 3.0 * 44.0/0.9 * 0.0254

    ρ = 1.0 # air density
    vinf = 5.0 # magnitude of freestream velocity
    vehicle_velocity = 49.7 # magnitude of the vehicle velocity
    omega = 0.0 # angular velocity of the vehicle

    velocity = vehicle_velocity * SVector{3}(1.0,0.0,0.0) # vehicle velocity
    angular_velocity = omega * SVector{3}(-1.0,0.0,0.0) # angular velocity

    save_steps = collect(range(0,length(time_range)-1)) # which steps to save info on
    dynamic = false # Do not use forces to determine states or motion

    # simulation parameters
    freestream = SimpleFreestream(vinf * SVector{3}(0.0,0.0,-1.0), ρ) # Freestream velocity and density

    # construct vehicle
    p_per_step_trailing = 3
    p_per_step_unsteady = 3
    vlm_system = construct_vlm_system()
    model = VortexLatticeModel(vlm_system; p_per_step_trailing, p_per_step_unsteady, max_timesteps=length(time_range), max_particles_override=50000, FMM=false) # at the time of this writing, FMM is not working but the default is true
    vehicle = RigidBodyVehicle(model; dynamic, velocity, angular_velocity, model_coordinates=Aerodynamics(), vehicle_coordinates=FlightDynamics())
    controller = PrescribedKinematics()

    # postprocessor
    history = History(vehicle, controller, save_steps)
    paraview = ParaviewOutput(save_steps)
    postprocessor = MultiPostprocessor((history, paraview))

    # create simulation object
    sim = Simulation(vehicle, time_range; freestream, postprocessor)

    return sim
end

function run_simulation(time_range, run_name, path)
    sim = construct_simulation(time_range)
    simulate!(sim, time_range; run_name, path)
    return sim
end

save_path = normpath(joinpath(this_dir,"tethered-results"))
!isdir(save_path) && mkdir(save_path)
# sim = run_simulation(range(0.0,stop=0.1,length=2), "tethered-wing", save_path);
sim = run_simulation(range(0.0,step=0.001,length=151), "tethered-wing", save_path);
println()


# # ----------------- GEOMETRY PARAMETERS ----------------------------------------
# # Wing description
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

# # ----------------- SIMULATION PARAMETERS --------------------------------------
# # Freestream
# magVinf         = 5.0                       # (m/s) freestream velocity
# magVeff         = 49.7                      # (m/s) vehicle effective velocity
# AOAeff          = 4.2                       # (deg) vehicle effective AOA
# rho             = 0.93                      # (kg/m^3) air density
# qinf            = 0.5*rho*magVeff^2         # (Pa) static pressure

# Vinf(X, t)      = magVinf*[1, 0, 0]         # Freestream function

# # Circular path
# R               = 3.0*b                     # (m) radius of circular path
# magVvehicle     = sqrt(magVeff^2 - magVinf^2) # (m/s) vehicle velocity
# omega           = magVvehicle/R             # (rad/s) angular velocity
# alpha           = AOAeff - atand(magVinf, magVvehicle) # (deg) vehicle pitch angle

# # ----------------- SOLVER PARAMETERS ------------------------------------------
# # Time parameters
# nrevs           = 2.50                      # Revolutions to resolve
# nsteps_per_rev  = 144                       # Number of time steps per revolution

# ttot            = nrevs / (omega/(2*pi))    # (s) total simulation time
# nsteps          = ceil(Int, nrevs * nsteps_per_rev) # Number of time steps

# # VLM and VPM parameters
# p_per_step      = 4                         # Number of particle sheds per time step

# sigma_vlm_solver= -1                        # VLM-on-VLM smoothing radius (deactivated with <0)
# sigma_vlm_surf  = 0.05*b                    # VLM-on-VPM smoothing radius
# lambda_vpm      = 2.125                     # VPM core overlap
#                                             # VPM smoothing radius
# sigma_vpm_overwrite = lambda_vpm * magVeff * (ttot/nsteps)/p_per_step

# shed_starting   = true                      # Whether to shed starting vortex
# vlm_rlx         = 0.7                       # VLM relaxation

# # ------------- 3) SIMULATION DEFINITION ---------------------------------------

# Vref = magVvehicle                          # Reference velocity to scale maneuver by
# RPMref = 0.0                                # Reference RPM to scale maneuver by
# Vinit = Vref*Vvehicle(0)                    # Initial vehicle velocity
# Winit = pi/180*(anglevehicle(1e-6) - anglevehicle(0))/(1e-6*ttot)  # Initial angular velocity
