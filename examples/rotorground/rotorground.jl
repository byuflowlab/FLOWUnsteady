#=##############################################################################
# DESCRIPTION
    Simulation of a DJI 9443 rotor in hover in ground effect (two-bladed rotor, 9.4 inches
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

run_name        = "rotorground-example"      # Name of this simulation
save_path       = run_name                  # Where to save this simulation
paraview        = false # Whether to visualize with Paraview

# Uncomment this to have the folder named after this file instead
# save_path     = String(split(@__FILE__, ".")[1])
# run_name      = "singlerotor"
# paraview      = false
# ----------------- GEOMETRY PARAMETERS ----------------------------------------

# Rotor geometry
rotor_file      = "DJI9443.csv"             # Rotor geometry
data_path       = uns.def_data_path         # Path to rotor database
pitch           = 0.0                       # (deg) collective pitch of blades
CW              = false                     # Clock-wise rotation
xfoil           = false                     # Whether to run XFOIL
read_polar      = vlm.ap.read_polar2        # What polar reader to use

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
n               = 20                        # Number of blade elements per blade
r               = 1/10                      # Geometric expansion of elements

# NOTE: Here a geometric expansion of 1/10 means that the spacing between the
#       tip elements is 1/10 of the spacing between the hub elements. Refine the
#       discretization towards the blade tip like this in order to better
#       resolve the tip vortex.

# Read radius of this rotor and number of blades
R, B            = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]

# ----------------- SIMULATION PARAMETERS --------------------------------------

# Operating conditions
RPM             = 5400                      # RPM
J               = 0.0001                    # Advance ratio Vinf/(nD)
AOA             = 0                         # (deg) Angle of attack (incidence angle)

rho             = 1.071778                  # (kg/m^3) air density
mu              = 1.85508e-5                # (kg/ms) air dynamic viscosity
speedofsound    = 342.35                    # (m/s) speed of sound

# NOTE: For cases with zero freestream velocity, it is recommended that a
#       negligible small velocity is used instead of zero in order to avoid
#       potential numerical instabilities (hence, J here is negligible small
#       instead of zero)

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
nrevs           = 10                        # Number of revolutions in simulation
nsteps_per_rev  = 36                        # Time steps per revolution
nsteps          = const_solution ? 2 : nrevs*nsteps_per_rev # Number of time steps
ttot            = nsteps/nsteps_per_rev / (RPM/60)       # (s) total simulation time

# VPM particle shedding
p_per_step      = 4                         # Sheds per time step
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
sigmafactor_vpmonvlm= 1                     # Shrink particles by this factor when
                                            #  calculating VPM-on-VLM/Rotor induced velocities

# Rotor solver
vlm_rlx         = 0.5                       # VLM relaxation <-- this also applied to rotors
hubtiploss_correction = ((0.4, 5, 0.1, 0.05), (2, 1, 0.25, 0.05)) # Hub and tip correction

# VPM solver
vpm_integration = vpm.euler                 # VPM temporal integration scheme
# vpm_integration = vpm.rungekutta3

vpm_viscous     = vpm.Inviscid()            # VPM viscous diffusion scheme
# vpm_viscous   = vpm.CoreSpreading(-1, -1, vpm.zeta_fmm; beta=100.0, itmax=20, tol=1e-1)

vpm_SFS         = vpm.SFS_none              # VPM LES subfilter-scale model
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


# ----------------- 1) VEHICLE DEFINITION --------------------------------------
println("Generating geometry...")

# Generate rotor
rotor = uns.generate_rotor(rotor_file; pitch=pitch,
                                        n=n, CW=CW, blade_r=r,
                                        altReD=[RPM, J, mu/rho],
                                        xfoil=xfoil,
                                        read_polar=read_polar,
                                        data_path=data_path,
                                        verbose=true,
                                        plot_disc=true
                                        );

println("Generating vehicle...")

# Generate vehicle
system = vlm.WingSystem()                   # System of all FLOWVLM objects
vlm.addwing(system, "Rotor", rotor)

rotors = [rotor];                           # Defining this rotor as its own system
rotor_systems = (rotors, );                 # All systems of rotors

wake_system = vlm.WingSystem()              # System that will shed a VPM wake
                                            # NOTE: Do NOT include rotor when using the quasi-steady solver
if VehicleType != uns.QVLMVehicle
    vlm.addwing(wake_system, "Rotor", rotor)
end

vehicle = VehicleType(   system;
                            rotor_systems=rotor_systems,
                            wake_system=wake_system
                         );


# ------------- 2) MANEUVER DEFINITION -----------------------------------------
# Non-dimensional translational velocity of vehicle over time
Vvehicle(t) = zeros(3)

# Angle of the vehicle over time
anglevehicle(t) = zeros(3)

# RPM control input over time (RPM over `RPMref`)
RPMcontrol(t) = 1.0

angles = ()                                 # Angle of each tilting system (none)
RPMs = (RPMcontrol, )                       # RPM of each rotor system

maneuver = uns.KinematicManeuver(angles, RPMs, Vvehicle, anglevehicle)


# ------------- 3) SIMULATION DEFINITION ---------------------------------------

Vref = 0.0                                  # Reference velocity to scale maneuver by
RPMref = RPM                                # Reference RPM to scale maneuver by
Vinit = Vref*Vvehicle(0)                    # Initial vehicle velocity
Winit = pi/180*(anglevehicle(1e-6) - anglevehicle(0))/(1e-6*ttot)  # Initial angular velocity

simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit);

# define ground
mirror_X = SVector{3}(R,0.0,0.0)
mirror_normal = SVector{3}(-1.0,0,0)
mirror = true

# Restart simulation
# restart_file = nothing

# NOTE: Uncomment the following line to restart a previous simulation.
#       Point it to a particle field file (with its full path) at a specific
#       time step, and `run_simulation` will start this simulation with the
#       particle field found in the restart simulation.

restart_file = "rotorhover-example_pfield.50"
restart_file = nothing


# ------------- 4) MONITORS DEFINITIONS ----------------------------------------

# Generate rotor monitor
monitor_rotor = uns.generate_monitor_rotors(rotors, J, rho, RPM, nsteps;
                                            t_scale=RPM/60,        # Scaling factor for time in plots
                                            t_lbl="Revolutions",   # Label for time axis
                                            save_path=save_path,
                                            run_name=run_name,
                                            figname="rotor monitor",
                                            )

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
runtime_function = uns.concatenate(monitors, wake_treatment_supress)

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
                    save_wopwopin=true,  # <--- Generates input files for PSU-WOPWOP noise analysis
                    save_code=@__FILE__
                    );




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


# ------------- 6) POSTPROCESSING ----------------------------------------------

# Post-process monitor plots
# include(joinpath(uns.examples_path, "rotorhover", "rotorhover_postprocessing.jl"))
