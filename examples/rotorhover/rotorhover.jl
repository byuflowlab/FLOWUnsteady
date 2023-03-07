#=##############################################################################
# DESCRIPTION
    Simulation of a DJI 9443 rotor in hover (two-bladed rotor, 9.4 inches
    diameter).

    This example replicates the experiment described in Zawodny & Boyd (2016),
    "Acoustic Characterization and Prediction of Representative,
    Small-scale Rotary-wing Unmanned Aircraft System Components."

# AUTHORSHIP
  * Author          : Eduardo J. Alvarez (edoalvarez.com)
  * Email           : Edo.AlvarezR@gmail.com
  * Created         : Mar 2023
  * Last updated    : Mar 2023
  * License         : MIT
=###############################################################################

#= TODO
    * [ ] Show a simplified example with a propeller case sweeping on J and validation
        * [ ] Start with one case, J=0.3, APC 10x7
        * [ ] Convert to J sweep, no plots, threaded. Compare to experimental.
                J = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.775]
        * [ ] Repeat with quasi-steady solver.
    * [ ] Bring down the number of elements in the DJI example to speed things up?
    * [ ] Visualization guide
        * [ ] Changing ParaView ugly default color
    * [ ] Rotor noise
    * [ ] Some words on generating your own rotor geometry
    * [ ] Cd monitor: Typically you can run one simulation with the dynamic
            coeff, write down the mean Cd, then switch to the static model coeff
            fixed to that value. It'll make the simulation 1.5x faster.
=#

import FLOWUnsteady as uns
import FLOWVLM as vlm
import FLOWVPM as vpm

run_name        = "rotorhover-example01"      # Name of this simulation

save_path       = run_name                  # Where to save this simulation
paraview        = true                      # Whether to visualize with Paraview


# ----------------- GEOMETRY PARAMETERS ----------------------------------------

# Rotor geometry
# rotor_file      = "DJI9443.csv"             # Rotor geometry
rotor_file      = "DJI9443-smoothpolars.csv"# Rotor geometry
data_path       = uns.def_data_path         # Path to rotor database
pitch           = 0.0                       # (deg) collective pitch of blades
CW              = false                     # Clock-wise rotation
xfoil           = false                     # Whether to run XFOIL
# xfoil           = true

# TODO: Hide this?
# read_polar      = vlm.ap.read_polar         # What polar reader to use
read_polar      = vlm.ap.read_polar2

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
n               = 50                        # Number of blade elements per blade
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

# NOTE: For cases with zero freestream velocity, in order to avoid numerical
#       instabilities, it is recommended that a negligible small velocity is
#       used instead of zero (hence, J here is negligible small instead of zero)

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
# nrevs           = 4                         # Number of revolutions in simulation
nrevs           = 6
# nrevs           = 10
nsteps_per_rev  = 72                        # Time steps per revolution
# nsteps_per_rev  = 72*5
nsteps          = const_solution ? 2 : nrevs*nsteps_per_rev # Number of time steps
ttot            = nsteps/nsteps_per_rev / (RPM/60)       # (s) total simulation time


# VPM particle shedding
p_per_step      = 2                         # Sheds per time step
shed_starting   = true                      # Whether to shed starting vortex
# shed_unsteady   = true                      # Whether to shed vorticity from unsteady loading
shed_unsteady   = false
max_particles   = ((2*n+1)*B)*nsteps*p_per_step + 1 # Maximum number of particles

# Regularization
sigma_rotor_surf= R/10                      # Rotor-on-VPM smoothing radius
# sigma_rotor_surf= R/40
# sigma_rotor_surf= R/80
lambda_vpm      = 2.125                     # VPM core overlap
                                            # VPM smoothing radius
sigma_vpm_overwrite = lambda_vpm * 2*pi*R/(nsteps_per_rev*p_per_step)

# Rotor solver
vlm_rlx         = 0.7                       # VLM relaxation <-- This also applied to Rotor
# vlm_rlx         = 0.5
# vlm_rlx         = 0.1

# hubtiploss_correction = vlm.hubtiploss_correction_prandtl   # Hub and tip correction
hubtiploss_correction = vlm.hubtiploss_correction_modprandtl
# hubtiploss_correction = ((0.4, 5, 0.1, 0.05), vlm.hubtiploss_correction_modprandtl[2])

# no_shedding_Rthreshold = 0.35           # Supress wake shedding on elements inboard of this radial station
# no_shedding_Rthreshold = shed_unsteady ? 0.0 : 0.35
# no_shedding_nstepsthreshold = 3*nsteps_per_rev # Supress wake shedding before this many time steps
# no_shedding_nstepsthreshold = 1*nsteps_per_rev # Supress wake shedding before this many time steps
# no_shedding_nstepsthreshold = nsteps

# VPM solver
vpm_viscous     = vpm.Inviscid()            # VPM viscous diffusion scheme
# vpm_viscous     = vpm.CoreSpreading(-1, -1, vpm.zeta_fmm; beta=100.0, itmax=20, tol=1e-1)
# vpm_SFS         = vpm.SFS_none            # VPM LES subfilter-scale model
# vpm_SFS         = vpm.SFS_Cd_twolevel_nobackscatter
# vpm_SFS         = vpm.SFS_Cd_threelevel_nobackscatter
vpm_SFS         = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive; alpha=0.999, clippings=[vpm.clipping_backscatter], maxC=1.0)
# vpm_SFS         = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive; clippings=[vpm.clipping_backscatter],
                                                        # controls=[control_sigmasensor],
                                                        # alpha=0.999, rlxf=0.005, minC=0, maxC=1)

# NOTE: In most practical situations, open rotors operate at a Reynolds number
#       high enough that viscous diffusion in the wake is actually negligible.
#       Hence, it does not make much of a difference whether we run the
#       simulation with viscous diffusion enabled or not. On the other hand,
#       such high Reynolds numbers mean that the wake quickly becomes turbulent
#       and it is crucial to use a subfilter-scale (SFS) model to accurately
#       capture the turbulent decay of the wake.




# TODO: Hide this
sigmafactor_vpmonvlm   = 1                  # Shrinks the particles by this factor when calculated VPM-on-VLM/Rotor induced velocities
# sigmafactor_vpmonvlm   = 5.50
unsteady_shedcrit = 0.001                   # Shed unsteady loading whenever circulation fluctuates more than this ratio

!xfoil ? @warn("XFOIL deactivated!") : nothing
!shed_unsteady ? @warn("Unsteady loading wake deactivated!") : nothing


if VehicleType == uns.QVLMVehicle
    # Mute warnings regarding potential colinear vortex filaments. This is
    # needed since the quasi-steady solver will probe induced velocities at the
    # lifting line of the blade
    uns.vlm.VLMSolver._mute_warning(true)
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

vehicle = uns.VLMVehicle(   system;
                            rotor_systems=rotor_systems,
                            wake_system=wake_system
                         );

# NOTE: Through the `rotor_systems` keyword argument to `uns.VLMVehicle` we
#       have declared any systems (groups) of rotors that share a common RPM.
#       We will later declare the control inputs to each rotor system when we
#       define the `uns.KinematicManeuver`.


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

# NOTE: `FLOWUnsteady.KinematicManeuver` defines a maneuver with prescribed
#       kinematics. `Vvehicle` defines the velocity of the vehicle (a vector)
#       over time. `anglevehicle` defines the attitude of the vehicle over time.
#       `angle` defines the tilting angle of each tilting system over time.
#       `RPM` defines the RPM of each rotor system over time.
#       Each of these functions receives a nondimensional time `t`, which is the
#       simulation time normalized by the total time `ttot`, from 0 to
#       1, beginning to end of simulation. They all return a nondimensional
#       output that is then scaled by either a reference velocity (`Vref`) or
#       a reference RPM (`RPMref`). Defining the kinematics and controls of the
#       maneuver in this way allows the user to have more control over how fast
#       to perform the maneuver, since the total time, reference velocity and
#       RPM are then defined in the simulation parameters shown below.


# ------------- 3) SIMULATION DEFINITION ---------------------------------------

Vref = 0.0                                  # Reference velocity to scale maneuver by
RPMref = RPM                                # Reference RPM to scale maneuver by
Vinit = Vref*Vvehicle(0)                    # Initial vehicle velocity
Winit = pi/180*(anglevehicle(1e-6) - anglevehicle(0))/(1e-6*ttot)  # Initial angular velocity

simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit);

# Restart simulation
restart_file = nothing

# NOTE: Uncomment the following line to restart a previous simulation.
#       Point it to a particle field file (with its full path) at a specific
#       time step, and `run_simulation` will start this simulation with the
#       particle field found in the restart simulation.

# restart_file = "/path/to/a/previous/simulation/rotorhover-example_Rotor_pfield.360"


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

monitors = uns.concatenate(monitor_rotor, monitor_enstrophy, monitor_Cd)


# ------------- 5) RUN SIMULATION ----------------------------------------------
println("Running simulation...")

uns.run_simulation(simulation, nsteps;
                    # ----- SIMULATION OPTIONS -------------
                    Vinf=Vinf,
                    # ----- SOLVERS OPTIONS ----------------
                    p_per_step=p_per_step,
                    max_particles=max_particles,
                    vpm_viscous=vpm_viscous,
                    vpm_SFS=vpm_SFS,
                    sigma_vlm_surf=sigma_rotor_surf,
                    sigma_rotor_surf=sigma_rotor_surf,
                    sigma_vpm_overwrite=sigma_vpm_overwrite,
                    vlm_rlx=vlm_rlx,
                    hubtiploss_correction=hubtiploss_correction,
                    shed_unsteady=shed_unsteady,
                    shed_starting=shed_starting,
                    extra_runtime_function=monitors,
                    # ----- OUTPUT OPTIONS ------------------
                    save_path=save_path,
                    run_name=run_name,
                    save_wopwopin=true,  # <--- Generate input files for PSU-WOPWOP noise analysis


                    sigmafactor_vpmonvlm=sigmafactor_vpmonvlm,
                    unsteady_shedcrit=unsteady_shedcrit,
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

# # Post-process monitor plots
# include(joinpath(uns.examples_path, "heavingwing", "heavingwing_postprocessing.jl"))
