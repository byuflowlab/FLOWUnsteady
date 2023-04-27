# [Run Simulation](@id vahanarun)

A mid fidelity resolution makes the computational cost
tractable and possible to be run the full maneuver (30 seconds of real time)
overnight on a laptop computer.
This is a video of the full maneuver in mid fidelity:

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
        src="https://www.youtube.com/embed/d__wNtRIBY8?hd=1"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

With a finer temporal and spatial resolution, it becomes impractical to resolve
the entire maneuver, and instead we recommend simulating one fragment of
the maneuver at a time.
For instance, here is a high-fidelity simulation of the transition from
hover to cruise:

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
        src="https://www.youtube.com/embed/-6aR37Z6hig?hd=1"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

As a reference, here are the parameters that we have used for the mid and high
fidelity simulations:


| Parameter | Mid fidelity | High fidelity | Description |
| :-------: | :----------: | :-----------: | :---------- |
| `n_factor`| `1` | `4` | Factor that controls the level of discretization of wings and blade surfaces |
| `nsteps`  | `4*5400` | `8*5400` | Time steps for the entire maneuver |
| `t_start` | `0` | `0.20*ttot` | (s) start simulation at this point in time |
| `t_quit`  | `ttot` | `0.30*ttot` | (s) end imulation at this point in time |
| `lambda_vpm` | `2.125` | `1.5*2.125` | VPM core overlap |
| `vlm_vortexsheet` | `false` | `true` | Whether to spread the wing surface vorticity as a vortex sheet |
| `vpm_integration` | `vpm.euler` | `vpm.rungekutta3` | VPM time integration scheme |


```@raw html
<br>
```

Along the way, in this simulation we exemplify the following advanced features:
* Defining a variable pitch for rotors between hover and cruise
* Using the [actuator surface model](@ref asm) for wing surfaces
* Defining a [wake treatment](@ref waketreatmentapi) that speeds up the
    simulation by removing particles that can be neglected

```@raw html
<br>
```


```julia
#=##############################################################################
# DESCRIPTION
    Simulation of eVTOL transition maneuver of a tandem tilt-wing multirotor
    aircraft. The aircraft configuration resembles the Vahana eVTOL aircraft
    but with tilt, stacked, and variable-pitch rotors.

# AUTHORSHIP
  * Author          : Eduardo J. Alvarez (edoalvarez.com)
  * Email           : Edo.AlvarezR@gmail.com
  * Created         : Feb 2021
  * Last updated    : Feb 2021
  * License         : MIT
=###############################################################################



import FLOWUnsteady as uns
import FLOWUnsteady: vlm, vpm, gt, Im

include(joinpath(uns.examples_path, "vahana", "vahana_vehicle.jl"))
include(joinpath(uns.examples_path, "vahana", "vahana_maneuver.jl"))
include(joinpath(uns.examples_path, "vahana", "vahana_monitor.jl"))

run_name        = "vahana"                  # Name of this simulation
save_path       = "vahana-example"          # Where to save this simulation
paraview        = true                      # Whether to visualize with Paraview

# ----------------- GEOMETRY PARAMETERS ----------------------------------------
n_factor        = 1                         # Discretization factor
add_wings       = true                      # Whether to include wings
add_rotors      = true                      # Whether to include rotors

# Reference lengths
R               = 0.75                      # (m) reference blade radius
b               = 5.86                      # (m) reference wing span
chord           = b/7.4                     # (m) reference wing chord
thickness       = 0.04*chord                # (m) reference wing thickness

# ----------------- SIMULATION PARAMETERS --------------------------------------
# Maneuver settings
Vcruise         = 15.0                      # (m/s) cruise speed (reference)
RPMh_w          = 600.0                     # RPM of main-wing rotors in hover (reference)
ttot            = 30.0                      # (s) total time to perform maneuver

use_variable_pitch = true                   # Whether to use variable pitch in cruise

# Freestream
Vinf(X,t)       = 1e-5*[1, 0, -1]           # (m/s) freestream velocity (if 0 the solvers can become unstable)
rho             = 1.225                     # (kg/m^3) air density
mu              = 1.81e-5                   # (kg/ms) air dynamic viscosity

# NOTE: Use these parameters to start and end the simulation at any arbitrary
#       point along the eVTOL maneuver (tstart=0 and tquit=ttot will simulate
#       the entire maneuver, tstart=0.20*ttot will start it at the beginning of
#       the hover->cruise transition)
tstart          = 0.00*ttot                 # (s) start simulation at this point in time
tquit           = 1.00*ttot                 # (s) end simulation at this point in time

start_kinmaneuver = true                    # If true, it starts the maneuver with the
                                            # velocity and angles of tstart.
                                            # If false, starts with velocity=0
                                            # and angles as initiated by the geometry
# ----------------- SOLVER PARAMETERS ------------------------------------------

# Aerodynamic solver
VehicleType     = uns.UVLMVehicle           # Unsteady solver
# VehicleType     = uns.QVLMVehicle         # Quasi-steady solver

# Time parameters
nsteps          = 4*5400                    # Time steps for entire maneuver
dt              = ttot/nsteps               # (s) time step

# VPM particle shedding
p_per_step      = 5                         # Sheds per time step
shed_starting   = false                     # Whether to shed starting vortex
shed_unsteady   = true                      # Whether to shed vorticity from unsteady loading
unsteady_shedcrit = 0.001                   # Shed unsteady loading whenever circulation
                                            #  fluctuates by more than this ratio

# Regularization of embedded vorticity
sigma_vlm_surf  = b/400                     # VLM-on-VPM smoothing radius
sigma_rotor_surf= R/20                      # Rotor-on-VPM smoothing radius
lambda_vpm      = 2.125                     # VPM core overlap
                                            # VPM smoothing radius
sigma_vpm_overwrite         = lambda_vpm * (2*pi*RPMh_w/60*R + Vcruise)*dt / p_per_step
sigmafactor_vpmonvlm        = 1             # Shrink particles by this factor when
                                            #  calculating VPM-on-VLM/Rotor induced velocities

# Rotor solver
vlm_rlx                     = 0.2           # VLM relaxation <-- this also applied to rotors
hubtiploss_correction       = vlm.hubtiploss_correction_prandtl # Hub and tip correction

# Wing solver: actuator surface model (ASM)
vlm_vortexsheet             = false         # Whether to spread the wing surface vorticity as a vortex sheet (activates ASM)
vlm_vortexsheet_overlap     = 2.125         # Overlap of the particles that make the vortex sheet
vlm_vortexsheet_distribution= uns.g_pressure# Distribution of the vortex sheet
# vlm_vortexsheet_sigma_tbv = thickness*chord / 100  # Size of particles in trailing bound vortices
vlm_vortexsheet_sigma_tbv   = sigma_vpm_overwrite
                                            # How many particles to preallocate for the vortex sheet
vlm_vortexsheet_maxstaticparticle = vlm_vortexsheet==false ? nothing : 6000000

# Wing solver: force calculation
KJforce_type                = "regular"     # KJ force evaluated at middle of bound vortices_vortexsheet also true)
include_trailingboundvortex = false         # Include trailing bound vortices in force calculations

include_unsteadyforce       = true          # Include unsteady force
add_unsteadyforce           = false         # Whether to add the unsteady force to Ftot or to simply output it

include_parasiticdrag       = true          # Include parasitic-drag force
add_skinfriction            = true          # If false, the parasitic drag is purely parasitic, meaning no skin friction
calc_cd_from_cl             = false         # Whether to calculate cd from cl or effective AOA
wing_polar_file             = "xf-n0012-il-500000-n5.csv"    # Airfoil polar for parasitic drag


# VPM solver
# vpm_integration = vpm.rungekutta3         # VPM temporal integration scheme
vpm_integration = vpm.euler

vpm_viscous     = vpm.Inviscid()            # VPM viscous diffusion scheme
# vpm_viscous   = vpm.CoreSpreading(-1, -1, vpm.zeta_fmm; beta=100.0, itmax=20, tol=1e-1)

# vpm_SFS       = vpm.SFS_none              # VPM LES subfilter-scale model
vpm_SFS         = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive;
                                  alpha=0.999, maxC=1.0,
                                  clippings=[vpm.clipping_backscatter],
                                  controls=[vpm.control_directional, vpm.control_magnitude])

if VehicleType == uns.QVLMVehicle
    # Mute warnings regarding potential colinear vortex filaments. This is
    # needed since the quasi-steady solver will probe induced velocities at the
    # lifting line of the blade
    uns.vlm.VLMSolver._mute_warning(true)
end




# ----------------- 1) VEHICLE DEFINITION --------------------------------------
vehicle = generate_vahana_vehicle(; xfoil       = false,
                                    n_factor    = n_factor,
                                    add_wings   = add_wings,
                                    add_rotors  = add_rotors,
                                    VehicleType = VehicleType,
                                    run_name    = "vahana"
                                    )



# ------------- 2) MANEUVER DEFINITION -----------------------------------------
maneuver = generate_maneuver_vahana(; add_rotors=add_rotors)

# Plot maneuver before running the simulation
uns.plot_maneuver(maneuver)



# ------------- 3) SIMULATION DEFINITION ---------------------------------------
# Reference parameters
Vref = Vcruise                              # Reference velocity to scale maneuver by
RPMref = RPMh_w                             # Reference RPM to scale maneuver by

# Initial conditions
t0 = tstart/ttot*start_kinmaneuver          # Time at start of simulation
Vinit = Vref*maneuver.Vvehicle(t0)          # Initial vehicle velocity
Winit = pi/180 * (maneuver.anglevehicle(t0+1e-12)- maneuver.anglevehicle(t0))/(ttot*1e-12) # Initial angular velocity

# Define simulation
simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                            Vinit=Vinit, Winit=Winit, t=tstart);

# Maximum number of particles (for pre-allocating memory)
max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.wake_system)*(p_per_step+1) + p_per_step) )
max_particles = tquit != Inf ? ceil(Int, max_particles*(tquit-tstart)/ttot) : max_particles
max_particles = min(10000000, max_particles)
max_particles = VehicleType==uns.QVLMVehicle ? 10000 : max_particles






# ------------- 4) MONITORS DEFINITIONS ----------------------------------------

# ------------- Routine for force calculation on wings
forces = []

# Calculate Kutta-Joukowski force
kuttajoukowski = uns.generate_aerodynamicforce_kuttajoukowski(KJforce_type,
                                sigma_vlm_surf, sigma_rotor_surf,
                                vlm_vortexsheet, vlm_vortexsheet_overlap,
                                vlm_vortexsheet_distribution,
                                vlm_vortexsheet_sigma_tbv;
                                vehicle=vehicle)
push!(forces, kuttajoukowski)


# Force due to unsteady circulation
if include_unsteadyforce
    unsteady(args...; optargs...) = uns.calc_aerodynamicforce_unsteady(args...; add_to_Ftot=add_unsteadyforce, optargs...)

    push!(forces, unsteady)
end

# Parasatic-drag force (form drag and skin friction)
if include_parasiticdrag
    parasiticdrag = uns.generate_aerodynamicforce_parasiticdrag(wing_polar_file;
                                                            calc_cd_from_cl=calc_cd_from_cl,
                                                            add_skinfriction=add_skinfriction)
    push!(forces, parasiticdrag)
end


# Stitch all the forces into one function
function calc_aerodynamicforce_fun(vlm_system, args...; per_unit_span=false, optargs...)

    # Delete any previous force field
    fieldname = per_unit_span ? "ftot" : "Ftot"
    if fieldname in keys(vlm_system.sol)
        pop!(vlm_system.sol, fieldname)
    end

    Ftot = nothing

    for force in forces
        Ftot = force(vlm_system, args...; per_unit_span=per_unit_span, optargs...)
    end

    return Ftot
end


# Extra options for generation of wing monitors
wingmonitor_optargs = (
                        include_trailingboundvortex=include_trailingboundvortex,
                        calc_aerodynamicforce_fun=calc_aerodynamicforce_fun
                      )

# ------------- Create monitors
monitors = generate_monitor_vahana(vehicle, rho, RPMref, nsteps,
                                         save_path, Vinf;
                                         add_wings=add_wings,
                                         wingmonitor_optargs=wingmonitor_optargs)




# ------------- INTERMEDIATE STEP BEFORE RUN ------------------------------------

# ------------- Define function for variable pitch

# Original blade twists
org_theta = [
                [
                    deepcopy(rotor._theta) for (ri, rotor) in enumerate(rotors)
                ]
                for (si, rotors) in enumerate(simulation.vehicle.rotor_systems)
            ]

# End time of each stage
#  Stage 1: [0, t1]  -> Take off
#  Stage 2: [t1, t2] -> Transition
#  Stage 3: [t2, t3] -> Cruise
#  Stage 4: [t3, t4] -> Transition
#  Stage 5: [t4, 1]  -> Landing
t1, t2, t3, t4 = 0.2, 0.3, 0.5, 0.6

# Pitch at each stage
pitch_takeoff  = 0
pitch_cruise   = 35
pitch_landing  = 0

# Function for smoothly transitioning pitch between stages
collective(tstr) =  tstr < t1 ? pitch_takeoff :
                    tstr < t2 ? pitch_takeoff + (pitch_cruise-pitch_takeoff)*(tstr-t1)/(t2-t1) :
                    tstr < t3 ? pitch_cruise :
                    tstr < t4 ? pitch_cruise + (pitch_landing-pitch_cruise)*(tstr-t3)/(t4-t3) :
                                pitch_landing

function variable_pitch(sim, args...; optargs...)

    if !use_variable_pitch
        return false
    end

    tstr = sim.t/sim.ttot               # Non-dimensional time

    for (si, rotors) in enumerate(sim.vehicle.rotor_systems)
        for (ri, rotor) in enumerate(rotors)

            if si==1                            # Main wing rotors

                # Restore original twist distribution
                rotor._theta .=  org_theta[si][ri]

                # Add collective pitch
                rotor._theta .+= 1.0 * (-1)^(rotor.CW)*collective(tstr)

            elseif si==2                        # Stacked upper rotors
                nothing

            elseif si==3                        # Stacked lower rotors
                nothing

            elseif si==4                        # Tandem-wing rotors
                rotor._theta .=  org_theta[si][ri]
                rotor._theta .+= 1.25 * (-1)^(rotor.CW)*collective(tstr)

            end

        end
    end

    return false
end


# ------------- Define wake treatment

# Remove by particle strength
# (remove particles neglibly faint, remove blown up)
rmv_strngth = 2*2/p_per_step * dt/(30/(4*5400))         # Reference strength
minmaxGamma = rmv_strngth*[0.0001, 0.05]                # Strength bounds (removes particles outside of these bounds)
wake_treatment_strength = uns.remove_particles_strength( minmaxGamma[1]^2, minmaxGamma[2]^2; every_nsteps=1)

# Remove by particle size
# (remove particle nearly singular, remove negligibly smeared)
minmaxsigma = sigma_vpm_overwrite*[0.1, 5]              # Size bounds (removes particles outside of these bounds)
wake_treatment_sigma = uns.remove_particles_sigma( minmaxsigma[1], minmaxsigma[2]; every_nsteps=1)

# Remove by distance
# (remove particles outside of the computational domain of interest, i.e., far from vehicle)
wake_treatment_sphere = uns.remove_particles_sphere((1.25*b)^2, 1; Xoff=[4.0, 0, 0])

# Concatenate all wake treatments
wake_treatment = uns.concatenate(wake_treatment_sphere, wake_treatment_strength, wake_treatment_sigma)



# ------------- Define runtime function: monitors + variable pitch + wake treatment
runtime_function = uns.concatenate(wake_treatment, monitors, variable_pitch)






# ------------- 5) RUN SIMULATION ----------------------------------------------
println("Running simulation...")


# Run simulation
uns.run_simulation(simulation, nsteps;
                    # ----- SIMULATION OPTIONS -------------
                    Vinf=Vinf,
                    rho=rho, mu=mu, tquit=tquit,
                    # ----- SOLVERS OPTIONS ----------------
                    p_per_step=p_per_step,
                    max_particles=max_particles,
                    max_static_particles=vlm_vortexsheet_maxstaticparticle,
                    vpm_integration=vpm_integration,
                    vpm_viscous=vpm_viscous,
                    vpm_SFS=vpm_SFS,
                    sigma_vlm_surf=sigma_vlm_surf,
                    sigma_rotor_surf=sigma_rotor_surf,
                    sigma_vpm_overwrite=sigma_vpm_overwrite,
                    sigmafactor_vpmonvlm=sigmafactor_vpmonvlm,
                    vlm_vortexsheet=vlm_vortexsheet,
                    vlm_vortexsheet_overlap=vlm_vortexsheet_overlap,
                    vlm_vortexsheet_distribution=vlm_vortexsheet_distribution,
                    vlm_vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv,
                    vlm_rlx=vlm_rlx,
                    hubtiploss_correction=hubtiploss_correction,
                    shed_starting=shed_starting,
                    shed_unsteady=shed_unsteady,
                    unsteady_shedcrit=unsteady_shedcrit,
                    extra_runtime_function=runtime_function,
                    # ----- OUTPUT OPTIONS ------------------
                    save_path=save_path,
                    run_name=run_name,
                    save_wopwopin=true,  # <--- Generates input files for PSU-WOPWOP noise analysis
                    );
```
