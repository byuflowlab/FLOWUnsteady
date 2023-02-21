# 4) Monitors Definitions

A monitor is a function that is passed to [`FLOWUnsteady.run_simulation`](@ref) as an
extra runtime function that is
called at every time step. Runtime functions are expected to return a Boolean
that indicates the need of stopping the simulation, such that if
`extra_runtime_function` ever returns `true`, the simulation will immediately be
ended.

Multiple monitors can be concatenated with boolean logic as follows
```julia
import FLOWUnsteady as uns

monitor_states = uns.generate_monitor_statevariables()
monitor_enstrophy = uns.generate_monitor_enstrophy()

monitors(args...; optargs...) = monitor_states(args...; optargs...) || monitor_enstrophy(args...; optargs...)
```
Then pass the monitor to the simulation as
```julia
uns.run_simulation(sim, nsteps; extra_runtime_function=monitors, ...)
```

!!! compat "Extra Runtime Function"
    `extra_runtime_function` is a function that is called at every time step
    after the state variables are updated and before outputting VTK files.
    The state variables of the simulation are passed to this function, giving
    the user complete freedom to modify the states of the simulation (*e.g.*,
    add/remove particles, clip vortex strengths, re-orient the vehicle) or
    to do some extra computation (*e.g.*, compute aerodynamic forces, create
    plots, write to files, etc).

    It is expected to be a function of the form
    `extra_runtime_function(sim, pfield, t, dt; vprintln) -> Bool`, where
    `sim` is the [`FLOWUnsteady.Simulation`](@ref) object, `pfield`
    is the [`FLOWVPM.ParticleField`](@ref), `t` is the current simulation time, `dt` is
    the length of the current time step, and `vprintln(str, v_lvl)` is a function
    for printing the verbose of the simulation (default to
    `vprintln = (args...)->nothing` if you have no need to print any
    verbose).

    If `extra_runtime_function(sim, pfield, t, dt)` is also used to break the
    simulation, such that if it ever returns `true`, the simulation will
    immediately quit.

## Monitor Generators
The following are functions for generating the monitors that serve most use
cases. See the source code of these monitors to get an idea of how to write your
own user-defined monitors.

```@docs
FLOWUnsteady.generate_monitor_rotors
FLOWUnsteady.generate_monitor_wing
FLOWUnsteady.generate_monitor_statevariables
FLOWUnsteady.generate_monitor_enstrophy
FLOWUnsteady.generate_monitor_Cd
```
## Force Calculators
The following are some possible methods for calculating aerodynamic forces.
Generator functions return a function that can be directly passed to
[`FLOWUnsteady.generate_monitor_wing`](@ref) through the keyword argument
`calc_aerodynamicforce_fun`.

```@docs
FLOWUnsteady.generate_calc_aerodynamicforce
FLOWUnsteady.generate_aerodynamicforce_kuttajoukowski
FLOWUnsteady.generate_aerodynamicforce_parasiticdrag
FLOWUnsteady.calc_aerodynamicforce_unsteady
```

## Wake Treatment
Since the full set of state variables is passed to `extra_runtime_function`,
this function can also be used to alter the simulation on the fly.
In some circumstances it is desirable to be able to remove or modify particles,
which process we call "wake treatment."
Wake treatment is often used to reduce computational cost (for
instance, by removing particle in regions of the flow that are not of interest),
and it can also be used to force numerical stability (for instance, by
removing or clipping particles with vortex strengths that grow beyond certain
threshold).

These wake treatment methods can be added into the pipeline of
`extra_runtime_function` as follows:
```julia
import FLOWUnsteady as uns

# Define monitors
monitor_states = uns.generate_monitor_statevariables()
monitor_enstrophy = uns.generate_monitor_enstrophy()

# Monitor pipeline
monitors(args...; optargs...) = monitor_states(args...; optargs...) || monitor_enstrophy(args...; optargs...)

# Define wake treatment
wake_treatment = uns.remove_particles_sphere(1.0, 1; Xoff=zeros(3))

# Extra runtime function pipeline
extra_runtime_function(args...; optargs...) = monitors(args...; optargs...) || wake_treatment(args...; optargs...)

```
Then pass this pipeline to the simulation as
```julia
uns.run_simulation(sim, nsteps; extra_runtime_function=extra_runtime_function, ...)
```

Below we list a few generator functions that return common wake treatment
methods.

```@docs
FLOWUnsteady.remove_particles_sphere
FLOWUnsteady.remove_particles_box
FLOWUnsteady.remove_particles_lowstrength
FLOWUnsteady.remove_particles_strength
FLOWUnsteady.remove_particles_sigma
```
