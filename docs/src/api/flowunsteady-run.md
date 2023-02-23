# (5) Run Simulation

```@docs
FLOWUnsteady.run_simulation
```


!!! compat "Extra Runtime Function"
    `extra_runtime_function` is a function that is called at every time step
    after the state variables are updated and before outputting VTK files.
    The state variables of the simulation are passed to this function, giving
    the user complete freedom to modify the states of the simulation (*e.g.*,
    add/remove particles, clip vortex strengths, re-orient the vehicle) or
    to do some extra computation (*e.g.*, compute aerodynamic forces, create
    plots, write to files, etc).

    This function is expected be of the form
    ```
    extra_runtime_function(sim, pfield, t, dt; vprintln) -> Bool
    ```
    where `sim` is the [`FLOWUnsteady.Simulation`](@ref) object, `pfield`
    is the [`FLOWVPM.ParticleField`](@ref), `t` is the current simulation time, `dt` is
    the length of the current time step, and `vprintln(str, v_lvl)` is a function
    for printing the verbose of the simulation (default to
    `vprintln = (args...)->nothing` if there is nothing to add to the verbose).

    The output of `extra_runtime_function` is a flag for breaking the
    simulation at the current time step, such that if it ever returns `true`,
    the simulation will immediately quit.

!!! compat "So, what is going on under the hood?"
    FLOWUnsteady is simply a runtime function that is provided to FLOWVPM, as
    shown below.
    Hence, FLOWVPM (green block) is the solver that is actually driving the
    simulation.
    FLOWUnsteady (blue block) acts as a runtime function inside a VPM simulation
    that at each time step uses the solvers in the gray block to compute surface
    vorticities and adds particles to embed such vorticity in the flow field
    (thus implementing the actuator surface/line models developed in
    [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/), Chapter
    6).

    This workflow makes it very easy to couple more solvers in the simulation.
    For instance, a structural solver can be inserted as a runtime function
    (step 6 of the blue block) that deflects the geometry according to
    the aerodynamic loads, obtaining a full aeroelastic simulation.

```@raw html
<center>
  <img src="../../assets/images/flowchart/flowchart00.png" alt="Pic here" style="width: 100%;"/>
</center>
```
