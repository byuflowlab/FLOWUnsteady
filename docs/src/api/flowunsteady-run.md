# 5) Run Simulation
FLOWUnsteady simulations are run by calling the following function:
```@docs
FLOWUnsteady.run_simulation
```

!!! compat "Required keyword arguments"
    The keyword arguments `sigma_vlm_surf` and `sigma_rotor_surf` are required.
    If left to their default value, the simulation will error out.

    Besides these two arguments, all other keyword arguments can be ignored and
    the simulation should run just fine without further customization.
