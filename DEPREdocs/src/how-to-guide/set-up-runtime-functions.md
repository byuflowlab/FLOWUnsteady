# How to Set up Run-time Functions

FLOWUnsteady allows for the definition of additional run-time functions, which are passed into the keyword argument, ```extra_runtime_function```, in the ```run_simulation()``` function.

These are typically formatted as follows:

```julia
function customfunction(inputs)

    function extra_runtime_function(sim, pfield, t, dt)

        #insert your function here

        return false

    end

end
```

If you have more than one additional function you'd like to run, a nice way for allowing arbitrarily defined functions is to define an encapsulating functions with the ```args...``` command such as

```julia
customfunc1 = customfunction1(inputs1)
customfunc2 = customfunction2(inputs2)
extrafunctions(args...) = customfunc1(args...) || customfunc2(args...)
```

## Creating Monitors

By incorporating plots into an extra runtime function, you can create monitors that will show you the progress of the simulation.  The definition of these very much depends on your use case. For the creators, aerodynamic forces are important, so we'll cover that now.

## Calculating Aerodynamic Forces

FLOWUnsteady has some functions that assist in calculating aerodynamic forces.  Specifically, the ```calc_aerodynamicforces()``` function and the ```decompose()``` function.

To get the total forces on a wing object, you need the current and previous wing state, as well as the freestream velocity and density.

```julia
Ftot = uns.calc_aerodynamicforces(currentwingobject,priorstepwingobject, pfield, Vinf, dt, rhoinf)
```

!!! note "Previous Wing"

    In order to save the previous wing, it is convenient to initialize prev_wing to nothing before entering the extra_runtime function, and using deep copy at the end of the extra runtime function (before returning ```false```).
    ```
    function customfunc(inputs)

        prev_wing = nothing

        extra_runtime_function(sim,pfield,t,dt)

            #custom function

            prev_wing = deepcopy(wing)

            return false

        end

    end
    ```

This gives you the total forces on the overall wing. If you want the forces on each section of the wing, you'd have to set the ```per_unit_span``` boolean keyword argument to ```true```

```julia
ftot = uns.calc_aerodynamicforces(currentwingobject,priorstepwingobject, pfield, Vinf, dt, rhoinf; per_unit_span=true)
```

To get the respective Lift, Drag, and Sideslip forces, you need to pass the total force vector into the ```decompose()``` function along with two orthogonal vectors indicating what direction the forces are acting.

For example, for steady, level flight in the typical aerodynamics frame, you could do

```julia
L, D, S = uns.decompose(Ftot,[0,0,1],[1,0,0])
```

Where lift is in the positive z-direction, or ```[0,0,1]``` and drag is in the positive x-direction, or ```[1,0,0]```.  The decompose function only needs the directions for the first two outputs, and then it calculates the third vector.

!!! note "Arbitrary Force Vectors"

    Because of how the decompose function is defined, you have no limits on what forces you calculate. That is to say, you can find the forces in any reference frame.  This also makes the user responsible for defining exactly what they want to use as the Lift vector, etc.


## Adding data to output files

By passing, say, a wing object into an extra runtime function, you can use the ```_addsolution()``` function in the VLM module to add data to the .vtk files output by the simulation.  This way, you can visualize in Paraview things such as forces and velocities in addition to the default outputs.

For example, if you had calculated Lift and Drag, and wanted to include those in the files used in Paraview visualization you could run the commands

```julia
vlm._addsolution(wingobject,"Lift",Lift)
vlm._addsolution(wingobject,"Drag",Drag)
```

You can also write data to other files that you define yourself.  For example, if you wanted to save the lift and drag coefficients in a .csv file for easy plotting later, you would simply open a file, write to it, and close it as one would normally do.