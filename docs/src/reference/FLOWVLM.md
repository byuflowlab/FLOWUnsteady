# FLOWVLM

FLOWVLM is a vortex lattice method (VLM) code for solving aerodynamics, but is far more than that.
FLOWVLM has a general three-dimensional geometric engine that powers much of the geometry of FLOWUnsteady and is also the foundation of the blade-element rotor solver.
In this section we describe how wing geometries are defined, how multiple wings can be grouped into a wing system, and how objects can be rotated and translated in space.

```@contents
Pages = ["FLOWVLM.md"]
```

## Wing Definition
A `FLOWVLM.Wing` object is simply a data structure that encapsulates a collection of horseshoes vortices (or a vortex lattice).
A horseshoe vortex is made out of one bound vortex located at the quarter-chord position, two trailing bound vortices going back to the trailing edge, and two semi-infinite vortices extending from the trailing edge out in the direction of the freestream, as shown below.
![alt text](https://media.githubusercontent.com/media/byuflowlab/FLOWUnsteady/master/docs/src/assets/referencefigs/horseshoe00.png)
A control point is associated to every horseshoe, which is located at the three-quarter chord position.
At this point, the no-flow-through boundary condition is imposed, canceling the component of the freestream that is normal to the surface.


!!! note "Unsteady wake"
    In the unsteady solver of FLOWUnsteady, semi-infinite trailing vortices of the horseshoes are replaced by vortex particles that are shed off the trailing edge at every time step.

There are three ways of defining a VLM wing. The first one is to manually add chord stations along the wing and discretize the sections in between into horseshoes.
This approach is tedious and is rarely used.
The second one is through the function `FLOWVLM.SimpleWing` that takes the parameters of the root and tip of the wing and interpolates and discretizes everything in between, which works well for most conventional wings.
Finally, for complex geometries (for instance, an asymmetric wing or a wing with winglets), the user can call `FLOWVLM.ComplexWing` that works pretty much like the option of manually building the wing but in a friendlier way.

### Manually Building the Wing

### `SimpleWing`

### `ComplexWing`
