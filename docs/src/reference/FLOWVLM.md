# FLOWVLM

FLOWVLM is a vortex lattice method (VLM) code for solving aerodynamics, but is far more than that.
FLOWVLM has a general three-dimensional geometric engine that powers much of the geometry of FLOWUnsteady and is also the foundation of the blade-element rotor solver.
In this section we describe how wing geometries are defined, how multiple wings can be grouped into a wing system, and how objects can be rotated and translated in space.

## Wing Definition
A `FLOWVLM.Wing` object is simply a data structure that encapsulates a collection of horseshoes vortices (or a vortex lattice).
A horseshoe vortex is made out of one bound vortex located at the quarter-chord position, two trailing bound vortices going back to the trailing edge, and two semi-infinite vortices extending from the trailing edge out in the direction of the freestream, as shown below.

<img src="../assets/referencefigs/horseshoe00.png" alt="drawing" width="400"/>

A control point is associated to every horseshoe, which is located at the three-quarter chord position.
At this point, the no-flow-through boundary condition is imposed, canceling the component of the freestream that is normal to the surface.

!!! note "Unsteady wake"

    In the unsteady solver of FLOWUnsteady, semi-infinite trailing vortices of the horseshoes are replaced by vortex particles that are shed off the trailing edge at every time step.

There are three ways of defining a VLM wing. The first one is to manually add chord stations along the wing and discretize the sections in between into horseshoes.
This approach is tedious and is rarely used.
The second one is through the function `FLOWVLM.SimpleWing` that takes the parameters of the root and tip of the wing and interpolates and discretizes everything in between, which works well for most conventional wings.
Finally, for complex geometries (for instance, an asymmetric wing or a wing with winglets), the user can call `FLOWVLM.ComplexWing` that works pretty much like the option of manually building the wing but in a friendlier way.

### Manually Building the Wing
  You can build a wing by adding each chord station manually and indicating the discretization in between.
  This is done by indicating the leading edge position and chord length at each station, and the number of horseshoes in between stations.
  This approach uses the `FLOWVLM.Wing` constructor to initialize the wing and the `FLOWVLM.addwing` function to add each chord.

```@docs
FLOWVLM.Wing
```

```@docs
FLOWVLM.addchord
```

  This requires the user to do all the calculations of what the geometry should look like based on the desired aspect ratio, taper ratio, etc.
  In the following line we are doing such calculations for a $40^\circ$-swept-back wing with an aspect ratio of 5.0.

```
  # Wing parameters
  b = 98*0.0254                   # (m) span
  ar = 5.0                        # Aspect ratio (span over tip chord)
  tr = 1.0                        # Taper ratio
  lambda = 45.0                   # (deg) sweep
  gamma = 0.0                     # (deg) dihedral
  twist_tip = 0.0                 # (deg) tip twist
  twist_root = 0.0                # (deg) root twist
  n = 4                           # Horseshoes in between chord stations

  # Calculations
  cr = 1/tr                       # Chord ratio (inverse of taper ratio)
  c_tip = b/ar                    # Chord at tip
  c_root = cr*c_tip               # Chord at root
  y_tip = b/2                     # y-position of tip leading edge
  x_tip = y_tip*tan(lambda*pi/180)# x-position of tip leading edge
  z_tip = y_tip*tan(gamma*pi/180) # z-position of tip leading edge
```

  We initialize the wing by giving it the first chord station, which corresponds to the left chord.

```
import FLOWVLM
vlm = FLOWVLM

# Initialize the wing with the left tip chord
wing = vlm.Wing(x_tip, -y_tip, z_tip, c_tip, twist_tip)
```

  Then we go on to add the root and the right tip chord.
```
  # Add the root chord
  vlm.addchord(wing, 0.0, 0.0, 0.0, c_root, twist_root, n)

  # Add the right tip chord
  vlm.addchord(wing, x_tip, y_tip, z_tip, c_tip, twist_tip, n)
```
  Notice that we placed the root at the `(0, 0, 0)` position.
  This doesn't always have to be this way, but remember to move the other chord stations accordingly if you want to place the nose somewhere else.

!!! tip "Build wings from left to right"
    It is recommended that wings are build from left to right, or from $-y$ to $+y$.
    Otherwise, the normals of the surface will point down in the negative $z$-direction and the circulation of the wing will be negative when generating upwards lift (positive $z$-direction).
    This is not a problem for the solver, but could be difficult to wrap your head around a negative circulation being associated to a positive lift.


### `SimpleWing`
asdasd

### `ComplexWing`
