# How to set up dependencies needed for FlightVehicleSim (October 2019)
Note: currently this simulation tool functions only with Julia v0.6, thus the following will assume Julia v0.6 syntax.

## [Geometric Tools](https://github.com/byuflowlab/GeometricTools.jl): 

Dependencies:
- PyCall
- PyPlot
- Dierckx
- Roots
- QuadGK

Clone the package using Julia

```Pkg.clone("https://github.com/byuflowlab/GeometricTools.jl.git")```

If using MacOS, you'll want to add 

```export PATH=$PATH:/Applications/ParaView-5.5.2.app/Contents/MacOS/```

to your .bash_profile, swapping the 5.5.2 for whatever version of [Paraview](https://www.paraview.org/download/) you have downloaded (you'll need to download it if you don't already have it).

## [FLOWVLM](https://github.com/byuflowlab/FLOWVLM)

Additional Dependencies for FLOWVLM:
- [CCBlade](https://github.com/byuflowlab/CCBlade.jl)
- [airfoil](https://github.com/EdoAlvarezR/airfoil)

To get CCBlade:

```Pkg.clone("https://github.com/byuflowlab/CCBlade.jl.git")```

To get airfoil, clone the repo (not in Julia) and put it somewhere good for you

```git clone https://github.com/EdoAlvarezR/airfoil.git```

You will then need to go into the airfoil/src/jxlight directory and build the Fortran binary. If using MacOS, do

```make gfortran```

and that should be sufficient to build it.

To make sure FLOWVLM is pointed to the airfoil code correctly, in FLOWVLM.jl (in the src folder of the FLOWVLM repo) change line 21 such that the airfoil_path is the path to the directory you just cloned.

Clone the repo (not in julia)

```git clone https://github.com/byuflowlab/FLOWVLM.git```

and then in the examples/future code, you need to name flowvlm_path the path to where you cloned the directory.
You will also need to include the FLOWVLM.jl file, and it is convenient to call FLOWVLM vlm as is done in the examples. Your code might begin with something like the following:

```
flowvlm_path = "/path_to_FLOWVLM/"
include(flowvlm_path*"src/FLOWVLM.jl")
vlm = FLOWVLM
```

## [MyPanel](https://github.com/EdoAlvarezR/MyPanel.jl/blob/master/src/MyPanel.jl)

Additional Dependencies for MyPanel
- ForwardDiff

Clone the package using Julia

```Pkg.clone("https://github.com/EdoAlvarezR/MyPanel.jl.git")```
