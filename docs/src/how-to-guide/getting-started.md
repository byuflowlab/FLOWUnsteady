# Getting Started

Before we install `FLOWUnsteady.jl`, let's install some of its dependencies.

## Setting up PyCall

The 'airfoilprep.py' package (wrapped by the 'AirfoilPrep.jl' package) is written in Python 2.7, so make sure that the Python version linked to `PyCall.jl` is 2.7. You can do this by running the following:

```julia
import Pkg
Pkg.add("PyCall")
ENV["PYTHON"] = "path/to/your/python2"
Pkg.build("PyCall")
```

Then close and reopen the Julia REPL, and run:

```julia
import PyCall
PyCall.pyversion
```

which should reveal your Python version:

```julia
v"2.7.16"
```

## `Paraview`

To visualize your simulations, we need to install the open-source visualization engine `Paraview`, which may be downloaded for free [here](https://www.paraview.org/download/). Versions `5.5.2` and `5.7.0` have been proven to work well. If you're using MacOS, you may need to export the folder with your `Paraview` executable to your path by running:

```bash
export PATH=$PATH:/path/to/your/paraview/
```

where your path likely looks like `/Applications/ParaView-5.5.2.app/Contents/MacOS/` (substituting `5.5.2` for your version number).

To test if `Paraview` is working well, you can pull up one of the examples under 'examples/' in the [`GeometricTools` repo](https://github.com/byuflowlab/GeometricTools.jl.git):

```julia
# Import Package
import Pkg
] add https://github.com/byuflowlab/GeometricTools.jl.git # type the `]` key to enter the package manager and add the GeometricTools Rep
import GeometricTools
gt = GeometricTools

# Load example
examplepath = joinpath(pathof(GeometricTools)[1:end-length("GeometricTools.jl")],"../examples/example_simple.jl")
include(examplepath)

# Run example: it will pull up paraview with a good-looking cube
simple_box2()
```

`Paraview` should open a rendering of a cube. You'll have to click the "eye" icon in the file tree next to `temp_vtk_example.vtk` to make it visible.

```@raw html
<img src="https://media.githubusercontent.com/media/byuflowlab/FLOWUnsteady/master/docs/src/assets/howtofigs/simple_cube.png" width="750"/>
```
<!-- 
## [FLOWVLM](https://github.com/byuflowlab/FLOWVLM)

Additional Dependencies for FLOWVLM:
- [CCBlade](https://github.com/byuflowlab/CCBlade.jl)
- [airfoil](https://github.com/EdoAlvarezR/airfoil)

### Getting CCBlade in the right version
To get CCBlade, clone the package in Julia

```Pkg.clone("https://github.com/byuflowlab/CCBlade.jl.git")```

You'll then need to go find your .julia/v0.6 directory (where all your v0.6 packages are stored, and go into the CCBlade directory and checkout the last commit before CCBlade was updated to v1.0

```git checkout bb897066c46bd10d0cad934af3fb677e4f9d0061```

### Getting airfoil
To get airfoil, clone the repo (not in Julia) and put it somewhere good for you

```git clone https://github.com/EdoAlvarezR/airfoil.git```

You will then need to go into the airfoil/src/jxlight directory and build the Fortran binary. If using MacOS, do

```make gfortran```

and that should be sufficient to build it.

Finally, clone the FLOWVLM repo in Julia

```Pkg.clone("https://github.com/byuflowlab/FLOWVLM.git")```

In addition, you will need to make a change in FLOWVLM.jl to make sure FLOWVLM is pointed to the airfoil code correctly. In FLOWVLM.jl (in the src folder of the FLOWVLM repo which is now in your .julia/v0.6 directory) change line 21 such that the airfoil_path is the path to the directory you just cloned.

```airfoil_path = "/path_to_airfoil/"```

## [MyPanel](https://github.com/EdoAlvarezR/MyPanel.jl/blob/master/src/MyPanel.jl)

Additional Dependencies for MyPanel
- ForwardDiff

Clone the package using Julia

```Pkg.clone("https://github.com/EdoAlvarezR/MyPanel.jl.git")```

# Setup Troubleshooting

Some things you might need to look out for:

1. Make sure your Homebrew (in Julia) is up to date. You may need to run the following:
```
using Homebrew
Homebrew.brew(`update-reset`)
```
in order to update your Homebrew.

2. You're going to have to make sure that things are in place in your Julia settings. Having things like Conda, HDF5, etc. on your machine doesn't necessarily mean that the Julia implementation has them as well.

3. If you don't have a fortran compiler, airfoil won't compile.  On MacOS, try `brew install gcc`

4. The airfoil compiler creates symbolic links. Right now, symbolic links do not work if you're trying to create them in [Box](http://box.byu.edu/), it won't work, you'll get the following error:
```
Linux - Gfortran
rm -f common.mk
ln -s ./config/config.LINUX_GFORTRAN.mk common.mk
ln: common.mk: Function not implemented
make: *** [gfortran] Error 1
```
So don't put airfoil in Box.

# Running things:

0. Clone the simulation engine (https://github.com/byuflowlab/FLOWUnsteady) to whenever you want it
1. Navigate to the fvs.jl file in the src/ directory.
2. change the `global extdrive_path` variable to be the directory where you want the files to be saved. But you can put it wherever you want.
3. If you don't yet have access to the VPM, you'll also need to comment out the lines:
```
import MyVPM
vpm = MyVPM
```
4. Now all you need to do is to include and run the example drive under the 'examples' folder:
```
include("vahana.jl")
visualize_maneuver_vahana()
```
and it should run, open an instance of paraview, and show the computed geometry and time steps. If you press play, you'll see the vahana evtol takeoff, cruise, and land! -->
