# How to set up dependencies needed for FlightVehicleSim (October 2019)
Note: currently this simulation tool functions only with Julia v0.6, thus the following will assume Julia v0.6 syntax.

## Setting up Julia v0.6.4
Most likely you have the latest version already installed in your system. In order to have multiple versions of Julia do the following
* Download the v0.6.4 binary from https://julialang.org/downloads/oldreleases.html
* Extract it anywhere, in my case it's located at '/home/edo/Programs/julia-0.6.4'. The exectuable binary is located at '/home/edo/Programs/julia-0.6.4/bin/julia'.
* Make a simbolic link in your system binaries to the Julia v0.6.4 binary: In my Linux machine, the binaries are located at '/usr/bin/'. Go to that path as 'cd /usr/bin/', then create the link through the following command 'sudo ln -s /home/edo/Programs/julia-0.6.4/bin/julia julia-0.6.4' replacing the path to where your julia binary is. To check that it worked, open a new terminal and do 'julia-0.6.4'.

## Setting up PyCall
'airfoilprep.py' in the 'airfoil' module is written in Python 2.7, so you need to make sure that the Python version that is linked to PyCall is a 2.7 version. My recommendation is to use the python version that the system is using, which most-likely is 2.7 (do 'which python' in the terminal to find the path to that Python binary). Hopefully the binary of python you choose is the one that is linked with 'pip' in your system, as that will save you tons of headaches. Then set up PyCall on that python version as

```
  Pkg.add("PyCall")
  ENV["PYTHON"] = "path/to/your/python"
  Pkg.build("Pycall")
```
Then close and open the julia terminal again.

## [GeometricTools](https://github.com/byuflowlab/GeometricTools.jl): 

Dependencies:
- PyCall
- PyPlot
- Dierckx
- Roots
- QuadGK

Go ahead and add these Julia packages first with the standard add syntax for v0.6

```Pkg.add("PackageName")```

After you have the dependencies, clone the package using Julia. (Though you can get the dependencies later when errors show up. It's up to you.)

```Pkg.clone("https://github.com/byuflowlab/GeometricTools.jl.git")```

If using MacOS, you'll want to add 

```export PATH=$PATH:/Applications/ParaView-5.5.2.app/Contents/MacOS/```

to your .bash_profile, swapping the 5.5.2 for whatever version of [Paraview](https://www.paraview.org/download/) you have downloaded (you'll need to download it if you don't already have it).

To test if GeometricTools + Paraview is working well, you can pull up one of the examples under 'examples/' in the GeometricTools repo:
```
  # Import Package
  import GeometricTools
  gt = GeometricTools
  
  # Load example
  example_path = "examples/example_simple.jl"
  include(joinpath(Pkg.dir("GeometricTools"), example_path))
  
  # Run example: it will pull up paraview with a good-looking cube
  simple_box2()
```

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

0. Clone the simulation engine (https://github.com/byuflowlab/FlightVehicleSim) to whenever you want it
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
and it should run, open an instance of paraview, and show the computed geometry and time steps. If you press play, you'll see the vahana evtol takeoff, cruise, and land!
