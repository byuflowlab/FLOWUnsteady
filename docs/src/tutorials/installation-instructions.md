# Installation Instructions

In order to install `FLOWUnsteady.jl`, set Julia up in your system (download from [https://julialang.org/downloads/](https://julialang.org/downloads/)) and follow the instructions listed here below.


!!! note "Julia version"
    FLOWUnsteady has been developed and tested in Julia v1.4.2, hence we recommend using this version.

## Setting up PyCall

The `airfoilprep.py` package (wrapped by the [`AirfoilPrep.jl`](https://github.com/byuflowlab/AirfoilPrep.jl) package) is written in Python 2.7, so make sure that the Python version linked to `PyCall.jl` is 2.7. After installing PyCall (`] add PyCall`), you can do this by running the following:

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

In order to visualize simulations, you need to install the open-source visualization engine `Paraview`, which may be downloaded [here](https://www.paraview.org/download/) for free. Any version will work well.

Some of the examples call Paraview through the Julia command `run('paraview')`, which is equivalent to calling Paraview from the terminal by typing `paraview`. If you are using a Linux distribution, in order to be able to call Paraview through `paraview`, add the Paraview binary to the user-level `bin` folder through a symbolic link:
```bash
sudo ln -s /path/to/your/paraview /usr/local/bin/paraview
```
If you're using MacOS, you will need to export the folder with your `Paraview` executable to your path by running:
```bash
export PATH=$PATH:/path/to/your/paraview/
```

where your path likely looks like `/Applications/ParaView-5.5.2.app/Contents/MacOS/` (substituting `5.5.2` for your version number).

To test if `Paraview` is working well, you can pull up one of the examples under `examples/` in the [`GeometricTools.jl` repo](https://github.com/byuflowlab/GeometricTools.jl.git):

```julia
# type the `]` key to enter the package manager and add GeometricTools
] add https://github.com/byuflowlab/GeometricTools.jl
import GeometricTools
gt = GeometricTools

# Load example
examplepath = joinpath(dirname(pathof(gt)), "..", "examples", "example_simple.jl")
include(examplepath)

# Run example: it will pull up paraview with a good-looking cube
simple_box2()
```

`Paraview` should open a rendering of a cube. You'll have to click the `Apply` button under properties to make it visible.

![Img](../assets/howtofigs/simple_cube.png)


## FLOW's Unregistered Dependencies

The following dependencies are unregistered Julia packages that need to be added manually through the command `] add github-url-to-the-package` in the Julia REPL.

* GeometricTools: [https://github.com/byuflowlab/GeometricTools.jl](https://github.com/byuflowlab/GeometricTools.jl)
* Xfoil: [https://github.com/byuflowlab/Xfoil.jl](https://github.com/byuflowlab/Xfoil.jl)
* AirfoilPrep: [https://github.com/byuflowlab/AirfoilPrep.jl](https://github.com/byuflowlab/AirfoilPrep.jl)
* FLOWVLM: [https://github.com/byuflowlab/FLOWVLM](https://github.com/byuflowlab/FLOWVLM)
* BPM: [https://github.com/byuflowlab/BPM.jl](https://github.com/byuflowlab/BPM.jl)
* FLOWNoise: [https://github.com/byuflowlab/FLOWNoise](https://github.com/byuflowlab/FLOWNoise)
* FLOWVPM: Contact Ed Alvarez ([edoalvarez.com](https://edoalvarez.com)) or the FLOW Lab ([flow.byu.edu](http://flow.byu.edu/)), and follow this instructions: [LINK](https://nbviewer.jupyter.org/url/edoalvar2.groups.et.byu.net/LabNotebook/202008/FLOWVPMSetupFinal.ipynb).
* (Optional) MyPanel: [https://github.com/EdoAlvarezR/MyPanel.jl](https://github.com/EdoAlvarezR/MyPanel.jl)

## Install FLOWUnsteady

Now you are ready to install the FLOWUnsteady package through the following command in the Julia REPL:

```
] add https://github.com/byuflowlab/FLOWUnsteady
```

# Setup Troubleshooting

Some things you might need to look out for:

1. Make sure your Homebrew (in Julia) is up to date. You may need to run the following in order to update your Homebrew:
> ```julia
    using Homebrew
    Homebrew.brew(`update-reset`)
    ```

2. You're going to have to make sure that things are in place in your Julia settings. Having things like Conda, HDF5, etc. on your machine doesn't necessarily mean that the Julia implementation has them as well.

3. If you don't have a fortran compiler, Xfoil (one of the dependencies of AirfoilPrep) won't compile.  On MacOS, try `brew install gcc`

4. The `AirfoilPrep.jl` compiler creates symbolic links. Symbolic links do not work if you're trying to create them in [Box](http://box.byu.edu/). You'll get the following error:
>```
Linux - Gfortran
rm -f common.mk
ln -s ./config/config.LINUX_GFORTRAN.mk common.mk
ln: common.mk: Function not implemented
make: *** [gfortran] Error 1
```
So don't put `AirfoilPrep.jl` in Box.

# Running The Examples

In order to test that FLOWUnsteady and all dependencies were successfully installed, try running some of the examples under [`FLOWUnsteady/examples/`](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/) (the outputs of some of the examples are shown in this notebook: [`docs/resources/examples.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/examples.ipynb)).

For instance, you can run the tandem heaving wing example through the following commands in the Julia REPL:

```julia
import FLOWUnsteady

# Path to examples folder
path_to_examples = joinpath(dirname(pathof(FLOWUnsteady)), "..", "examples")

# Include tandem heaving wing example
include(joinpath(path_to_examples, "tandemheavingwing.jl"))

# Run tandem heaving wing example
tandemheavingwing(;
                    VehicleType=FLOWUnsteady.QVLMVehicle,   # Quasi-steady solver
                    # VehicleType=FLOWUnsteady.UVLMVehicle, # Unsteady solver (requires FLOWVPM)
                    save_path="tandemheaving-example/"
                  );

# OPTIONAL: Call Paraview for visualization
vtk_files = "tandemheaving-example/bertins_Main_vlm...vtk;bertins_Tilting_Tandem_vlm...vtk;bertins_pfield...vtk;"
run(`paraview --data=$(vtk_files)`)
```

This will pull up Paraview showing the computed geometry and time steps. Sit back, press play, and enjoy the simulation that you have just run (it should look like the video shown below).
[![Vid here](../assets/img/play01_wide.png)](https://youtu.be/Pch94bKpjrQ)


!!! note "Quasi-steady solver"
    If you don't have access to the VPM code yet, you can run any of the examples with the quasi-steady solver. This is done by switching the vehicle type from the unsteady VLM (`FLOWUnsteady.UVLMVehicle`) to the quasi-steady VLM vehicle (`FLOWUnsteady.QVLMVehicle`). In the example above, you only need to change the `VehicleType` keyword argument to `VehicleType=FLOWUnsteady.QVLMVehicle`. This is what the quasi-steady simulation looks like:

```@raw html
<img src="../../assets/vid/tandemheaving142_1.gif" alt="Vid" width="600px"/>
```
