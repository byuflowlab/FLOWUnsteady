# [Installation Instructions](@id installation)

FLOWUnsteady is developed in the [Julia](https://julialang.org) programming
language, which is a modern, high-level, dynamic programming language for
high-performance computing. For visualization and postprocessing, FLOWUnsteady
uses [ParaView](https://www.paraview.org), which is an open-source software for
scientific and HPC visualization.

The following instructions walk you through how to install Julia, ParaView, and
FLOWUnsteady.


!!! info "Windows Users"
    On Windows please follow [these instructions](@ref windows) first to
    set up Windows Subsystem for Linux.

## Julia

* Download and install Julia: [julialang.org](https://julialang.org/downloads)
  (currently we are supporting up to Julia v1.10, so we recommend using
    [v1.10.2](https://julialang.org/downloads) or
    [v1.6.7 LTS](https://julialang.org/downloads/#long_term_support_release)
    )
* Add Julia to user-level `bin` folder
  ```bash
  sudo ln -s /[user-specific-path/Julia-1.x.x]/bin/julia /usr/local/bin/julia
  ```
  Replace `/[user-specific-path/Julia-1.x.x]/` with the path where Julia got
  installed.
  For instance, in MacOS the full path looks like this:
  `/Applications/Julia-1.10.app/Contents/Resources/julia/bin/julia`

If you were successfull, typing the following in the terminal will launch the
Julia REPL:
```bash
julia
```


## [ParaView](@id paraview)

* Download and install ParaView: [paraview.org](https://www.paraview.org/download/)
* Add ParaView to user-level `bin` folder
  ```bash
  sudo ln -s /[user-specific-path/ParaView-5.x.x]/paraview /usr/local/bin/paraview
  ```
  Replace `/[user-specific-path/ParaView-5.x.x]/` with the path where ParaView got
  installed.
  For instance, in MacOS the full path looks like this:
  `/Applications/ParaView-5.12.0.app/Contents/MacOS/paraview`

If you were successfull, typing the following in the terminal will launch the
ParaView:
```bash
paraview
```

To test that ParaView is correctly installed, run one of the examples
of the [GeometricTools](https://github.com/byuflowlab/GeometricTools.jl) package as follows
(copy/paste this in the Julia REPL):

```julia
# Install GeometricTools
import Pkg; Pkg.add("GeometricTools")

import GeometricTools as gt

# Load example
examplepath = joinpath(dirname(pathof(gt)), "..", "examples", "example_simple.jl")
include(examplepath)

# Run example: it will pull up paraview with a good-looking cube
simple_box2()
```

ParaView will then pull up with a rendering of a cube (click the `Apply` button under `Properties` to make it visible).

```@raw html
<p align="center">
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/simple_cube.png" alt="Pic here" style="width: 75%;"/>
</p>
```

## *(Optional)* OpenVSP
FLOWUnsteady can import geometry created in [OpenVSP](https://openvsp.org/) using [VSPGeom.jl](https://github.com/byuflowlab/VSPGeom.jl).
We recommend [installing OpenVSP](https://openvsp.org/download.php) in your system, then adding VSPGeom.jl to Julia:
```julia
] add VSPGeom
```


## [PyCall](@id pycall)

One of the dependencies ([AirfoilPrep.jl](https://github.com/byuflowlab/AirfoilPrep.jl)) is a wrapper of Python code that is written in Python v3.8+.
For this reason, make sure that your Python version linked to PyCall is 3.8 or higher.
You can do that as follows:
```julia
# Install PyCall
import Pkg; Pkg.add("PyCall")

# Point environment to your Python 3.8 (or higher)
ENV["PYTHON"] = "path/to/your/python3"

# Rebuild PyCall
Pkg.build("PyCall")
```
Then close and reopen the Julia REPL, and run:
```julia
import PyCall
PyCall.pyversion
```
which should reveal your Python version:
```julia
v"3.8"
```

Since PyCall now relies on a custom install of Python3, make sure that:
* matplotlib, mpmath, and scipy are installed in that Python,
  ```bash
  pip3 install matplotlib mpmath scipy --user
  ```
* For optimal experience, verify that matplotlib uses the Qt5Agg backend. Useful instructions can be found [here](https://github.com/JuliaPy/PyPlot.jl#os-x) and [here](https://stackoverflow.com/questions/58627696/warning-pyplot-is-using-tkagg-backend-which-is-known-to-cause-crashes-on-macos).

If you run into errors with PyPlot while running FLOWUnsteady, they are likely related to one of those two items.

## Add FLOWUnsteady

You are now ready to install the FLOWUnsteady package.

In the terminal clone the following repos:
```
git clone https://github.com/byuflowlab/FLOWUnsteady
git clone https://github.com/byuflowlab/AirfoilPrep.jl.git
git clone https://github.com/byuflowlab/FLOWVLM.git
git clone https://github.com/byuflowlab/FLOWNoise.git
git clone https://github.com/byuflowlab/BPM.jl.git
git clone https://github.com/byuflowlab/FLOWVPM.jl.git
cd ./FLOWUnsteady
```

Now, dev each of the above repos into FLOWUnsteady. 

In the julia REPL perform the following commands:
```julia
import Pkg
Pkg.activate(".")
url = "https://github.com/byuflowlab/"
Pkg.rm([("AirfoilPrep"), ("FLOWVLM"), ("FLOWNoise"), ("BPM"), ("FLOWVPM")])
Pkg.develop(path="../AirfoilPrep.jl")
Pkg.develop(path="../FLOWVLM")
Pkg.develop(path="../FLOWNoise")
Pkg.develop(path="../BPM.jl")
Pkg.develop(path="../FLOWVPM.jl")
```

Then activate the desired project environment and dev the FLOWUnsteady folder as well as the other added packages.
```julia
Pkg.activate() 
Pkg.develop(path=".")
Pkg.develop(path="../AirfoilPrep.jl")
Pkg.develop(path="../FLOWVLM")
Pkg.develop(path="../FLOWNoise")
Pkg.develop(path="../BPM.jl")
Pkg.develop(path="../FLOWVPM.jl")
```

# Test FLOWUnsteady

In order to test that FLOWUnsteady and all dependencies were successfully installed, try running some of the examples under [`FLOWUnsteady/examples/`](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/).

For instance, you can run the [Simple Wing](@ref simple_wing) example
as follows:

```julia
import FLOWUnsteady as uns

include(joinpath(uns.examples_path, "wing", "wing.jl"))
```

or you can run the [Tethered Wing](@ref) example:
```julia
import FLOWUnsteady as uns

include(joinpath(uns.examples_path, "tetheredwing.jl"))
```

This will pull up Paraview visualizing the simulation. Kick off your shoes, sit back, and
enjoy the simulation that you have just run.

```@raw html
<br><br>

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/tetheredwing-example-00small.gif" alt="Vid here" style="width: 80%;"/>
</center>

<br><br><br><br>
<br><br><br><br>
```


!!! info "2D View"
    When opening a simulation with a flat surface (like `wing.jl`), ParaView
    automatically activates its 2D view mode. Disable the 2D view by clicking
    these two buttons:
    ![pic](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/paraview-2d00.png)

!!! info "CPU Parallelization"
    If any of the examples is taking longer than 10 to 20 minutes to run, it is
    possible that ExaFMM was compiled without OpenMP, thus running in only one
    core as opposed to parallelizing the computation across all your CPU cores.

    To confirm that ExaFMM is successfully parallelized, pull up whatever CPU
    monitor is available in your operative system and confirm that Julia is
    using all the cores as the simulation is running. For instance, the
    Resources tab of the Task Manager in Windows should look like this:
    ![pic](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cpus02.png)
    and `htop` in the terminal (Linux and MacOS) should look like this:
    ![pic](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cpus01-2.png)
