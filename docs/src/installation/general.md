# Installation Instructions

FLOWUnsteady is developed in the [Julia](https://julialang.org) programming
language, which is a modern, high-level, dynamic programming language for
high-performance computing. For visualization and postprocessing, FLOWUnsteady
uses [ParaView](https://www.paraview.org), which is an open-source software for
scientific and HPC visualization.

The following instructions will walk you from installing Julia through running
your first FLOWUnsteady simulation.


!!! info "Windows Users"
    If using Windows, please follow [these instructions](@ref windows) to set up
    Windows Subsystem for Linux before continuing.

## Julia

Download and install Julia: [julialang.org](https://julialang.org/downloads)

We recommend using v1.6.7 (long-term support) or the latest stable release.


## [ParaView](@id paraview)

Download and install ParaView: [paraview.org](https://www.paraview.org/download/)

Some of the examples call ParaView through the Julia command `run('paraview')`, which is equivalent to calling ParaView from the terminal by typing `paraview`. If you are using a Linux distribution, in order to be able to call ParaView with the command `paraview`, add the ParaView binary to the user-level `bin` folder through a symbolic link:
```bash
sudo ln -s /path/to/your/paraview /usr/local/bin/paraview
```

If you're using MacOS, you will need to export the folder with your ParaView executable to your path by running:
```bash
export PATH=$PATH:/path/to/your/paraview/
```
where your path likely looks like `/Applications/ParaView-5.5.2.app/Contents/MacOS/` (substituting `5.5.2` for your version number).

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

ParaView will then pull up with a rendering of a cube (click the `Apply` button under properties to make it visible).

```@raw html
<p align="center">
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/simple_cube.png" alt="Pic here" style="width: 75%;"/>
</p>
```


## [PyCall](@id pycall)

One of the dependencies, [AirfoilPrep.jl](https://github.com/byuflowlab/AirfoilPrep.jl), is a wrapper of Python code that is written in Python 3.8.
For this reason, make sure that your Python version linked to PyCall is 3.8.
You can do that as follows:
```julia
# Install PyCall
import Pkg; Pkg.add("PyCall")

# Point environment to your Python 3.8
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

## PyPlot
Since PyCall now relies on a custom install of Python3, make sure that:
1. matplotlib, mpmath, and scipy are installed in that Python
2. for optimal experience, verify that matplotlib uses the Qt5Agg backend. Useful instructions can be found [here](https://github.com/JuliaPy/PyPlot.jl#os-x) and [here](https://stackoverflow.com/questions/58627696/warning-pyplot-is-using-tkagg-backend-which-is-known-to-cause-crashes-on-macos).

If you run into errors with PyPlot while running FLOWUnsteady, they are likely related to one of those two items.

## FLOWVPM

FLOWVPM uses a
[fast multipole code](https://en.wikipedia.org/wiki/Fast_multipole_method)
called [ExaFMM](https://www.bu.edu/exafmm/)
that accelerates the computation of particle interactions.
ExaFMM is written in c++ and we have developed a Julia wrapper for it,
[FLOWExaFMM](https://github.com/byuflowlab/FLOWExaFMM.jl).

Before installing [FLOWVPM](https://github.com/byuflowlab/FLOWVPM.jl),
first you will have to install FLOWExaFMM and compile ExaFMM, as follows.

* [Julia REPL] Install CxxWrap:
  ```julia
  import Pkg
  Pkg.add(name="CxxWrap", version="0.11.2")
  ```

* [Terminal] Clone FLOWExaFMM:
  ```bash
  git clone https://github.com/byuflowlab/FLOWExaFMM path/to/FLOWExaFMM
  ```
  (replace `path/to/FLOWExaFMM` with your preferred path in your local)



* Compile ExaFMM running the script [`build.sh`](https://github.com/byuflowlab/FLOWExaFMM.jl/blob/master/build.sh):
  ```bash
  cd path/to/FLOWExaFMM
  sh build.sh
  ```
  or in MacOs:
  ```bash
  cd path/to/FLOWExaFMM
  sh build_macos.sh
  ```
  This should have generated the file `fmm.so` under `src/`, which is a binary
  library containing ExaFMM.

Now that ExaFMM is compiled, you can add FLOWExaFMM to your Julia environment as
a development package pointing directly to where you compiled the package and
add FLOWVPM:

* Add FLOWExaFMM:
  ```julia
  # In the Julia REPL
  ] develop path/to/FLOWExaFMM
  ```


* Test FLOWExaFMM:
  ```julia
  ] test FLOWExaFMM
  ```
  This will return a heart-warming "Hello world!" if ExaFMM was correctly compiled.


* Add FLOWVPM:
  ```julia
  ] add https://github.com/byuflowlab/FLOWVPM.jl
  ```


* Test FLOWVPM:
  ```julia
  ] test FLOWVPM
  ```


If you run into any issues, please try the following:
* Test that you can correctly compile C++ code wrapped for Julia following these instructions: [LINK](https://nbviewer.org/github/byuflowlab/FLOWVPM.jl/blob/master/docs/installation-linux.ipynb)
* Mac user may also need to take a look at these instructions: [LINK](https://github.com/byuflowlab/FLOWUnsteady/issues/26)
* Instructions for BYU Fulton supercomputer: [LINK](https://nbviewer.jupyter.org/url/edoalvar2.groups.et.byu.net/LabNotebook/202108/FLOWVPMSuperComputer.ipynb)

If issues persist, please check the resolved issues in [the FLOWExaFMM repo](https://github.com/byuflowlab/FLOWExaFMM.jl/issues)
and feel free to open a new issue.


!!! info "Windows"
    FLOWExaFMM currently supports Linux-based operative systems and MacOS.
    Windows users typically set up
    [a virtual machine with Linux](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
    to run scientific codes like FLOWUnsteady, and we recommend doing so.

    We currently provide limited support for Windows machines, and we would graciously
    accept pull requests automating the compilation of FLOWExaFMM on Windows.


## Unregistered Packages

The following dependencies are Julia packages that are not registered in the officially Julia registry.
For this reason, they need to be added manually through the following command in the Julia REPL:
```julia
] add github-url-to-the-package
```

| Package         | GitHub URL                                        |
| --------------: | :-----------                                      |
| AirfoilPrep     | [https://github.com/byuflowlab/AirfoilPrep.jl](https://github.com/byuflowlab/AirfoilPrep.jl)  |
| FLOWVLM         | [https://github.com/byuflowlab/FLOWVLM](https://github.com/byuflowlab/FLOWVLM)                |
| BPM             | [https://github.com/byuflowlab/BPM.jl](https://github.com/byuflowlab/BPM.jl)                  |
| FLOWNoise       | [https://github.com/byuflowlab/FLOWNoise](https://github.com/byuflowlab/FLOWNoise)            |

You can install all packages at once as follows:
```julia
import Pkg

url = "https://github.com/byuflowlab/"
packages = ("AirfoilPrep.jl", "FLOWVLM", "BPM.jl", "FLOWNoise")

Pkg.add([ Pkg.PackageSpec(; url=url*name) for name in packages ])
```

!!! info "Troubleshooting"
    Some things you might need to look out for:
    * Make sure your Homebrew (in Julia) is up to date. You may need to run the following in order to update your Homebrew:
      ```julia
        using Homebrew
        Homebrew.brew(`update-reset`)
      ```

    * Make sure that things are in place in your Julia settings. Having things like Conda, HDF5, etc. on your machine doesn't necessarily mean that the Julia implementation has them as well.

    * If you don't have a fortran compiler, Xfoil (one of the dependencies of AirfoilPrep) won't compile.  On MacOS, try `brew install gcc`

    * The `AirfoilPrep.jl` compiler creates symbolic links. Symbolic links do not work if you're trying to create them in [Box](http://box.byu.edu/). You'll get the following error:
      ```bash
        Linux - Gfortran
        rm -f common.mk
        ln -s ./config/config.LINUX_GFORTRAN.mk common.mk
        ln: common.mk: Function not implemented
        make: *** [gfortran] Error 1
      ```
      So don't put `AirfoilPrep.jl` in Box.


## Add FLOWUnsteady

You are now ready to install the FLOWUnsteady package through the following command in the Julia REPL:

```
] add https://github.com/byuflowlab/FLOWUnsteady
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


!!! info "CPU Parallelization"
    If any of the examples is taking longer than 10 to 20 minutes to run, it is
    possible that ExaFMM was compiled without OpenMPI, thus running in only one
    core as opposed to parallelizing the computation across all your CPU cores.

    To confirm that ExaFMM is successfully parallelized, pull up whatever CPU
    monitor is available in your operative system and confirm that Julia is
    using all your cores. For instance, the Gnome system monitor in Ubuntu
    should look like this:
    ![pic](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cpus00.png)
    and `htop` in the terminal should look like this
    ![pic](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cpus01.png)
