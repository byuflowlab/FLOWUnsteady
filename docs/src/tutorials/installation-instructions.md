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

To test if `Paraview` is working well, you can pull up one of the examples under `examples/` in the [`GeometricTools.jl`](https://github.com/byuflowlab/GeometricTools.jl.git) repo:

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

### Setting up FLOWExaFMM and FLOWVPM for Mac

#### Set up CxxWrap
Once you have received access to FLOWExaFMM and FLOWVPM (linked in the above instructions LINK), you need to first set up CxxWrap.  Start by adding CxxWrap:
```
]
add CxxWrap
test CxxWrap
```

!!! note "Cmake"
    This will likely throw an error if you do not have Cmake installed on your system.  For Mac, you may consider using homebrew to install cmake: `brew install cmake`

Once CxxWrap is added and has passed it's own tests, you may consider trying out the hello world example described in that package documentation.

#### Set Up FLOWExaFMM
Once CxxWrap is working, you can use it to find where Julia keeps the jlcxx files:

```julia
import CxxWrap
CxxWrap.prefix_path()
```

You will need to save the output of `CxxWrap.prefix_path()` somewhere to use shortly.

Now go ahead and add FLOWExaFMM
```
]
add https://github.com/byuflowlab/FLOWExaFMM
```

You may be asked to provide your credentials for github in order to add this package (thus you need to get access beforehand).

After the package has been added, locate it in the terminal.

```
cd .julia/packages/FLOWExaFMM/
```

Then `cd` into the subdirectory there (or the one for the most current version you just added).

Once there, locate the make3d.sh file. If make3d.sh does not exist, create it, and copy in
```bash
export JlCxx_DIR= #paste output of CxxWrap.prefix_path() here
THIS_DIR=$(pwd)
BUILD_DIR=${THIS_DIR}/build
rm -rf ${BUILD_DIR}
mkdir ${BUILD_DIR}
cd ${BUILD_DIR}
echo "Configuring build..."
echo "  Source path: ${THIS_DIR}/deps/3d"
CC=clang CXX=clang++ cmake ${THIS_DIR}/deps/3d
echo "Making files..."
cmake --build .
echo "Copying dynamic library..."
cp lib/libfmm.dylib ../src/fmm.dylib
```

Find line that begins with
```
export JlCxx_DIR=
```

This is where you paste the output of `CxxWrap.prefix_path()` you saved earilier.

Now `cd` into the `deps/3d/` directory and makesure that CMakeLists.txt is there. If it is not, create it and copy the following in.
```
project(FMM CXX)

cmake_minimum_required(VERSION 3.12)  # min required version for os x with openmp
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_VERBOSE_MAKEFILE ON)  # turn on if you want verbose
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# apple-specific changes
set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -Wno-unused-command-line-argument")
set(OpenMP_CXX_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY ${OpenMP_CXX_LIB_NAMES})

# set traverse flag
set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} -DEXAFMM_EAGER")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEXAFMM_EAGER")

# auto find cxx and openmp
find_package(JlCxx)
find_package(OpenMP REQUIRED)
get_target_property(JlCxx_location JlCxx::cxxwrap_julia LOCATION)
get_filename_component(JlCxx_location ${JlCxx_location} DIRECTORY)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${JlCxx_location}")

message(STATUS "Found JlCxx at ${JlCxx_location}")

# set include and library locations
add_library(fmm SHARED fmm.cxx)
target_include_directories(fmm PRIVATE ${CMAKE_SOURCE_DIR})
target_link_directories(fmm PRIVATE "/usr/local/lib")  # USER: probably not needed on linux, but not auto found in mac/homebrew
target_link_libraries(fmm PRIVATE JlCxx::cxxwrap_julia OpenMP::OpenMP_CXX)

install(TARGETS
  fmm
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION lib)
```

You can now go back up to the top level and run the make3d.sh file.

You should get an output similar to:
```
Configuring build...
  Source path: ~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d
-- The CXX compiler identification is Clang 10.0.1
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/local/bin/clang++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Found Julia executable: /Applications/Julia-1.4.app/Contents/Resources/julia/bin/julia
-- Julia_VERSION_STRING: 1.4.2
-- Julia_INCLUDE_DIRS:   /Applications/Julia-1.4.app/Contents/Resources/julia/include/julia
-- Julia_LIBRARY_DIR:    /Applications/Julia-1.4.app/Contents/Resources/julia/lib
-- Julia_LIBRARY:        /Applications/Julia-1.4.app/Contents/Resources/julia/lib/libjulia.1.4.dylib
-- JULIA_HOME:           /Applications/Julia-1.4.app/Contents/Resources/julia/bin
-- Julia_LLVM_VERSION:   v8.0.1
-- Julia_WORD_SIZE:      64
-- Found Julia: /Applications/Julia-1.4.app/Contents/Resources/julia/lib/libjulia.1.4.dylib (found version "1.4.2")
-- Found OpenMP_CXX: -Xpreprocessor -fopenmp -Wno-unused-command-line-argument (found version "4.5")
-- Found OpenMP: TRUE (found version "4.5")
-- Found JlCxx at ~/.julia/artifacts/6017255205dc4fbf4d962903a855a0c631f092dc/lib
-- Configuring done
-- Generating done
-- Build files have been written to: ~/.julia/packages/FLOWExaFMM/iM4yf/build
Making files...
/usr/local/Cellar/cmake/3.18.2/bin/cmake -S~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d -B~/.julia/packages/FLOWExaFMM/iM4yf/build --check-build-system CMakeFiles/Makefile.cmake 0
/usr/local/Cellar/cmake/3.18.2/bin/cmake -E cmake_progress_start ~/.julia/packages/FLOWExaFMM/iM4yf/build/CMakeFiles ~/.julia/packages/FLOWExaFMM/iM4yf/build//CMakeFiles/progress.marks
/Library/Developer/CommandLineTools/usr/bin/make  -f CMakeFiles/Makefile2 all
/Library/Developer/CommandLineTools/usr/bin/make  -f CMakeFiles/fmm.dir/build.make CMakeFiles/fmm.dir/depend
cd ~/.julia/packages/FLOWExaFMM/iM4yf/build && /usr/local/Cellar/cmake/3.18.2/bin/cmake -E cmake_depends "Unix Makefiles" ~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d ~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d ~/.julia/packages/FLOWExaFMM/iM4yf/build ~/.julia/packages/FLOWExaFMM/iM4yf/build ~/.julia/packages/FLOWExaFMM/iM4yf/build/CMakeFiles/fmm.dir/DependInfo.cmake --color=
Scanning dependencies of target fmm
/Library/Developer/CommandLineTools/usr/bin/make  -f CMakeFiles/fmm.dir/build.make CMakeFiles/fmm.dir/build
[ 50%] Building CXX object CMakeFiles/fmm.dir/fmm.cxx.o
/usr/local/bin/clang++ -DJULIA_ENABLE_THREADING -Dfmm_EXPORTS -I~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d -isystem ~/.julia/artifacts/6017255205dc4fbf4d962903a855a0c631f092dc/include -isystem /Applications/Julia-1.4.app/Contents/Resources/julia/include/julia -Xpreprocessor -fopenmp -Wno-unused-command-line-argument -DEXAFMM_EAGER -DEXAFMM_EAGER -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk -fPIC -Xpreprocessor -fopenmp -Wno-unused-command-line-argument -std=gnu++17 -o CMakeFiles/fmm.dir/fmm.cxx.o -c ~/.julia/packages/FLOWExaFMM/iM4yf/deps/3d/fmm.cxx
[100%] Linking CXX shared library lib/libfmm.dylib
/usr/local/Cellar/cmake/3.18.2/bin/cmake -E cmake_link_script CMakeFiles/fmm.dir/link.txt --verbose=1
/usr/local/bin/clang++ -Xpreprocessor -fopenmp -Wno-unused-command-line-argument -DEXAFMM_EAGER -DEXAFMM_EAGER -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk -dynamiclib -Wl,-headerpad_max_install_names -o lib/libfmm.dylib -install_name @rpath/libfmm.dylib CMakeFiles/fmm.dir/fmm.cxx.o   -L/usr/local/lib  -Wl,-rpath,/usr/local/lib -Wl,-rpath,~/.julia/artifacts/6017255205dc4fbf4d962903a855a0c631f092dc/lib -Wl,-rpath,/Applications/Julia-1.4.app/Contents/Resources/julia/lib ~/.julia/artifacts/6017255205dc4fbf4d962903a855a0c631f092dc/lib/libcxxwrap_julia.0.8.0.dylib /Applications/Julia-1.4.app/Contents/Resources/julia/lib/libjulia.1.4.dylib -lomp
[100%] Built target fmm
/usr/local/Cellar/cmake/3.18.2/bin/cmake -E cmake_progress_start ~/.julia/packages/FLOWExaFMM/iM4yf/build/CMakeFiles 0
Copying dynamic library...
```

If all went to plan, you should be able to now go into julia and run the 'hello world' test set up there.
```
import FLOWExaFMM
@show FLOWExaFMM.greet()
```

#### Set up FLOWVPM
Make sure that you have the HDF5 library installed on your machine. You can do that via homebrew with
```
brew install HDF5
```

In Julia, add the FLOWVPM package.
```
]
add https://github.com/byuflowlab/FLOWVPM
```

## Install FLOWUnsteady

Now you are ready to install the FLOWUnsteady package through the following command in the Julia REPL:

```
] add https://github.com/byuflowlab/FLOWUnsteady
```

# Setup Troubleshooting

Some things you might need to look out for:

1. Make sure your Homebrew (in Julia) is up to date. You may need to run the following in order to update your Homebrew:
```julia
  using Homebrew
  Homebrew.brew(`update-reset`)
```

2. You're going to have to make sure that things are in place in your Julia settings. Having things like Conda, HDF5, etc. on your machine doesn't necessarily mean that the Julia implementation has them as well.

3. If you don't have a fortran compiler, Xfoil (one of the dependencies of AirfoilPrep) won't compile.  On MacOS, try `brew install gcc`

4. The `AirfoilPrep.jl` compiler creates symbolic links. Symbolic links do not work if you're trying to create them in [Box](http://box.byu.edu/). You'll get the following error:
```bash
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
