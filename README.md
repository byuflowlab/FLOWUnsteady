# Flight Vehicle Simulator

[![Vid here](docs/img/play00.png)](https://youtu.be/-xTHvwIe_34)

High-fidelity simulation engine of fully-unsteady flight vehicle. This module is
an experimental architecture stitching together mid and high-fidelity
aerodynamic tools developed at BYU's FLOW Lab: [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
(geometric engine), [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM) (VLM and
strip theory solver), [`CCBlade`](https://github.com/byuflowlab/CCBlade.jl)
(blade element momentum solver),
[`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl) (3D inviscid panel
solver), and `FLOWVPM` (viscous vortex particle method). Most of the tools are
still using Julia 0.6.4 as they are pending for a major revamp, hence this
simulation framework is conceived to work only in Julia 0.6.4.

FEATURES
* Viscous, unsteady wake mixing of rotors and lifting surfaces.
* Fully resolved rotor-on-rotor, rotor-on-wing, wing-on-rotor, and wing-on-wing
interactions.
* Fully resolved unsteady loads during prescribed kinematic maneuvers.

LIMITATIONS
* Separation is only captured trough strip theory, without attempting to shed
separation wakes. Also,
* no viscous drag is captured through VLM and panel models. Besides that,
* UNLIMITED POWER!

FUTURE WORK
* Coupling of aerodynamic loads and flight path allowing dynamic simulations.
* Bluff body separation and panel-predicted viscous drag (?).


For validation and numerical recommendations, check this notebook in the
documentation: [`docs/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FlightVehicleSim/blob/master/docs/validation.ipynb).

# Files
  * `src/fvs_geometry.jl`: Geometries are defined here.
  * `src/fvs_kinematics.jl`: Kinematics of different maneuvers are defined here.
  * `examples/`: Some example simulations.
  * `docs/`: Documentation (open Jupyter notebooks with [nbviewer](https://nbviewer.jupyter.org/)).
    - `docs/instructions-setup.md`: Instructions for setting up this package.
    - [`docs/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FlightVehicleSim/blob/master/docs/validation.ipynb): Validation of models implemented in this package, and numeric observations.

# Dependencies
  * [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
  * [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM)
  * [`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl)
  * `FLOWVPM`: Contact Ed Alvarez or the FLOW Lab.
  * Paraview  : Not needed, but examples call Paraview for visualization of
      outputs.

# Related Projects
  * [Unsteady eVTOL transition](https://github.com/byuflowlab/alvarezanderson2020-unsteady-evtol-transition)
  * [Wind-harvesting aircraft design](https://github.com/byuflowlab/mehr2020-airborne-wind-aero)

# Examples
**HEAVING WING:** `examples/heavingwing.jl`
<img src="docs/vid/bertinsheaving00.gif" alt="Vid here" style="width: 900px;"/>

**CROSS-WIND CIRCULAR PATH:** `examples/circularpath.jl`
<img src="docs/vid/circularpath03_1.gif" alt="Vid here" style="width: 900px;"/>

**INTERACTING TANDEM HEAVING WING:** `examples/tandemheavingwing.jl`
[![Vid here](docs/img/play01.png)](https://youtu.be/Pch94bKpjrQ)

# Authorship
  * Author    : Eduardo J Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT License
