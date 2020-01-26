# FLOW's Unsteady Aerodynamics Suite

<img src="docs/img/blownwing00.png" alt="Pic here" style="width: 900px;"/>

Simulation engine of mixed-fidelity unsteady aerodynamics and aeroacoustics.
This suite brings together mid and high-fidelity
aerodynamic tools developed at BYU's [FLOW Lab](http://flow.byu.edu/): [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
(geometric engine), [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM) (VLM and
strip theory solver), [`CCBlade`](https://github.com/byuflowlab/CCBlade.jl)
(blade element momentum solver),
[`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl) (3D inviscid panel
solver), and `FLOWVPM` (viscous vortex particle method). The aeroacoustics
solver uses PSU-WOPWOP (FW-H solver) and [`FLOWNoise`](https://github.com/byuflowlab/FLOWNoise).


This module is written in Julia 0.6.4 since most of the codes are still using
Julia 0.6.4 as they are pending for a major revamp.

**FEATURES**
* Viscous, unsteady wake mixing of rotors and lifting surfaces.
* Fully resolved rotor-on-rotor, rotor-on-wing, wing-on-rotor, and wing-on-wing
interactions.
* Fully resolved unsteady loads during prescribed kinematic maneuvers.

**LIMITATIONS**
* Viscous drag and separation is only captured through strip theory, without
attempting to shed separation wakes.
* No viscous drag is captured through VLM and panel models.

**FUTURE WORK**
* Coupling of aerodynamic loads and flight path allowing dynamic simulations.
* Bluff body separation and panel-predicted viscous drag (?).


For validation and numerical recommendations, check this notebook in the
documentation: [`docs/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FlightVehicleSim/blob/master/docs/validation.ipynb).

# Folders and Files
  * `src/`: Source code.
  * `examples/`: Example simulations.
  * `docs/`: Documentation (open Jupyter notebooks with [nbviewer](https://nbviewer.jupyter.org/)).
    - `docs/instructions-setup.md`: Instructions for setting up this package.
    - [`docs/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FlightVehicleSim/blob/master/docs/validation.ipynb): Validation of models implemented in this package, and numeric observations.

# Dependencies
  * [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
  * [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM)
  * [`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl)
  * `FLOWVPM`: Contact [Ed Alvarez](http://edoalvarez.com) or the FLOW Lab.
  * Paraview  : Not needed, but examples call Paraview for visualization of
      outputs.

# Related Projects
  * [Unsteady eVTOL transition](https://github.com/byuflowlab/alvarezanderson2020-unsteady-evtol-transition)
  * [Wind-harvesting aircraft design](https://github.com/byuflowlab/mehr2020-airborne-wind-aero)

# Examples
**HEAVING WING:** `examples/heavingwing.jl`
<img src="docs/vid/bertinsheaving00.gif" alt="Vid here" style="width: 900px;"/>

**CROSS-WIND CIRCULAR PATH:** `examples/circularpath.jl`
[<img src="docs/img/circlesim_wide.jpg" alt="Vid here" style="width: 900px;"/>](docs/vid/circularpath03_1.gif)

**HOVERING ROTOR:** `examples/singlerotor.jl`
<img src="docs/vid/fvs_singlerotor02.gif" alt="Vid here" style="width: 900px;"/>

**INTERACTING TANDEM HEAVING WING:** `examples/tandemheavingwing.jl`
[![Vid here](docs/img/play01.png)](https://youtu.be/Pch94bKpjrQ)


**BLOWN WING:** `examples/blownwing/blownwing.jl`
<img src="docs/img/blownwing02_wide.png" alt="Pic here" style="width: 900px;"/>


**eVTOL TRANSITION:** `examples/vahana/vahana.jl` (in progress)
[![Vid here](docs/img/play00.png)](https://youtu.be/-xTHvwIe_34)


# Authorship
  * Author    : Eduardo J Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT License
