# Flight Vehicle Simulator

[![Vid here](docs/img/play00.png)](https://youtu.be/-xTHvwIe_34)

High-fidelity simulation engine of fully-unsteady flight vehicle. This module is
an experimental architecture stitching together mid and high-fidelity
aerodynamic tools developed at BYU's FLOW Lab: `GeometricTools` (geometric
engine), `FLOWVLM` (VLM and strip theory solver), `CCBlade` (blade element
momentum solver), `MyPanel` (3D inviscid panel solver), and `FLOWVPM` (viscous
vortex particle method). Most of the tools are still using Julia 0.6.3 as they
are pending for a major revamp, hence this simulation framework is conceived to
work only in Julia 0.6.3.

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



# Dependencies
  * [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
  * [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM)
  * [`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl)
  * `FLOWVPM`: Available upon request.
  * Paraview  : Not needed, but examples call Paraview for visualization of
      outputs.

# Related Projects
  * [Unsteady eVTOL transition](https://github.com/byuflowlab/alvarezanderson2020-unsteady-evtol-transition)
  * [Airborne wind energy design](https://github.com/byuflowlab/mehr2020-airborne-wind-aero)

# Authorship
  * Author    : Eduardo J Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT License
