# Welcome to FLOWUnsteady

![blown wing](../resources/img/blownwing00.png)

FLOWUnsteady is a simulation engine of mixed-fidelity unsteady aerodynamics and aeroacoustics.
This suite brings together mid and high-fidelity
aerodynamics tools developed at BYU's [FLOW Lab](http://flow.byu.edu/): [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
(geometric engine), [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM) (VLM and
strip theory solver), [`CCBlade`](https://github.com/byuflowlab/CCBlade.jl)
(blade element momentum solver),
[`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl) (3D inviscid panel
solver), and `FLOWVPM` (viscous vortex particle method). The aeroacoustic
solver integrates PSU-WOPWOP (FW-H solver) and [`FLOWNoise`](https://github.com/byuflowlab/FLOWNoise) (BPM code).

This package was written for Julia 1.4.2.

If you are brand-new to FLOWUnsteady, you should begin with our [Getting Started](@ref) guide in the Tutorials.  You may then consider taking a glance at how to [Visualize with Paraview](@ref) in order to be prepared to visualize as you go.

After getting the code set up, you'll be able to take your [First Steps](@ref) with FLOWUnsteady.

For additional validation and numerical recommendations, check this notebook: [`docs/resources/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/validation.ipynb).

For example simulations, check this notebook: [`docs/resources/examples.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/examples.ipynb).

For an example and validation of aeroacoustics, check this notebook: [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb).

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


# Examples
**HEAVING WING:** `examples/heavingwing.jl`
![blown wing](../resources/vid/bertinsheaving00.gif)

**CROSS-WIND CIRCULAR PATH:** `examples/circularpath.jl`
![blown wing](../resources/vid/circularpath03_1.gif)

**HOVERING ROTOR:** `examples/singlerotor.jl`
![blown wing](../resources/vid/fvs_singlerotor02.gif)

**INTERACTING TANDEM HEAVING WING:** `examples/tandemheavingwing.jl`
[![Vid here](../resources/img/play01_wide.png)](https://youtu.be/Pch94bKpjrQ)


**BLOWN WING:** `examples/blownwing/blownwing.jl`
[![Vid here](../resources/img/blownwingplay03.png)](https://youtu.be/3REcIdIXrZA)


**eVTOL TRANSITION:** `examples/vahana/vahana.jl` (in progress)
[![Vid here](../resources/img/play00_wide.png)](https://youtu.be/-xTHvwIe_34)

# Framework Flowchart
<img src=".../resources/img/flowunstrady_diagram03.png" alt="Pic here" style="width: 900px;"/>

# Publications
  * Alvarez, E. J., & Ning, A. (2021, in progress). <i>Unsteady Mixed-fidelity Aerodynamics Solver
for Maneuvering Multirotor Aircraft</i>. AIAA SciTech Forum. <a href="https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/AlvarezNing_2021-SciTechAbstract-FLOWUnsteady_solver.pdf"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A. (2020, in review). <i>Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover</i>. Journal of the American Helicopter Society. <br><br>
  * Alvarez, E. J., (2020). <i>Quasi-steady Aerodynamics Solver for a High-fidelity Controls Framework</i>. FLOWUnsteady Documentation. <a href="https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/quasisteadysolver.pdf"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2020, accepted). <i>High-fidelity Modeling of Multirotor Aerodynamic Interactions for Aircraft Design</i>. AIAA Journal. <a href="https://byu.box.com/shared/static/s4h4w165x4vlxcmhgmxi05ngq0qab9o0.pdf"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2019). <i>Modeling Multirotor Aerodynamic Interactions Through the Vortex Particle Method</i>. AIAA AVIATION Forum. DOI: <a href="https://doi.org/10.2514/6.2019-2827 ">10.2514/6.2019-2827</a> <a href="http://edoalvar2.groups.et.byu.net/public/AlvarezNing_2019-AVIATION-Multirotor_aerodynamic_interactions_through_VPM-STATIC.pdf"><b>[SLIDES]</b></a><a href="https://scholarsarchive.byu.edu/facpub/3191/"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2018). <i>Development of a Vortex Particle Code for the Modeling of Wake Interaction in Distributed Propulsion</i>. AIAA AVIATION Forum. DOI: <a href="https://doi.org/10.2514/6.2018-3646 ">10.2514/6.2018-3646</a> <a href="http://www.et.byu.edu/~edoalvar/public/AlvarezNing_2018-AIAA-VPM_distibuted_propulsion-SLIDE-static.pdf"><b>[SLIDES]</b></a><a href="https://scholarsarchive.byu.edu/facpub/2116/"><b>[PDF]</b></a><br><br>


# Authorship
  * Main developer    : Eduardo J Alvarez
  * Email             : Edo.AlvarezR@gmail.com
  * Website           : [edoalvarez.com](https://www.edoalvarez.com/)
  * Created           : Oct 2019
  * License           : MIT License
