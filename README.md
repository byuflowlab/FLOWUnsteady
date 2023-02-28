![FLOWUnsteady logo](http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/flowunsteady-logo-wide03.png)

[![](https://img.shields.io/badge/docs-stable-blue.svg)](https://flow.byu.edu/FLOWUnsteady/)

TODO
* [ ] Rewrite description
* [ ] Add a validations section compiling all validation studies
* [ ] List of publications
* [ ] Installation instructions
* [ ] Brief theory
* [ ] Visualization Guidelines
* [ ] Numerical recommendations notebook
* [ ] Examples
  * [x] Simple wing
  * [ ] Circular path?
  * [ ] Heaving wing
  * [ ] Single rotor aero
  * [ ] Single rotor noise
  * [ ] Blown wing
  * [ ] Vahana
* [ ] API
  * [ ] Add pics of each monitor
  * [x] Docstring for run_simulation
  * [x] Polish docstrings
    * [x] Vehicle: Rotor
    * [x] Vehicle: SimpleWing and ComplexWing
    * [x] Add database definition
    * [x] Postprocessing: noise functions
    * [x] Postprocessing: Fluid domain
* [x] Move large figure files to J Drive

<img src="docs/resources/images/blownwing00.png" alt="Pic here" style="width: 900px;"/>

If you like this package please give it a star. We like stars =]

Simulation engine of mixed-fidelity unsteady aerodynamics and aeroacoustics.
This suite brings together mid and high-fidelity
aerodynamics tools developed at BYU's [FLOW Lab](http://flow.byu.edu/): [`GeometricTools`](https://github.com/byuflowlab/GeometricTools.jl)
(geometric engine), [`FLOWVLM`](https://github.com/byuflowlab/FLOWVLM) (VLM and
strip theory solver), [`CCBlade`](https://github.com/byuflowlab/CCBlade.jl)
(blade element momentum solver),
[`MyPanel`](https://github.com/EdoAlvarezR/MyPanel.jl) (3D inviscid panel
solver), and `FLOWVPM` (viscous vortex particle method). The aeroacoustic
solver integrates PSU-WOPWOP (FW-H solver), [FLOW's BPM](https://github.com/byuflowlab/BPM.jl), and [`FLOWNoise`](https://github.com/byuflowlab/FLOWNoise), obtaining both tonal and broadband acoustic noise.

This package is written for Julia 1.4.2 or higher.

Official documentation: [https://flow.byu.edu/FLOWUnsteady/](https://flow.byu.edu/FLOWUnsteady/)

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
documentation: [`docs/resources/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/validation.ipynb).

For example simulations, check this notebook: [`docs/resources/examples.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/examples.ipynb).

# Folders and Files
  * `src/`: Source code.
  * `examples/`: Example simulations.
  * `docs/resources/`: Documentation (open Jupyter notebooks with [nbviewer](https://nbviewer.jupyter.org/)).
    - `docs/resources/instructions-setup.md`: Instructions for setting up this package.
    - [`docs/resources/validation.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/validation.ipynb): Validation of models implemented in this package and numeric observations.
    - [`docs/resources/examples.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/docs/resources/examples.ipynb): Results of example simulations.
  * [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb): Rotor aeroacoustic noise validation and example.

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

# Sponsors
<img src="docs/resources/images/sponsors00.png" alt="Pic here" style="width: 900px;"/>

# Examples
**HEAVING WING:** `examples/heavingwing.jl`

<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/bertinsheaving00.gif" alt="Vid here" style="width: 400px;"/>


**CROSS-WIND CIRCULAR PATH:** `examples/circularpath.jl`
[<img src="docs/resources/images/circlesim_wide.jpg" alt="Vid here" style="width: 900px;"/>](http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/circularpath03_1.gif)


**HOVERING ROTOR:** `examples/singlerotor.jl`

<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/fvs_singlerotor02.gif" alt="Vid here" style="width: 400px;"/>

**INTERACTING TANDEM HEAVING WING:** `examples/tandemheavingwing.jl`
[![Vid here](docs/resources/images/play01_wide.png)](https://youtu.be/Pch94bKpjrQ)

**BLOWN WING:** `examples/blownwing/blownwing.jl`
[![Vid here](docs/resources/images/blownwingplay03.png)](https://youtu.be/3REcIdIXrZA)

**Wind-harvesting Aircraft:** `examples/windcraft/windcraft.jl` (in progress)
[![Vid here](docs/resources/images/windcraftwake.jpg)](https://youtu.be/iFM3B4_N2Ls)

**eVTOL TRANSITION:** `examples/vahana2/vahana.jl`, [VIDEO](https://youtu.be/d__wNtRIBY8)

<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/vorticitytake01-smallreducedslower02-2.gif" alt="Vid here" style="width: 400px;"/>
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/acousticstake01-smallreducedslower03.gif" alt="Vid here" style="width: 400px;"/>

**Rotor Aeroacoustic Noise:** [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb)

<img src="docs/resources/images/rotornoise01.png" alt="Pic here" style="width: 400px;"/>

<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/cfdnoise_ningdji_multi_005D_03_15.gif" alt="Vid here" style="width: 400px;"/>

# Framework Flowchart
<img src="docs/resources/images/flowchart00.png" alt="Pic here" style="width: 900px;"/>

# Publications
  * Alvarez, E. J., & Ning, A. (2021, in progress). <i>Unsteady Mixed-fidelity Aerodynamics Solver
for Maneuvering Multirotor Aircraft</i>. AIAA SciTech Forum. <a href="https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/AlvarezNing_2021-SciTechAbstract-FLOWUnsteady_solver.pdf"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A. (2020, in review). <i>Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover</i>. Journal of the American Helicopter Society. <a href="http://edoalvar2.groups.et.byu.net/public/AlvarezSchenkCritchfield_2020-PresentationVFSForum-multirotor_noise_interactions_in_hoverSTATIC.pdf"><b>[SLIDES]</b></a><a href="https://scholarsarchive.byu.edu/facpub/4053/"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., (2020). <i>Quasi-steady Aerodynamics Solver for a High-fidelity Controls Framework</i>. FLOWUnsteady Documentation. <a href="https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/quasisteadysolver.pdf"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2020). <i>High-fidelity Modeling of Multirotor Aerodynamic Interactions for Aircraft Design</i>. AIAA Journal. DOI: <a href="https://arc.aiaa.org/doi/10.2514/1.J059178">10.2514/1.J059178</a> <a href="https://scholarsarchive.byu.edu/facpub/4179/"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2019). <i>Modeling Multirotor Aerodynamic Interactions Through the Vortex Particle Method</i>. AIAA AVIATION Forum. DOI: <a href="https://doi.org/10.2514/6.2019-2827 ">10.2514/6.2019-2827</a> <a href="http://edoalvar2.groups.et.byu.net/public/AlvarezNing_2019-AVIATION-Multirotor_aerodynamic_interactions_through_VPM-STATIC.pdf"><b>[SLIDES]</b></a><a href="https://scholarsarchive.byu.edu/facpub/3191/"><b>[PDF]</b></a><br><br>
  * Alvarez, E. J., & Ning, A. (2018). <i>Development of a Vortex Particle Code for the Modeling of Wake Interaction in Distributed Propulsion</i>. AIAA AVIATION Forum. DOI: <a href="https://doi.org/10.2514/6.2018-3646 ">10.2514/6.2018-3646</a> <a href="http://www.et.byu.edu/~edoalvar/public/AlvarezNing_2018-AIAA-VPM_distibuted_propulsion-SLIDE-static.pdf"><b>[SLIDES]</b></a><a href="https://scholarsarchive.byu.edu/facpub/2116/"><b>[PDF]</b></a><br><br>


# Authorship
  * Main developer    : Eduardo J Alvarez
  * Email             : Edo.AlvarezR@gmail.com
  * Website           : [edoalvarez.com](https://www.edoalvarez.com/)
  * Created           : Oct 2019
  * License           : MIT License
