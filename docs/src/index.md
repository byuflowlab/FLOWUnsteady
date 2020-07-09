# Welcome to FLOWUnsteady

```@raw html
<img src="assets/img/blownwing00.png" alt="Vid" width="700px"/>
```

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

SOURCE CODE: [https://github.com/byuflowlab/FLOWUnsteady](https://github.com/byuflowlab/FLOWUnsteady)

!!! note "Vortex particle method (VPM) solver"
    All aerodynamics codes used in FLOWUnsteady are opensource and available in Github, except for the VPM solver. Without the VPM, the user can still use all quasi-steady aerodynamic and aeroacoustic solvers in FLOWUnsteady. To get full access to the unsteady solver, please contact Eduardo Alvarez ([edoalvarez.com](https://edoalvarez.com)) or the FLOW Lab ([flow.byu.edu](http://flow.byu.edu/)).

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
![Vid](assets/vid/bertinsheaving00.gif)

**CROSS-WIND CIRCULAR PATH:** `examples/circularpath.jl`
![Vid](assets/vid/circularpath03_1.gif)

**HOVERING ROTOR:** `examples/singlerotor.jl`
![Vid](assets/vid/fvs_singlerotor02.gif)

**INTERACTING TANDEM HEAVING WING:** `examples/tandemheavingwing.jl`
[![Vid here](assets/img/play01_wide.png)](https://youtu.be/Pch94bKpjrQ)

**BLOWN WING:** `examples/blownwing/blownwing.jl`
[![Vid here](assets/img/blownwingplay03.png)](https://youtu.be/3REcIdIXrZA)


**Wind-harvesting Aircraft:** `examples/windcraft/windcraft.jl` (in progress)
[![Vid here](assets/img/windcraftwake.jpg)](https://youtu.be/iFM3B4_N2Ls)

**eVTOL TRANSITION:** `examples/vahana/vahana.jl` (in progress)
[![Vid here](assets/img/play00_wide.png)](https://youtu.be/f_AkQW37zqs)

**Rotor Aeroacoustic Noise:** [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb)

```@raw html
<img src="assets/img/rotornoise01.png" alt="Vid" width="400px"/>
```
```@raw html
<img src="assets/vid/cfdnoise_ningdji_multi_005D_03_15.gif" alt="Vid" width="600px"/>
```
```@raw html
<img src="assets/vid/cfdnoise_ningdji_multi_005D_03_18.gif" alt="Vid" width="600px"/>
```

# Framework Flowchart
![Img](assets/img/flowunstrady_diagram03.png)

# Publications
  * Alvarez, E. J., & Ning, A. (2021, in progress). *Unsteady Mixed-fidelity Aerodynamics Solver for Maneuvering Multirotor Aircraft*. AIAA SciTech Forum. [\[PDF\]](https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/AlvarezNing_2021-SciTechAbstract-FLOWUnsteady_solver.pdf)
  * Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A. (2020, in review). *Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover*. Journal of the American Helicopter Society.
  * Alvarez, E. J., (2020). *Quasi-steady Aerodynamics Solver for a High-fidelity Controls Framework*. FLOWUnsteady Documentation. [\[PDF\]](https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/quasisteadysolver.pdf)
  * Alvarez, E. J., & Ning, A. (2020, accepted). *High-fidelity Modeling of Multirotor Aerodynamic Interactions for Aircraft Design*. AIAA Journal. [\[PDF\]](https://byu.box.com/shared/static/s4h4w165x4vlxcmhgmxi05ngq0qab9o0.pdf)
  * Alvarez, E. J., & Ning, A. (2019). *Modeling Multirotor Aerodynamic Interactions Through the Vortex Particle Method*. AIAA AVIATION Forum. DOI: [10.2514/6.2019-2827](https://doi.org/10.2514/6.2019-2827)[\[SLIDES\]](http://edoalvar2.groups.et.byu.net/public/AlvarezNing_2019-AVIATION-Multirotor_aerodynamic_interactions_through_VPM-STATIC.pdf)[\[PDF\]](https://scholarsarchive.byu.edu/facpub/3191/)
  * Alvarez, E. J., & Ning, A. (2018). *Development of a Vortex Particle Code for the Modeling of Wake Interaction in Distributed Propulsion*. AIAA AVIATION Forum. DOI: [10.2514/6.2018-3646](https://doi.org/10.2514/6.2018-3646)[\[SLIDES\]](http://www.et.byu.edu/~edoalvar/public/AlvarezNing_2018-AIAA-VPM_distibuted_propulsion-SLIDE-static.pdf)[\[PDF\]](https://scholarsarchive.byu.edu/facpub/2116/)


# Authorship
  * Main developer    : Eduardo J Alvarez
  * Email             : Edo.AlvarezR@gmail.com
  * Website           : [edoalvarez.com](https://www.edoalvarez.com/)
  * Created           : Oct 2019
  * License           : MIT License
