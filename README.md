<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/flowunsteady-logo-wide03.png" alt="FLOWUnsteady logo" style="width:100%">

<p align="right">
  <span style="color:#2f6990;">
    <i>Interactional aerodynamics solver for multirotor aircraft and wind energy</i>
  </span>
</p>

<p align="right">
  <a href="https://github.com/byuflowlab/FLOWUnsteady">
    <img src="https://img.shields.io/badge/code-open%20source-brightgreen.svg">
  </a>
  <a href="https://flow.byu.edu/FLOWUnsteady/">
    <img src="https://img.shields.io/badge/docs-stable-blue.svg">
  </a>
</p>

---

FLOWUnsteady is an open-source variable-fidelity framework for unsteady
aerodynamics and aeroacoustics based on the
[reformulated vortex particle method](https://scholarsarchive.byu.edu/etd/9589/)
(rVPM).
This suite brings together various tools developed by the
[FLOW Lab](http://flow.byu.edu/) at Brigham Young University: Vortex lattice
method, strip theory, blade elements, 3D panel method, and rVPM.
The suite also integrates an FW-H solver and a BPM code for tonal
and broadband prediction of aeroacoustic noise.
In the low end of fidelity, simulations are similar to a free-wake method,
while in the high end simulations become meshless large eddy simulations.


* *Documentation:* [flow.byu.edu/FLOWUnsteady](https://flow.byu.edu/FLOWUnsteady)
* *Code:* [github.com/byuflowlab/FLOWUnsteady](https://github.com/byuflowlab/FLOWUnsteady)

### What is the Reformulated VPM?

The [reformulated VPM](https://scholarsarchive.byu.edu/etd/9589/) is a meshless
CFD method solving the LES-filtered incompressible Navier-Stokes equations in
their vorticity form,
<p align="center">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/vorticityns.png" alt="img" style="width:40%">
</p>
It uses a Lagrangian (meshless) scheme, which not only
avoids the hurdles of mesh generation, but it also conserves vortical structures
over long distances with minimal numerical dissipation.

The rVPM uses particles to discretize the Navier-Stokes equations, with the
particles representing radial basis functions that construct a continuous
vorticity/velocity field. The basis functions become the LES filter, providing a
variable filter width and spatial adaptation as the particles are convected and
stretched by the velocity field. The local evolution of the filter width
provides an extra degree of freedom to reinforce conservation laws, which makes
the reformulated VPM numerically stable (overcoming the numerical issues that
plague the classic VPM).

This meshless LES has several advantages over conventional mesh-based CFD.
In the absence of a mesh,   
1. the rVPM does not suffer from the numerical dissipation introduced by a mesh
2. integrates over coarser discretizations without losing physical accuracy
3. derivatives are calculated analytically rather than approximated through a stencil.

Furthermore, rVPM is highly efficient since it uses computational elements only
where there is vorticity (rather than meshing the entire space), usually being ~100x
faster than conventional mesh-based LES with comparable accuracy.


While rVPM is well suited for resolving unbounded flows (wakes), complications
arise when attempting to impose boundary conditions (solid boundaries) on the flow.
This is because (1) the method is meshless, and (2) boundary conditions must
be imposed on the Navier-Stokes equations in the form of vorticity.
FLOWUnsteady is a framework designed to introduce solid boundaries into the rVPM
using actuator models.
Wings and rotors are introduced in the computational domain through actuator
line and surface models that use low-fidelity aerodynamic methods
(*e.g.*, VLM, lifting line,
panels, etc) to compute forces and embed the associated
vorticity back into the LES domain.


<p><br></p>


<p align="center"> <a href="https://www.youtube.com/watch?v=-6aR37Z6hig&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-vahanahi.png" alt="youtube.com/watch?v=-6aR37Z6hig" style="width:70%"> </a> </p>


### Variable Fidelity

rVPM considerably reduces engineering time by avoiding the hurdles of mesh
generation. Furthermore, since it is not limited by the time-step and stability
constraints of conventional mesh-based CFD, rVPM can be used across all levels
of fidelity, all in the same framework by simply coarsening or refining the
simulation.
In the low end of fidelity, simulations are similar to a free-wake method,
while in the high end simulations become meshless large eddy simulations.
Thus, FLOWUnsteady can be used as a high-fidelity tool that is orders of
magnitude faster than mesh-based CFD, or as a variable-fidelity tool for
the different stages of design.

<p align="left">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/flowunsteady-variablefidelity.jpg" alt="img" style="width:100%">
</p>

### Capabilities

  > **Simulation:**
  > *Tilting wings and rotors*
  > *• Rotors with variable RPM and variable pitch*
  > *• Asymmetric and stacked rotors*
  > *• Maneuvering vehicle with prescribed kinematics*
  >
  > **rVPM Solver:**
  > *Fast-multipole acceleration through [ExaFMM](https://joss.theoj.org/papers/10.21105/joss.03145)*
  > *• CPU parallelization through OpenMP*
  > *• Second-order spatial accuracy and third-order RK time integration*
  > *• Numerically stable by reshaping particles subject to vortex stretching*
  > *• Subfilter-scale (SFS) model of turbulence associated to vortex stretching*
  > *• SFS model coefficient computed dynamically or prescribed*
  > *• Viscous diffusion through core spreading*
  >
  > **Wing Models:**
  > *Actuator line model through lifting line + VLM*
  > *• Actuator surface model through vortex sheet + VLM*
  > *• Parasitic drag through airfoil lookup tables*
  >
  > **Rotor Model:**
  > *Actuator line model through blade elements*
  > *• Airfoil lookup tables automatically generated through XFOIL*
  > *• Aeroacoustic noise through FW-H (PSU-WOPWOP) and BPM*
  >
  > **Geometry:**
  > *Simple lofts and bodies of revolution through FLOWUnsteady*
  > *• Import complex geometry from [OpenVSP](https://openvsp.org/)*
  >
  > **Under development *(*🤞*coming soon)*:**
  > *Advanced actuator surface models through 3D panel method (for ducts, wings, and fuselage)*
  > *• Unstructured surface grids*
  > *• Bluff bodies through vortex sheet method*
  >
  > **Limitations:**
  > *Viscous drag and separation is only captured through airfoil lookup tables, without attempting to shed separation wakes*
  > *• Incompressible flow only (though wave drag can be captured through airfoil lookup tables)*
  > *• CPU parallelization through OpenMP without support for distributed memory (no MPI, i.e., only single-node runs)*
  >
  > *Coded in [the Julia language](https://www.infoworld.com/article/3284380/what-is-julia-a-fresh-approach-to-numerical-computing.html) for Linux, MacOS, and Windows WSL.*





More about the models inside FLOWUnsteady:
<p align="center">
  <a href="https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/nasaamsseminar2.png" alt="https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html" style="width:70%">
  </a>
</p>

<p><br></p>

### Selected Publications
See the following publications for an in-depth dive into the theory and validation:

* E. J. Alvarez, J. Mehr, & A. Ning (2022), "FLOWUnsteady: An Interactional Aerodynamics Solver for Multirotor Aircraft and Wind Energy," *AIAA AVIATION Forum*. [**[VIDEO]**](https://youtu.be/SFW2X8Lbsdw) [**[PDF]**](https://scholarsarchive.byu.edu/facpub/5830/)
* E. J. Alvarez (2022), "Reformulated Vortex Particle Method and Meshless Large Eddy Simulation of Multirotor Aircraft.," *Doctoral Dissertation, Brigham Young University*. [**[VIDEO]**](https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html) [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)
* E. J. Alvarez & A. Ning (2023), "Stable Vortex Particle Method Formulation for Meshless Large-Eddy Simulation," *AIAA Journal*. [**[PDF]**](https://arc.aiaa.org/doi/epdf/10.2514/1.J063045)

<p><br></p>

### Examples and Tutorials

**Propeller:** [[Tutorial](https://flow.byu.edu/FLOWUnsteady/examples/propeller-J040)] [[Validation](https://flow.byu.edu/FLOWUnsteady/theory/validation/#Propeller)]

<p align="center"> <a href="https://www.youtube.com/watch?v=lUIytQybCpQ&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-prop.png" alt="youtube.com/watch?v=lUIytQybCpQ" style="width:70%"> </a> </p>


**Rotor in Hover:** [[Tutorial](https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-aero)] [[Validation](https://flow.byu.edu/FLOWUnsteady/theory/validation/#Rotor)]

<p align="center"> <a href="https://www.youtube.com/watch?v=u9SgYbYhPpU&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-hover.png" alt="youtube.com/watch?v=u9SgYbYhPpU" style="width:70%"> </a> </p>


**Blown Wing:** [[Tutorial](https://flow.byu.edu/FLOWUnsteady/examples/blownwing-aero)] [[Validation](https://flow.byu.edu/FLOWUnsteady/theory/validation/#Rotor-Wing-Interactions)]

<p align="center"> <a href="https://www.youtube.com/watch?v=GfS3NoVrFfU&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-rotorwing.png" alt="youtube.com/watch?v=GfS3NoVrFfU" style="width:70%"> </a> </p>


**Ducted Fan:** [[Slides](http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/alvarez_2023-SLIDES-VPM_for_EDF_in_Non_Axisymmetric_Flow.pdf)]

<p align="center"> <a href="https://www.youtube.com/watch?v=BQpar3A0X-w&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-edf.png" alt="youtube.com/watch?v=BQpar3A0X-w" style="width:70%"> </a> </p>


**Airborne-Wind-Energy Aircraft:** [[Video](https://www.youtube.com/watch?v=iFM3B4_N2Ls)]

<p align="left">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/circular-fdom-top02.jpg" alt="img" style="width:75%">
</p>


**eVTOL Transition:** [[Tutorial](https://flow.byu.edu/FLOWUnsteady/examples/vahana-vehicle/)]

Mid-fidelity
<p align="center"> <a href="https://www.youtube.com/watch?v=d__wNtRIBY8&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-vahanamid.png" alt="youtube.com/watch?v=d__wNtRIBY8" style="width:70%"> </a> </p>

High-fidelity
<p align="center"> <a href="https://www.youtube.com/watch?v=-6aR37Z6hig&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-vahanahi.png" alt="youtube.com/watch?v=-6aR37Z6hig" style="width:70%"> </a> </p>


**Aeroacoustic Noise:** [[Tutorial](https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-acoustics)] [[Validation](https://flow.byu.edu/FLOWUnsteady/theory/validation/#Rotor)]

<p align="center"> <a href="https://www.youtube.com/watch?v=ntQjP6KbZDk&hd=1"> <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/youtube-vahananoise.png" alt="youtube.com/watch?v=ntQjP6KbZDk" style="width:70%"> </a> </p>

<p align="center">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cfdnoise_ningdji_multi_005D_03_20.gif" alt="Vid" style="width:60%"/>
</p>





### Sponsors

<p align="center">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/sponsors01.png" alt="img" style="width:100%">
  <br><br><br>
</p>



### About

FLOWUnsteady is an open-source project jointly led by the
[FLOW Lab](http://flow.byu.edu/) at Brigham Young University and
[Whisper Aero](http://whisper.aero/).
All contributions are welcome.

If you find FLOWUnsteady useful in your work, we kindly request that you cite the following paper [[URL]](https://arc.aiaa.org/doi/10.2514/6.2022-3218) [[PDF]](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=6735&context=facpub):

>Alvarez, E. J., Mehr, J., and Ning, A., “FLOWUnsteady: An Interactional Aerodynamics Solver for Multirotor Aircraft and Wind Energy,” AIAA AVIATION 2022 Forum, Chicago, IL, 2022. DOI:[10.2514/6.2022-3218](https://doi.org/10.2514/6.2022-3218).

If you were to encounter any issues or have questions, please first read through
[the documentation](https://flow.byu.edu/FLOWUnsteady/), [open/closed
issues](https://github.com/byuflowlab/FLOWUnsteady/issues?q=is%3Aissue+is%3Aclosed),
and [the discussion forum](https://github.com/byuflowlab/FLOWUnsteady/discussions?discussions_q=).
If the issue still persists, please participate in
[the discussion forum](https://github.com/byuflowlab/FLOWUnsteady/discussions?discussions_q=)
and/or [open a new issue](https://github.com/byuflowlab/FLOWUnsteady/issues).

  * Developers/contributors : [Eduardo J. Alvarez](https://www.edoalvarez.com/) (main), [Cibin Joseph](https://github.com/cibinjoseph), [Judd Mehr](https://www.juddmehr.com/), [Ryan Anderson](https://flow.byu.edu/people/), [Eric Green](https://flow.byu.edu/people/)
  * Created           : Sep 2017
  * License           : MIT License
