```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/flowunsteady-logo-wide03.png" alt="FLOWUnsteady logo" style="width:100%">
```

```@raw html
<p align="right">
  <span style="color:#2f6990;">
    <i>Interactional aerodynamics solver for multirotor aircraft and wind energy</i>
  </span>
</p>
```

```@raw html
<p align="right">
  <a href="https://github.com/byuflowlab/FLOWUnsteady">
    <img src="https://img.shields.io/badge/code-open%20source-brightgreen.svg">
  </a>
  <a href="https://flow.byu.edu/FLOWUnsteady/">
    <img src="https://img.shields.io/badge/docs-stable-blue.svg">
  </a>
</p>
```

---

FLOWUnsteady is a variable-fidelity framework for unsteady aerodynamics and
aeroacoustics based on the
[reformulated vortex particle method](https://scholarsarchive.byu.edu/etd/9589/)
(rVPM).
This suite brings together various tools developed by the
[FLOW Lab](http://flow.byu.edu/) at Brigham Young University: Vortex lattice
method, strip theory, blade-element momentum, 3D panel method, and rVPM.
The suite also integrates an FW-H solver and a BPM code for tonal
and broadband prediction of aeroacoustic noise.


* *Documentation:* [flow.byu.edu/FLOWUnsteady](https://flow.byu.edu/FLOWUnsteady)
* *Code:* [github.com/byuflowlab/FLOWUnsteady](https://github.com/byuflowlab/FLOWUnsteady)

### What is the Reformulated VPM?

The [reformulated VPM](https://scholarsarchive.byu.edu/etd/9589/) is a meshless
CFD method solving the LES-filtered incompressible Navier-Stokes equations in
their vorticity form,
```@raw html
<p align="center">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/vorticityns.png" alt="img" style="width:40%">
</p>
```
It uses a Lagrangian (meshless) scheme, which not only
avoids the hurdles of mesh generation, but it also conserves vortical structures
over long distances with minimal numerical dissipation.

The rVPM uses particles to discretize the Navier-Stokes equations, with the
particles representing radial basis functions that construct a continuous
vorticity/velocity field. The basis functions become the LES filter, providing a
variable filter width and spatial adaption as the particles are convected and
stretched by the velocity field. The local evolution of the filter width
provides an extra degree of freedom to reinforce conservations laws, which makes
the reformulated VPM numerically stable (overcoming the numerical issues that
plague the classic VPM).

This meshless LES has several advantages over conventional mesh-based CFD.
In the absence of a mesh,   
1. the rVPM does not suffer from the numerical dissipation introduced by a mesh
2. integrates over coarser discretizations without losing physical accuracy
3. derivatives are calculated analytically rather than approximated through a stencil

Furthermore, rVPM is highly efficient since it uses computational elements only
where there is vorticity (rather than meshing the entire space), making it 100x
faster than conventional mesh-based LES with comparable accuracy.


While rVPM is well suited for resolving unbounded flows (wakes), it is
difficult to introduce solid boundaries in its computational domain.
This is because (1) the method is meshless, and (2) boundary conditions must
be imposed in the Navier-Stokes equations in the form of vorticity.
FLOWUnsteady is a framework designed to introduce solid boundaries into the rVPM
using actuator models.
Wings and rotors are introduced in the computational domain through actuator
line and surface models that use low-fidelity aerodynamic methods
(*e.g.*, VLM, lifting line,
panels, etc) to compute forces and embed the associated
vorticity back into the LES domain.


```@raw html
<p><br></p>
```

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/-6aR37Z6hig?hd=1"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

### Variable Fidelity for Preliminary-to-Detailed Design

rVPM considerably reduces engineering time by avoiding the hurdles of mesh
generation. Furthermore, since it is not limited by the time-step and stability
constraints of conventional mesh-based CFD, rVPM can be used across all levels
of fidelity, all in the same framework by simply coarsening or refining the
simulation. Thus, FLOWUnsteady can be used as a high-fidelity tool that is orders of
magnitude faster than mesh-based CFD, while also providing designers with a
true variable-fidelity tool for the different stages of design.

```@raw html
<p align="left">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/flowunsteady-variablefidelity.jpg" alt="img" style="width:100%">
</p>
```

### Capabilities

  > **Simulation:**
  > *Tilting wings and rotors*
  > *• Rotors with variable RPM and variable pitch*
  > *• Maneuvering vehicle with prescribed kinematics*
  >
  > **rVPM Solver:**
  > *Fast-multipole acceleration through [ExaFMM](https://joss.theoj.org/papers/10.21105/joss.03145)*
  > *• CPU parallelization through OpenMPI*
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
  > **Under development *(*🤞*coming soon)*:**
  > *Advanced actuator surface models through 3D panel method (for ducts, wings, and fuselage)*
  > *• Bluff bodies through vortex sheet method*
  >
  > **Limitations:**
  > *Viscous drag and separation is only captured through airfoil lookup tables, without attempting to shed separation wakes*
  > *• Incompressible flow only (though wave drag can be captured through airfoil lookup tables)*
  > *• CPU parallelization through OpenMPI without support for distributed memory (no MPI, i.e., only single-node* runs)
  > *• Limited support for Windows*





More about the models inside FLOWUnsteady:
```@raw html
<p align="center">
  <a href="https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html">
    <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/nasaamsseminar2.png" alt="https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html" style="width:70%">
  </a>
</p>
```

```@raw html
<p><br></p>
```

### Selected Publications
See the following publications for an in-depth dive into the theory and validation:

* E. J. Alvarez, J. Mehr, & A. Ning (2022). "FLOWUnsteady: An Interactional Aerodynamics Solver for Multirotor Aircraft and Wind Energy." *AIAA AVIATION Forum*. [**[VIDEO]**](https://youtu.be/SFW2X8Lbsdw) [**[PDF]**](https://scholarsarchive.byu.edu/facpub/5830/)
* E. J. Alvarez & A. Ning (2022). "Reviving the Vortex Particle Method: A Stable Formulation for Meshless Large Eddy Simulation." *(in review)* [**[PDF]**](https://arxiv.org/pdf/2206.03658.pdf)
* E. J. Alvarez (2022). "Reformulated Vortex Particle Method and Meshless Large Eddy Simulation of Multirotor Aircraft". *Doctoral Dissertation, Brigham Young University*. [**[VIDEO]**](https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html) [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)

```@raw html
<p><br></p>
```

### Examples

**Propeller:** [Tutorial] [Validation] [[Video](https://www.youtube.com/watch?v=lUIytQybCpQ)]

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/lUIytQybCpQ?hd=1&rel=0"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

**Rotor in Hover:** [Tutorial] [Validation] [[Video](https://www.youtube.com/watch?v=u9SgYbYhPpU)]

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/u9SgYbYhPpU?hd=1&rel=0"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

**Blown Wing:** [Tutorial] [Validation]

```@raw html
<p align="center">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/prowimhtp-wvol34-cropped00.jpg" alt="img" style="width:100%">
</p>
```

```@raw html
<p><br></p>
```

**Airborne-Wind-Energy Aircraft:** [Validation] [[Video](https://www.youtube.com/watch?v=iFM3B4_N2Ls)]

```@raw html
<p align="left">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/circular-fdom-top02.jpg" alt="img" style="width:75%">
</p>
```


**eVTOL Transition:** [Tutorial] [[Video 1](https://www.youtube.com/watch?v=-6aR37Z6hig)] [[Video 2](https://www.youtube.com/watch?v=d__wNtRIBY8)]

Mid-fidelity
```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/d__wNtRIBY8?hd=1&rel=0"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```
High-fidelity
```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/-6aR37Z6hig?hd=1&rel=0"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```


**Aeroacoustic Noise:** [Tutorial] [Validation]

```@raw html
<p align="left">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cfdnoise_ningdji_multi_005D_03_20.gif" alt="Vid" width="60%"/>
</p>
```

```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
        src="https://www.youtube.com/embed/ntQjP6KbZDk?hd=1&rel=0"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```




### Sponsors

```@raw html
<p align="center">
  <img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/sponsors01.png" alt="img" style="width:100%">
  <br><br><br>
</p>
```



### About

FLOWUnsteady is an open-source project jointly led by the
[FLOW Lab](http://flow.byu.edu/) at Brigham Young University and
[Whisper Aero](http://whisper.aero/).
All contributions are welcome.

If you find FLOWUnsteady useful in your work, we kindly request that you cite the following paper [[URL]](https://arc.aiaa.org/doi/10.2514/6.2022-3218) [[PDF]](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=6735&context=facpub):

>Alvarez, E. J., Mehr, J., and Ning, A., “FLOWUnsteady: An Interactional Aerodynamics Solver for Multirotor Aircraft and Wind Energy,” AIAA AVIATION 2022 Forum, Chicago, IL, 2022. DOI:[10.2514/6.2022-3218](https://doi.org/10.2514/6.2022-3218).

If you were to encounter any issues, please first read through
[the documentation](https://flow.byu.edu/FLOWUnsteady/) and [open/closed
issues](https://github.com/byuflowlab/FLOWUnsteady/issues?q=is%3Aissue+is%3Aclosed).
If the issue still persists, please
[open a new issue](https://github.com/byuflowlab/FLOWUnsteady/issues).

  * Main developer    : Eduardo J. Alvarez ([edoalvarez.com](https://www.edoalvarez.com/))
  * Created           : Oct 2019
  * License           : MIT License






  TODO
  * [ ] README
    * [x] Rewrite description
    * [ ] Add links to examples in README
  * [ ] Theory
    * [ ] Add a validations section compiling all validation studies
    * [ ] List of publications
    * [ ] Brief theory
  * [x] Installation instructions
  * [ ] Visualization Guidelines
  * [ ] Numerical recommendations notebook
  * [ ] Examples
    * [x] Simple wing
    * [x] Circular path?
    * [x] Heaving wing
    * [x] Single rotor aero
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
  * [x] Citing guide

```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/light/vorticitytake01-smallreducedslower02-2.gif" alt="Vid here" style="width: 400px;"/>
```
