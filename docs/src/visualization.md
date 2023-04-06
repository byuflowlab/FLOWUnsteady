# Visualization Guide

## What NOT to do
Unfortunately, the first thing that ParaView shows when you open a particle
field is a "scatter plot" full of confusing colors like this:

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443-low-rainbow00-bad00.jpg"
    alt="Pic here" style="width: 75%;"/>
</center>
```

Not surprisingly, the VPM literature is full of these visualizations, which
provide very little information about the simulation.
[Poor colormap aside](https://www.nature.com/articles/s41467-020-19160-7),
why is this a bad visualization? Even though we are coloring the strengths of
the particles, one must remember that the particle strength
``\boldsymbol\Gamma`` is the integral of the vorticity over the volume
that is discretized by the particle,
``\boldsymbol\Gamma = \int\limits_{\mathrm{Vol}} \boldsymbol\omega \mathrm{d}V``.
This is equivalent to the average vorticity times the volume,
``\boldsymbol\Gamma = \overline{\boldsymbol\omega} \mathrm{Vol}``, which means
that the particle strength is proportional to the volume that it discretizes.
Thus, the particle strength has no physical significance, but is purely a numerical
artifact. *(In fact, $\boldsymbol\Gamma$ are nothing more than the coefficients of the
radial basis function that reconstructs the vorticity field as
``\overline{\boldsymbol\omega}\left( \mathbf{x}\right) \approx \sum\limits_p \boldsymbol{\Gamma}_p \zeta_{\sigma_p}(\mathbf{x}-\mathbf{x}_p)``.)*

Another VPM visualization that is commonly found in the literature simply shows
the position of the particles:
```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443-low-dots00-bad00.jpg"
    alt="Pic here" style="width: 75%;"/>
</center>
```
Why is this a bad visualization? Even though its intent is to depict the
flow field by taking advantage of the fact that particles follow streamlines and vortical
structures, what
is actually visualized is the spatial discretization.
After all, we are just looking at the position of the computational elements.
Here is an equivalent visualization of the computational elements in a
mesh-based simulation of the same rotor in hover:
```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/visualization-mesh00.png"
    alt="Pic here" style="width: 40%;"/>
</center>
```
Not very insightful, is it?

## What we recommend
In order to correctly visualize the flow field that is predicted by the VPM
solver, one must reconstruct the vorticity field using the particles as a
radial basis function and compute the velocity field using the Biot-Savart law.
This can be computationally intensive and is not a streamlined process.

Alternatively, a quick visualization that sheds insights into both the flow field
and the discretization is to make glyphs that point in the direction
of ``\boldsymbol\Gamma`` (arrows scaled by magnitude), thus forming the vortex lines:

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443-low-glyph02.jpg"
    alt="Pic here" style="width: 100%;"/>
</center>
```

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443-low-glyph01.jpg"
    alt="Pic here" style="width: 75%;"/>
</center>
```

This type of visualization has four advantages:
* It clearly **reveals the vortical structure** (vortex lines) in a quick glimpse
* One can **envision the velocity field** around the vortex lines using the right
  hand rule (Biot-Savart law shown below)
* Adding small points at the position of each particle gives us an idea of
  the **spatial resolution around vortices**
* One can easily spot if the simulation is becoming numerically unstable since
  **blown-up particles become giant arrows**

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/visualization-particle01.png"
    alt="Pic here" style="width: 80%;"/>
</center>
```

!!! info "Glyph Visualization"
    A `.pvsm` file with a glyph visualization of the particle field is available here:
    [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/singlerotor-monitors-particles11.pvsm)
    (`right click → save as...`).

    To open in ParaView: `File → Load State → (select .pvsm file)` then
    select "Search files under specified directory" and point it to the
    folder where the simulation was saved.

## Fluid domain

While the "glyph" visualization shown above uses the native variables that are
computed and outputted by the VPM (``\boldsymbol\Gamma_p`` and
``\mathbf{x}_p``), the full fluid domain can be computed from the particle
field in postprocessing.
This is done using the particles as
[a radial basis function](@ref particlediscretization) to construct a continuous
and analytical vorticity field ``\boldsymbol\omega(\mathbf{x})``, and the
Biot-Savart law is used to obtain a continuous and analytical velocity field
``\mathbf{u}(\mathbf{x})``.
Since ``\boldsymbol\omega(\mathbf{x})`` and ``\mathbf{u}(\mathbf{x})`` are
analytical functions, their values and their derivatives can be calculated
anywhere in space.

FLOWUnsteady provides the function
[`uns.computefluiddomain`](@ref) that reads a simulation and processes it to
generate its fluid domain.
See the [Rotor in Hover tutorial](@ref rotorfdom) for an example on how to use
[`uns.computefluiddomain`](@ref).


Whenever reporting results, we recommend showing the vortex elements
(glyph visualization) and the fluid domain side by side to give a clear idea of
the spatial resolution and the resulting flow field (see example below), which
is custom in conventional mesh-based CFD.

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/visualization-highfidelity00.png"
    alt="Pic here" style="width: 100%;"/>
</center>
```
```@raw html
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/visualization-meshbased00.png"
      alt="Pic here" style="width: 85%;"/>
```

```@raw html
<br><br>
```

Here is an example of a simulation superimposing the particle field and
the resulting fluid domain:
```@raw html
<div style="position:relative;padding-top:50%;">
    <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
        src="https://www.youtube.com/embed/Bf4pIt7oi5k?hd=1"
        title="YouTube video player" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen></iframe>
</div>
```

!!! info "Volume Rendering"
      A `.pvsm` file with a volume rendering of the vorticity field is
      available here:
      [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443-fdom03.pvsm)
      (`right click → save as...`).

      To open in ParaView: `File → Load State → (select .pvsm file)` then
      select "Search files under specified directory" and point it to the
      folder where the simulation was saved.


!!! tip "Practice Dataset and Template"
      To help you practice in ParaView, we have uploaded the particle field
      and fluid domain of [the DJI 9443 rotor case](@ref rotorhoveraero) of
      mid-high fidelity and high fidelity simulations:
      * High fidelity: [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/rotorhover-highfidelity-particlefdom.zip)
      * Mid-high fidelity: [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/rotorhover-midhighfidelity-particlefdom.zip)

      We have also added a `.pvsm` ParaView state file that you can use as a
      template for visualizing your particle field with glyphs and processing the
      fluid domain.
      To open in ParaView: `File → Load State → (select .pvsm file)` then
      select "Search files under specified directory" and point it to the
      folder with your simulation results.
