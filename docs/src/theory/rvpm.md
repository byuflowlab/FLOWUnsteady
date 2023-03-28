# Reformulated VPM

The following is an excerpt from  E. J. Alvarez & A. Ning (2022), *"Meshless Large Eddy Simulation of Rotor-Wing Interactions Through the Reformulated Vortex Particle Method,"* (in review).


## Vorticity Navier-Stokes

In recent work[^1][^2], a new formulation of the vortex particle method (VPM) has been derived from the LES-filtered Navier-Stokes equations.
The new method, referred to as the reformulated VPM or rVPM, is an LES that is both numerically stable and meshless, and is able to accurately resolve mean and fluctuating large-scale features of turbulent flow with minimal computational effort.
Hereby we concisely summarize the governing equations of the reformulated VPM, and the reader is referred to Ref.[^1] and the doctoral dissertation[^2] accompanying this work for a detailed derivation of the method.

The reformulated VPM uses a Lagrangian scheme to solve the vorticity form of the LES-filtered Navier-Stokes equations
```math
\begin{align}
        \frac{\partial \overline{ \omega_i }}{\partial t}
        + \overline{ u_j } \frac{\partial \overline{ \omega_i }}{\partial x_j}
    & =
        \overline{ \omega_j } \frac{\partial \overline{ u_i }}{\partial x_j}
    +
        \nu \nabla^2 \overline{ \omega_i } -
        \frac{\partial T'_{ij}}{\partial x_j} +
        \frac{\partial T_{ij}}{\partial x_j}
,\end{align}
```
where the bar denotes the filter operator,[^a] and ``T_{ij} \equiv \overline{ u_i \omega_j } - \overline{ u_i } \, \overline{ \omega_j }`` is the subfilter-scale (SFS) vorticity stress capturing the interactions between large-scale dynamics and SFS dynamics.
The term ``\frac{\partial T'_{ij}}{\partial x_j}`` represents the SFS contributions arising from the advective term (vorticity advection), while ``\frac{\partial T_{ij}}{\partial x_j}`` represents the contributions arising from vortex stretching.
For simplicity, Eq. (1) is written in vector notation as
```math
\begin{align}
    \frac{\text{d} }{\text{d} t} \overline{ \boldsymbol\omega }
    = \left( \overline{ \boldsymbol\omega } \cdot \nabla \right) \overline{ \mathbf{u} } +
    \nu \nabla^2 \overline{ \boldsymbol\omega }
    - \mathbf{E}_\mathrm{adv} - \mathbf{E}_\mathrm{str}
,\end{align}
```
where ``\left( \mathbf{E}_\mathrm{adv} \right)_i \equiv \frac{\partial T'_{ij}}{\partial x_j}`` is the SFS vorticity advection, ``\left( \mathbf{E}_\mathrm{str} \right)_i \equiv - \frac{\partial T_{ij}}{\partial x_j}`` is the SFS vortex stretching, and the ``\frac{\text{d} }{\text{d} t}`` operator is the linearized version of the filtered material derivative, ``\frac{\text{d} }{\text{d} t} () \equiv \frac{\partial }{\partial t}() + {(\overline{ {\bf u} } \cdot \nabla)()}``.
Notice that casting the Navier-Stokes equation into this vorticity form gets rid of all dependance on pressure.
Furthermore, this equation depends on ``\boldsymbol{\omega}`` alone since ``\bf{u}`` can be calculated directly from ``\boldsymbol{\omega} = \nabla \times \bf{u}`` through the Biot-Savart law.


## Particle Discretization

The material derivative in Eq. (2) and the material-conservative nature of the vorticity makes the ``\boldsymbol{\omega}`` field especially well fit for a Lagrangian description.
The unfiltered ``\boldsymbol{\omega}`` field is discretized with singular vortex particles of positions ``{\bf x}_p`` and coefficients ``\boldsymbol{\Gamma}_p`` (called *vortex strength*), approximating ``\boldsymbol{\omega}`` as
```math
\begin{align} % \label{eq:particle:dirac}
    \boldsymbol{\omega}({\bf x},t) \approx \sum
        \limits_p \boldsymbol{\Gamma}_p (t)
            \delta  ({\bf x} - {\bf x}_p(t))
,\end{align}
```
where ``\delta`` is the Dirac delta.
Applying the filter operator,
```math
\begin{align*}
        \overline{ \boldsymbol{\omega} } \left( \mathbf{x} \right)
    & =
        \int\limits_{-\infty}^\infty
            \boldsymbol{\omega}\left( \mathbf{y} \right)
            \zeta_\sigma(\mathbf{x}-\mathbf{y})
        \,\mathrm{d}\mathbf{y}
    % \\ &
    \approx
        \int\limits_{-\infty}^\infty
            \left(
                \sum\limits_p
                    \boldsymbol{\Gamma}_p
                    \delta ({\textbf y} - {\textbf x}_p)
            \right)
            \zeta_\sigma(\mathbf{x}-\mathbf{y})
        \,\mathrm{d}\mathbf{y}
,\end{align*}
```
the Dirac delta collapses the integral, obtaining an approximation of the filtered vorticity field as
```math
\begin{align} % \label{eq:particle:blob}
        \overline{\boldsymbol\omega}\left( \mathbf{x},\,t \right)
    & \approx
        \sum\limits_p
            \boldsymbol{\Gamma}_p (t)
            \zeta_{\sigma_p}(\mathbf{x}-\mathbf{x}_p(t))
,\end{align}
```
where ``\zeta_{\sigma}(\mathbf{x}) \equiv \frac{1}{\sigma^3} \zeta \left(\frac{\Vert \mathbf{x} \Vert}{\sigma} \right)`` is the filter kernel of width ``\sigma`` and radial basis ``\zeta``.
As seen in Eq. (4), the filter operator has the effect of spreading the vortex strength ``\boldsymbol{\Gamma}_p`` in space, regularizing the singularity originally introduced by the Dirac delta.
Thus, the filter kernel takes the role of a basis function that is used to discretize ``\overline{\boldsymbol\omega}`` through particles.
We let the filter width ``\sigma`` (here on called *smoothing radius* or *core size*) change in time and space according to the evolution of each individual particle.
The particle field constructs a continuous vorticity field through radial basis functions as given by Eq. (4), and also a continuous velocity field by inverting the relation ``{\overline{\boldsymbol\omega} = \nabla \times \overline{\mathbf{u}}}`` as
```math
\begin{align} % \label{eq:U:reg}
        \overline{ \mathbf{u} }\left( \mathbf{x} \right)
    =
        - \frac{1}{4\pi}\sum\limits_p g_{\sigma_p}\left( \mathbf{x}-\mathbf{x}_p \right)
            \frac{\mathbf{x}-\mathbf{x}_p}{\Vert\mathbf{x}-\mathbf{x}_p\Vert^3}
            \times
            \boldsymbol\Gamma_p
,\end{align}
```
where ``g_{\sigma}`` is the regularizing function[^3] associated with the filter kernel ``\zeta_\sigma``.
Hence, all fluid properties—like ``\overline{\mathbf{u}}`` and its spatial derivatives—are continuous and can be computed analytically.


## Governing Equations

Similar to the process that led from Eq. (3) to Eq. (4), we use singular particles to discretize the LES-filtered vorticity equation given in Eq. (2), and arrive to the governing equations of the reformulated VPM:
```math
\begin{align}
    % \label{eq:rvpm:ge:dxdt}
    % \bullet \quad
    &
        \frac{\text{d}}{\text{d}t}{\bf x}_p
        =
        \overline{\mathbf{u}}({\bf x}_p)
\end{align}
```
```math
\begin{align}
    % \\ % \label{eq:rvpm:ge:dGammadt}
    \begin{split}
        % \bullet \quad
        & \frac{\mathrm{d} }{\mathrm{d} t} \boldsymbol{\Gamma}_p
        =
            \left(
                \boldsymbol{\Gamma}_p \cdot \nabla
            \right)
            \overline{\mathbf{u}} ({\bf x}_p)
            -
            \frac{g + f}{\frac{1}{3} + f}
            \left\{
                \left[
                    \left(
                            \boldsymbol{\Gamma}_p  \cdot \nabla
                        \right)
                        \overline{\mathbf{u}} {\small ({\bf x}_p)}
                \right]
                \cdot \hat{\boldsymbol{\Gamma}}_p
            \right\} \hat{\boldsymbol{\Gamma}}_p
        \\
        & \qquad \qquad \,\,\,
        \qquad \qquad \quad \,\,
            - \frac{C_d}{\zeta_{\sigma_p} ({\bf 0})}
            \left[
                \mathbf{E}_\mathrm{str} ({\bf x}_p)
                -
                \frac{f}{\frac{1}{3} + f}
                \left(
                    \mathbf{E}_\mathrm{str} ({\bf x}_p) \cdot \hat{\boldsymbol{\Gamma}}_p
                \right) \hat{\boldsymbol{\Gamma}}_p
            \right]
    \end{split}
\end{align}
```
```math
\begin{align}
    % \\ % \label{eq:rvpm:ge:dsigmadt}
    % \bullet \quad
    &
        \frac{\mathrm{d} }{\mathrm{d} t} \sigma_p
        =
        - \left(
            \frac{g + f}{1 + 3f}
        \right)
        \frac{\sigma_p}{\Vert \boldsymbol{\Gamma}_p \Vert}
            \left[
                \left(
                        \boldsymbol{\Gamma}_p  \cdot \nabla
                    \right)
                    \overline{\mathbf{u}} {\small ({\bf x}_p)}
            \right]
            \cdot \hat{\boldsymbol{\Gamma}}_p
        +
        \left(
            \frac{f}{1 + 3f}
        \right)
        \frac{\sigma_p}{\Vert \boldsymbol{\Gamma}_p \Vert}
        \frac{C_d}{\zeta_{\sigma_p} ({\bf 0})}
        \mathbf{E}_\mathrm{str} ({\bf x}_p) \cdot \hat{\boldsymbol{\Gamma}}_p
\end{align}
```
```math
\begin{align}
    % \\ % \label{eq:rvpm:ge:viscous}
    % \bullet \quad
    &
        \left(
            \frac{\text{d} }{\text{d} t} \overline{ \boldsymbol\omega }
        \right)_\mathrm{viscous}
        =
        \nu \nabla^2 \overline{ \boldsymbol\omega }
\end{align}
```
where Eq. (6) resolves vorticity advection by convecting the particles, Eq. (7) governs the evolution of vortex strength, and Eq. (8) governs the evolution of particle size.
Eq. (7) in conjunction with Eqs. (6) and (8) resolve the inviscid part of the LES-filtered vorticity Navier-Stokes equation, while the viscous part in Eq. (9) can be resolved through any of the schemes previously proposed in the literature (*e.g.*, vortex redistribution method, particle strength exchange, or core spreading).

The main headway of the reformulated VPM over the classic VPM is that rVPM uses the particle size, or ``\frac{\mathrm{d} }{\mathrm{d} t} \sigma_p``, as an extra degree of freedom to reinforce conservation laws.
As shown in References [^1] and [^2], momentum and mass conservation leads to ``f = 0`` and ``g = \frac{1}{5}``, and Eqs. (7) and (8) become
```math
\begin{align*}
    % \bullet \quad
    & \frac{\mathrm{d} }{\mathrm{d} t} \boldsymbol{\Gamma}_p
    =
        \left(
            \boldsymbol{\Gamma}_p \cdot \nabla
        \right)
        \overline{ \mathbf{u} } (\mathbf{x}_p)
        -
        \frac{3}{5}
        \left\{
            \left[
                \left(
                        \boldsymbol{\Gamma}_p  \cdot \nabla
                    \right)
                    \overline{ \mathbf{u} } {\small (\mathbf{x}_p)}
            \right]
            \cdot \hat{\boldsymbol{\Gamma}}_p
        \right\} \hat{\boldsymbol{\Gamma}}_p
        - \frac{C_d}{\zeta_{\sigma_p} (\mathbf{0})}
            \mathbf{E}_\mathrm{str} (\mathbf{x}_p)
    \\
    % \bullet \quad
    &
        \frac{\mathrm{d} }{\mathrm{d} t} \sigma_p
        =
        -
        \frac{1}{5}
        \frac{\sigma_p}{\Vert \boldsymbol{\Gamma}_p \Vert}
            \left[
                \left(
                        \boldsymbol{\Gamma}_p  \cdot \nabla
                    \right)
                    \overline{ \mathbf{u} } {\small (\mathbf{x}_p)}
            \right]
            \cdot \hat{\boldsymbol{\Gamma}}_p
,\end{align*}
```
which is the formulation referred to as the "reformulated VPM."
Notice that when ``f = g = 0`` and ``\mathbf{E}_\mathrm{str}`` is neglected, Eqs. (7) and (8) collapse back to the classic VPM equations, making these equations a generalization of the classic method.
In Reference [^2] is shown that the classic VPM turns out to violate both conservation of momentum and mass when it assumes ``\frac{\mathrm{d} }{\mathrm{d} t} \sigma_p = 0``, which explains the tendency of the classic VPM to be numerically unstable.
Furthermore, notice that the rVPM equations do not require more computation than the classic VPM: when SFS effects are neglected (``\mathbf{E}_\mathrm{str}=0``), both ``\frac{\mathrm{d} \sigma_p }{\mathrm{d}t}`` and ``\frac{\mathrm{d} \boldsymbol{\Gamma}_p}{\mathrm{d} t}`` are calculated directly and solely from vortex stretching, ``\left( \boldsymbol{\Gamma}_p \cdot \nabla  \right) \overline{\mathbf{u}} ({\bf x}_p)``.

> For an in-depth derivation of the rVPM governing equations, see Chapters 1 and 2 in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589).[^2]

## [Turbulence Model](@id sfsmodel)

Turning our attention back to the SFS stress tensor ``T_{ij}``, the accuracy of LES hinges on the modeling of this tensor.
Its divergence represents the rate at which enstrophy—a measure of rotational kinetic energy—is transferred from resolved scales to subfilter scales (diffusion) and from subfilter scales to resolved scales (backscatter).
In vortex methods, the most common SFS models use variants of the Smagorinsky eddy-viscosity model formulated for the vorticity stress.[^4][^5]
However, these models are developed on the basis of homogeneous isotropic turbulence, which makes them overly diffusive in simulations with coherent vortical structures.
In Reference [^1], the following anisotropic model of SFS vortex stretching is proposed:
```math
\begin{align*} % \label{eq:Estr}
        \mathbf{E}_\mathrm{str} \left( \mathbf{x} \right)
    \approx
        \sum\limits_q
            \zeta_{\sigma}(\mathbf{x}-\mathbf{x}_q)
            \left(
                \boldsymbol{\Gamma}_q \cdot \nabla
            \right)
            \left(
                \overline{\mathbf{u}} \left( \mathbf{x} \right) - \overline{\mathbf{u}} \left( \mathbf{x}_q \right)
            \right)
.\end{align*}
```
The model coefficient ``C_d`` is calculated dynamically at the position of every particle as
```math
\begin{align*} % \label{eq:Cd:GammaMLave}
        C_d
    =
        \frac{
            \left< \boldsymbol\Gamma_p \cdot \mathbf{L} \right>
        }{
            \left< \boldsymbol\Gamma_p \cdot \mathbf{m} \right>
        }
,\end{align*}
```
where ``\left< \cdot \right>`` denotes an integration along Lagrangian trajectories[^6], and
```math
\begin{align*}
    &
        \mathbf{m} =
            \frac{\sigma^3}{\zeta(0)} \frac{\partial \mathbf{E}_\mathrm{str} }{\partial \sigma} (\mathbf{x}_p)
    \\
    &
        \mathbf{L} =
            \frac{3}{\sigma}
            \left( \boldsymbol\Gamma_p \cdot \nabla \right)
            \left(
                \mathbf{u} (\mathbf{x}_p) - \overline{\mathbf{u}} (\mathbf{x}_p)
            \right)
            +
            \left( \boldsymbol\Gamma_p \cdot \nabla \right)
            \frac{\partial \overline{\mathbf{u}} }{\partial \sigma}  (\mathbf{x}_p)
.\end{align*}
```
This dynamic procedure is based on a simultaneous balance of enstrophy-production and derivatives between true and modeled SFS contributions.
Backscatter is controlled by clipping the model coefficient to ``C_d=0`` whenever the condition ``C_d \boldsymbol\Gamma_p \cdot \mathbf{E}_\mathrm{str} (\mathbf{x}_p) \geq 0`` is not satisfied.
This results in a low-dissipation SFS model that uses vortex stretching as the physical mechanism for turbulence, which is well suited for flows with coherent vortical structures where the predominant cascade mechanism is vortex stretching.

> For an in-depth derivation of the SFS model, see Chapter 3 in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589).[^2]


## Immersed Vorticity

In order to immerse the vorticity of solid boundaries into the LES-filtered Navier-Stokes equations, the filtered vorticity field ``\overline{\boldsymbol\omega}(\mathbf{x}, t)`` is decomposed into a free-vorticity field ``\overline{\boldsymbol\omega}_\mathrm{free}(\mathbf{x}, t)`` and a bound-vorticity field ``\overline{\boldsymbol\omega}_\mathrm{bound}(\mathbf{x}, t)`` as
```math
\begin{align*}
        \overline{\boldsymbol\omega}
    =
        \overline{\boldsymbol\omega}_\mathrm{free} + \overline{\boldsymbol\omega}_\mathrm{bound}
.\end{align*}
```
Both components can be discretized with vortex particles as
```math
\begin{align*}
        \overline{\boldsymbol\omega} (\mathbf{x})
    =
        \underbrace{
            \sum\limits_p {\boldsymbol\Gamma}_p \zeta_{\sigma_p} \left( \mathbf{x} - \mathbf{x}_p \right)
        }_{\overline{\boldsymbol\omega}_\mathrm{free}}
        +
        \underbrace{
            \sum\limits_b {\boldsymbol\Gamma}_b \zeta_{\sigma_b} \left( \mathbf{x} - \mathbf{x}_b \right)
        }_{\overline{\boldsymbol\omega}_\mathrm{bound}}
,\end{align*}
```
where the particles discretizing the free-vorticity field evolve according to the rVPM governing equations, Eqs. (6) through (9), while the ones discretizing the bound-vorticity are embedded on the solid boundaries and their strength is calculated by actuator models derived in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589),[^2] Chapter 6.
The velocity field is obtained by inverting the relation ``{\boldsymbol\omega = \nabla \times \mathbf{u}}``, resulting in
```math
\begin{align*}
        \overline{ \mathbf{u} }\left( \mathbf{x} \right)
    =
        \underbrace{
            \sum\limits_p g_{\sigma_p}\left( \mathbf{x}-\mathbf{x}_p \right)
                \mathbf{K}\left( \mathbf{x}-\mathbf{x}_p \right)
                \times
                \boldsymbol\Gamma_p
        }_{\overline{\mathbf{u}}_\mathrm{free}}
        +
        \underbrace{
            \sum\limits_b g_{\sigma_b}\left( \mathbf{x}-\mathbf{x}_b \right)
                \mathbf{K}\left( \mathbf{x}-\mathbf{x}_b \right)
                \times
                \boldsymbol\Gamma_b
        }_{\overline{\mathbf{u}}_\mathrm{bound}}
,\end{align*}
```
which includes the velocity induced by both free and bound vorticity components, and where `` \mathbf{K}\left( \mathbf{x} \right) \equiv - \frac{1}{4\pi} \frac{\mathbf{x}}{\Vert\mathbf{x}\Vert^3}``.
Thus, the evolution of the free particles is influenced by the vorticity immersed at the solid boundaries, affecting their convection and vortex stretching through the velocity field induced by the bound particles.

The immersed vorticity not only affects the evolution of existing free vorticity, but it also creates new free vorticity at the boundary through viscous diffusion.
In reality, vorticity is created in the boundary layer, it builds up as it travels along the surface, and it is eventually shed off the surface either by the Kutta condition at the trailing edge, flow separation, or other turbulent mechanisms.
On a slender body, the vorticity can be assumed to be shed at the trailing edge.

In most models inside FLOWUnsteady, instead of creating vorticity through the viscous diffusion equation, the immersed vorticity is shed along a prescribed trailing edge.
This approach neglects the wake created by flow separation.
However, the effects of flow separation on loading (like the drop in lift and increase in pressure drag on a stalled airfoil) can still be captured whenever lookup airfoil tables are used.

!!! compat "Recommended"
    For an in-depth discussion of the actuator line and surface models implemented in FLOWUnsteady, see Chapter 6 in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589).[^2]

## Other Schemes

In the default settings of FLOWUnsteady, vortex stretching is resolved with the transposed scheme and the divergence of the vorticity field is treated through the relaxation scheme developed by Pedrizzeti.[^7]
The time integration of the governing equations is done through a low-storage third-order Runge-Kutta scheme.
A Gaussian kernel is used as the LES filter ``\zeta_\sigma`` (or VPM radial basis function).
Like the classic VPM, the reformulated VPM is spatially second-order accurate in the convective term when a Gaussian basis is used.
Viscous diffusion is solved through the core spreading method coupled with the radial basis function interpolation approach for spatial adaptation developed by Barba.[^8]
This viscous scheme has second-order spatial convergence, while showing linear convergence when coupled with spatial adaptation.
The fast multipole method (FMM) is used for the computation of the regularized Biot-Savart law, approximating the velocity field and vortex stretching through spherical harmonics with computational complexity ``\mathcal{O}(N)``, where ``N`` is the number of particles.
The FMM computation of vortex stretching is performed through an efficient complex-step derivative approximation,[^9] implemented in a modified version of the open-source, parallelized code [ExaFMM](https://joss.theoj.org/papers/10.21105/joss.03145).
[FLOWVPM](https://github.com/byuflowlab/FLOWVPM.jl) and [FLOWUnsteady](https://github.com/byuflowlab/FLOWUnsteady) are implemented in [the Julia language](https://julialang.org), which is a modern, high-level, dynamic programming language for high-performance computing.

!!! compat "Recommended"
    For an in-depth discussion of the numerical schemes implemented in FLOWUnsteady, see Chapter 4 in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589).[^2]

[^1]: E. J. Alvarez & A. Ning (2022), "Reviving the Vortex Particle Method: A Stable Formulation for Meshless Large Eddy Simulation," *(in review)*. [**[PDF]**](https://arxiv.org/pdf/2206.03658.pdf)

[^2]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and Meshless Large Eddy Simulation of Multirotor Aircraft," *Doctoral Dissertation, Brigham Young University*. [**[VIDEO]**](https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html) [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)

[^3]: Winckelmans, G., and Leonard, A., “Contributions to Vortex Particle Methods for the Computation of Three-Dimensional Incompressible Unsteady Flows,” Journal of Computational Physics, Vol. 109, No. 2, 1993.

[^4]: Winckelmans, G. S., “Some progress in large-eddy simulation using the 3D vortex particle method,” CTR Annual Research Briefs, , No. 2, 1995, pp. 391–415.

[^5]: Mansfield, J. R., Knio, O. M., and Meneveau, C., “A Dynamic LES Scheme for the Vorticity Transport Equation: Formulation and a Priori Tests,” Journal of Computational Physics, Vol. 145, No. 2, 1998, pp. 693–730.

[^6]: Meneveau, C., Lund, T. S., and Cabot, W. H., “A Lagrangian dynamic subgrid-scale model of turbulence,” Journal of Fluid Mechanics, Vol. 319, No. -1, 1996, p. 353.

[^7]: Pedrizzetti, G., “Insight into singular vortex flows,” Fluid Dynamics Research, Vol. 10, No. 2, 1992, pp. 101–115.

[^8]: Barba, L. A., Leonard, A., and Allen, C. B., “Advances in viscous vortex methods - Meshless spatial adaption based on radial basis function interpolation,” International Journal for Numerical Methods in Fluids, Vol. 47, No. 5, 2005, pp. 387–421. Also, Barba, L. A., “Vortex Method for computing high-Reynolds number Flows: Increased accuracy with a fully mesh-less formulation,” California Institute of Technology, Vol. 2004, 2004.

[^9]: Alvarez, E. J., and Ning, A., “High-Fidelity Modeling of Multirotor Aerodynamic Interactions for Aircraft Design,” AIAA Journal, Vol. 58, No. 10, 2020, pp. 4385–4400.

[^a]: Let ``\phi`` be a field and ``\zeta_\sigma`` a filter kernel with cutoff length ``\sigma``, the filter operator is defined as ``\overline{\phi} \left( \mathbf{x} \right) \equiv \int\limits_{-\infty}^\infty \phi(\mathbf{y})\zeta_\sigma(\mathbf{x}-\mathbf{y}) \,\mathrm{d}\mathbf{y}``.
