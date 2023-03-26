# Convergence

The following is a compilation of convergence studies found in the literature
using FLOWUnsteady.

## Wing Performance

*Source: E. J. Alvarez, 2022*[^1]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-particles06.png" alt="Pic here" style="width: 40%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-n100-00.png" alt="Pic here" style="width: 40%;"/>
</center>
```

> **Case:** ``45^\circ`` swept-back wing at an angle of attack of ``4.2^\circ``.
> Aspect ratio of 5.0, RAE 101 airfoil section with 12% thickness, no dihedral,
> twist, nor taper. Freestream velocity of $49.7,\mathrm{m/s}$, corresponding to
> a chord-based Reynolds number of $1.7 \times 10^6$.
> Convergence of loading distribution and integrated lift and drag as the number
> of wing elements is increased.

> **Takeaways:**
> * Resolving the wake for about 1.5 span-distances is sufficient to converge wing loading
> * Using between 100 and 200 elements fully resolves the wing loading ($C_D$
>   changes by less than 1% with $n_\mathrm{wing} > 100$)

```@raw html
<table>
  <tr>
    <td align=center>
      <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-conv00.png"
                                                        alt="Pic here" style="width: 65%;"/>
    </td>
    <td align=center>
      <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-conv01.png"
                                                        alt="Pic here" style="width: 80%;"/>
    </td>
  </tr>
</table>

<br>

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-conv02.png" alt="Pic here" style="width: 100%;"/>
</center>
```






## Rotor Performance

#### APC 10x7 Propeller Case
*Source: E. J. Alvarez and A. Ning, 2020*[^2]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/propmodel00.png" alt="Pic here" style="width: 55%;"/>
</center>
```

> **Case:** APC 10x7E propeller at an advance ratio of 0.6, tip Mach number of
> 0.36, and $\mathrm{Re}_c = 6.2 \times 10^4$ and
> $\mathrm{Re}_D = 6.5 \times 10^5$.
> Convergence of thrust w.r.t. the following parameters:
> * ``N_\mathrm{steps}``, number of time steps per revolution.
> * ``N_\mathrm{sheds}``, number of particle sheds per revolution.
> * ``\lambda``, core overlap between tip particles defined as
>   ``\lambda=\frac{\sigma}{\Delta x}``. Also controlled with the particle
>   smoothing factor ``f_\sigma = \sigma/R``, .
> * ``n``, number of blade elements per blade.
> Default values: ``N_\mathrm{steps}=72``, ``N_\mathrm{sheds}=144``,
> ``\lambda=2.125`` (or ``f_\sigma=0.093``), and ``n=50`` as each parameter is
> independently varied.

> **Takeaways:**
> * Temporal discretization error less than 1% with ``N_\mathrm{steps} \geq 72``
>   (or time step smaller  than ``5^\circ``)
> * Spatial discretization error less than 1% with ``N_\mathrm{sheds} \geq 144``
>   and ``n = 50``.
> * ``C_T`` starts to diverge as the stability threshold (``\lambda=1``) is
>   approached, while also diverging as ``f_\sigma>0.25`` (equivalent to
>   ``\sigma`` larger than ``0.25R``) leads to unphysical wake dynamics caused
>   by excessive smoothing. Since the instability associated with
>   ``\lambda\rightarrow1`` plateaus at ``\lambda\approx2``, we recommend
>   using ``\lambda \approx 2.125`` whenever possible.

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/apcconv00.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<table>
  <tr>
    <td align=center>
      <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/apcconv01.png"
                                                        alt="Pic here" style="width: 100%;"/>
    </td>
    <td align=center>
      <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/apcconv02.png"
                                                        alt="Pic here" style="width: 100%;"/>
    </td>
  </tr>
</table>

<br>
```

#### Beaver Propeller Case
Beaver convergence.

#### Wind Turbine Case
*Source: J. Mehr, E. J. Alvarez, and A. Ning, 2022*[^4]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/UAEturbine.png" alt="Pic here" style="width: 45%;"/>
</center>
```

> **Case:** Test series "H" from UAE study at US Department of Energy.

> **Takeaways:**
> * Spatio-temporal discretization error on thrust $C_T$ is less than 1% with 72
>   steps per revolution (or ``5^\circ`` per step), while error on power $C_P$ is
>   less than 3%.
> * Blade discretization error less than 0.1% with 50 blade elements per blade.

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/UAEturbine-conv00.png"
      alt="Pic here" style="width: 100%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/UAEturbine-conv01.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```


## Rotor Wake
Beaver wake convergence.

## Immersed Vorticity
1/100 $\sigma$ rule.







[^1]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and Meshless
    Large Eddy Simulation of Multirotor Aircraft," *Doctoral Dissertation, Brigham
    Young University*. [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)

[^2]: E. J. Alvarez & A. Ning (2020), "High-Fidelity Modeling of Multirotor
    Aerodynamic Interactions for Aircraft Design," *AIAA Journal*.
    [**[DOI]**](https://doi.org/10.2514/1.J059178)
    [**[PDF]**](https://scholarsarchive.byu.edu/facpub/4179/)

[^4]: J. Mehr, E. J. Alvarez, & A. Ning (2022), "Interactional Aerodynamics
    Analysis of a Multi-Rotor Energy Kite," (in review).
