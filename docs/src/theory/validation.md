# Validation

The following is a compilation of validation studies found in the literature
using FLOWUnsteady.

## Wing

*Sources: E. J. Alvarez, 2022,*[^1] *and E. J. Alvarez and A. Ning, 2022*[^2]



```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-particles06.png" alt="Pic here" style="width: 40%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-n100-00.png" alt="Pic here" style="width: 40%;"/>
</center>
```

> **Case:** ``45^\circ`` swept-back wing at an angle of attack of ``4.2^\circ``,
> aspect ratio of 5.0, RAE 101 airfoil section with 12% thickness, no dihedral,
> twist, nor taper. Freestream velocity of $49.7,\mathrm{m/s}$, corresponding to
> a chord-based Reynolds number of $1.7 \times 10^6$. The high sweep of the
> wing causes non-negligible spanwise flow. The wing loads reported by
> Weber and Brebner (experimental) were integrated from pressure-tap measurements,
> hence the drag reported in this section includes induced and form drag
> while excluding skin friction drag.

> **Results:** (Excerpt from E. J. Alvarez and A. Ning, 2022[^2])
> *"Fig. 9 shows the loading distribution and integrated lift and drag across
> AOA predicted with the actuator surface model (ASM), compared to the experimental
> measurements. The loading distribution shows satisfactory agreement with the
> experiment, validating that both the circulation solver and the force
> calculation... are accurate for predicting not only lift but also drag
> distribution across the span. The integrated lift and drag (Fig. 9, bottom)
> show excellent agreement with the experiment from $0^\circ$ to $10.5^\circ$.
> We expect this to be the case only for mild AOAs before approaching stall
> conditions since our ASM does not capture the mechanisms of flow separation.
> Thus, through this swept-wing case, we gain confidence that our ASM yields
> accurate predictions in conditions with spanwise flow up to a moderate AOA."*


```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-val00.png"
                                          alt="Pic here" style="width: 100%;"/>
</center>
```

## Rotor

#### Hover Case: Aero

*Source: E. J. Alvarez, 2022*[^1]*, and E. J. Alvarez and A. Ning, 2022*[^6]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/visualization-highfidelity00.png"
                                            alt="Pic here" style="width: 80%;"/>
</center>
```

> **Case:** DJI 9443 rotor in hover at 5400 RPM. This is a two-bladed rotor
> with a diameter of 9.4 inches. Validation case at Mach number of 0.2,
> $\mathrm{Re}_c = 6 \times 10^4$, and $\mathrm{Re}_{0.7D} = 7 \times 10^5$.
> Thrust and blade loading are compared to experiment and mesh-based CFD.
> Computational time is benchmarked against conventional mesh-based CFD.


> **Results:** (Excerpt from E. J. Alvarez and A. Ning, 2020[^6])
> *"...the VPM simulation shows excellent agreement with the experiment,
> predicting a mean $C_T$ value within 2% of the experimental mean value...
> Our meshless LES appears to be two orders of magnitude faster than a
> mesh-based LES with similar fidelity, while being one order of magnitude
> faster than a low-fidelity URANS simulation and three orders of magnitude
> than high-fidelity DES."*

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-dji9443-aero00.png"
      alt="Pic here" style="width: 49%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-dji9443-aero01.png"
      alt="Pic here" style="width: 49%;"/>
  <br><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-dji9443-aero02.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```


#### Hover Case: Acoustics

*Source: E. J. Alvarez, A. Schenk, T. Critchfield, and A. Ning, 2020*[^7]

> **Case:** Same DJI 9443 case in hover. FLOWUnsteady predictions (VPM) are
> compared against experiment and URANS (STAR-CCM+) and DES (OVERFLOW2).

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-dji9443-noise00.png"
      alt="Pic here" style="width: 100%;"/>
  <br><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-dji9443-noise10.png"
      alt="Pic here" style="width: 70%;"/>
</center>

<br>
```
NOTE: The broadband results in E. J. Alvarez, A. Schenk, T. Critchfield, and
A. Ning, 2020[^7] contained a bug that ended up evaluating the microphone array
in the wrong plane, which led to a large discrepancy in A-weighted
OASPL. For updated results, see the [Rotor in Hover tutorial](@ref rotorhovernoise).

#### Forward Flight Case

*Source: R. M. Erhard and J. J. Alonso, 2022*[^8]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard00.png"
                                            alt="Pic here" style="width:100%;"/>
</center>
```

> **Case:** TUD F29 rotor in forward flight at an incidence angle (edgewise
> flow). This is a four-bladed rotor. Thrust and power predicted by FLOWUnsteady
> (VPM) was compared to experiment and low-fidelity methods at  multiple
> incidence angles $\alpha$ and advance ratios $J$.

> **Results:** Good agreement between FLOWUnsteady and experiment for incidence
> angles up to $\alpha=60^\circ$ across advance ratio.

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard01.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```


## Wind Turbine
*Source: J. Mehr, E. J. Alvarez, and A. Ning, 2022*[^5]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/UAEturbine.png" alt="Pic here" style="width: 45%;"/>
</center>
```

> **Case:** Test series "H" from UAE study at US Department of Energy. Sweep on
> tip speed ratio $\lambda = \frac{\omega R}{u_\infty}$.


> **Results:** (Excerpt from J. Mehr, E. J. Alvarez, and A. Ning, 2020[^5])
> *"The torque coefficient outputs are also within 10% for tip speed ratios
> above, and within 25% for tip speed ratios under, $\lambda = 2.0$;
> also notice that the absolute magnitudes of the torque coefficient are very
> small at low tip speed ratios, assuaging any concerns about the higher
> relative errors at those operational states."*

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/UAEturbine-val00.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```

## Propeller

#### APC 10x7 Case
*Source: E. J. Alvarez and A. Ning, 2020*[^3]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/apc10x7geom.png" alt="Pic here" style="width: 55%;"/>
</center>
```

> **Case:** APC 10x7E propeller (2 blades, 10'' diameter, solidity 0.1) at a tip
> Mach number of 0.36, $\mathrm{Re}_c = 6.2 \times 10^4$, and
> $\mathrm{Re}_D = 6.5 \times 10^5$. Sweep on advance ratio
> $J = \frac{u_\infty}{n d}$.

> **Results:** (Excerpt from E. J. Alvarez and A. Ning, 2020[^3])
> *"... it is confirmed that the VPM propeller model is valid... across low and
> moderately high advance ratios."*

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/apc10x7val00.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```


#### Beaver Case
*Source: E. J. Alvarez*[^3]*and E. J. Alvarez and A. Ning, 2020*[^9]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/prowim_isolatedprop-vortvol_3-shortsmall01.gif"
      alt="Vid here" style="width: 75%;"/>
</center>
```

> **Case:** Beaver propeller (4 blades, 0.237 m diameter) at
> $\mathrm{Re}_{0.7D} = 1.8\times 10^6$.
> Extensive validation with predicted thrust, torque, efficiency, blade loading, and
> flow field compared to experimental measurements.

```@raw html

<center>
  <b>Sweep of advance ratio</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver01.png"
      alt="Pic here" style="width: 100%;"/>

  <br>
  <b>Sweep of incidence angle</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver02.png"
      alt="Pic here" style="width: 75%;"/>

  <br><br>
  <b>Blade loading</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver03.png"
      alt="Pic here" style="width: 100%;"/>

  <br>
  <b>Wake structure</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver04.png"
      alt="Pic here" style="width: 80%;"/>

  <br><br>
  <b>Flow field</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver05.png"
      alt="Pic here" style="width: 100%;"/>
  <br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-beaver06.png"
      alt="Pic here" style="width: 100%;"/>

  <br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/prowim_isolatedprop-vortvolslice_1-shortsmall00.gif"
      alt="Vid here" style="width: 70%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/prowim_isolatedprop-vortslice_1-shortsmall00.gif"
      alt="Vid here" style="width: 70%;"/>
</center>

<br>
```


## Rotor-Rotor Interactions

#### Side-by-Side Rotors in Hover
*Source: E. J. Alvarez and A. Ning, 2020*[^3]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-sidebyside00.png" alt="Pic here" style="width: 55%;"/>
</center>
```

> **Case:** Two side-by-side DJI rotors in hover. Two-bladed rotors with a
> diameter of 9.4 inches. Test at 4860 RPM, tip Mach number of 0.18,
> $\mathrm{Re}_c = 6.2\times 10^4$, and $\mathrm{Re}_{0.7D} = 6.5\times 10^5$.
> Aerodynamic interactions as the tip-to-tip spacing $s$ between rotors is
> decreased.

> **Results:** FLOWUnsteady accurately captures both the drop in thrust and
> the increase in fluctuations as rotors are brought closer together.

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-sidebyside01.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```

#### Side-by-Side and Tandem Propeller
*Source: R. M. Erhard and J. J. Alonso, 2022*[^8]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard02.png"
                                            alt="Pic here" style="width:100%;"/>
</center>
```

> **Case:** Two TUD F29 propellers in side-by-side and tandem configurations at
> an multile incidence angles (edgewise flow). This is a four-bladed rotor. Thrust and power predicted by FLOWUnsteady
> (VPM) was compared to experiment and low-fidelity methods at  multiple
> incidence angles $\alpha$ and advance ratios $J$.

> **Results:** Good agreement between FLOWUnsteady and experiment for incidence
> angles up to $\alpha=60^\circ$ across advance ratio.

```@raw html

<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard05.png"
    alt="Pic here" style="width: 90%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard04.png"
    alt="Pic here" style="width: 70%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard03.png"
      alt="Pic here" style="width: 50%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-erhard06.png"
      alt="Pic here" style="width: 47%;"/>
</center>

<br>
```

## Rotor-Wing Interactions

#### Tailplane w/ Tip-Mounted Propellers
*Source: E. J. Alvarez*[^3]*and E. J. Alvarez and A. Ning, 2020*[^9]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane00.png"
      alt="Pic here" style="width: 100%;"/>
</center>
```

> **Case:** Horizontal stabilizer (tailplane) with tip-mounted Beaver propellers.
> Wing with low aspect ratio ($b/c=2.7$), symmetric NACA $64_2-\mathrm{A}015$
> profile, and elevator flap. Test at advance ratio $J$ of 0.8 and inboard-up propeller
> rotation at a thrust setting of $C_T = 0.0935$.
> Flap deflection captured in the actuator surface model (ASM) through an
> equivalent twist.
> Effects of wake impingement on wing loading predicted by FLOWUnsteady are
> compared to experiment.

```@raw html

<center>
  <b>Flat wing ($\alpha = 0^\circ$, $\delta_e=0^\circ$)</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane03.png"
      alt="Pic here" style="width: 90%;"/>

  <br><br><br>
  <b>Lift augmentation</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane02.png"
      alt="Pic here" style="width: 35%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane08.png"
      alt="Pic here" style="width: 50%;"/>

  <br><br><br>
  <b>Wing wake and Streamtube</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane04.png"
      alt="Pic here" style="width: 64%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane05.png"
      alt="Pic here" style="width: 35%;"/>

  <br><br><br>
  <b>Effects of swirl direction</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane06.png"
      alt="Pic here" style="width: 100%;"/>

  <br><br><br>
  <b>Strong impingement</b><br>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-tailplane09.png"
      alt="Pic here" style="width: 100%;"/>
</center>

<br>
```

#### Blown Wing
*Source: E. J. Alvarez*[^3]*and E. J. Alvarez and A. Ning, 2020*[^9]

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-blown00.png"
      alt="Pic here" style="width: 40%;"/>
</center>
```

> **Case:** Straight wing with Beaver propellers mounted mid-span.
> Wing of aspect ratio $b/c=5.33$, symmetric NACA $64_2-\mathrm{A}015$
> profile. Test at advance ratio $J$ of 0.85 and inboard-up propeller
> rotation at a thrust setting of $C_T = 0.121$.
> Effects of wake impingement on wing loading predicted by FLOWUnsteady are
> compared to experiment.

```@raw html
<center>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-blown01.png"
      alt="Pic here" style="width: 80%;"/>
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val-blown02.png"
      alt="Pic here" style="width: 80%;"/>
</center>

<br>
```




[^1]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and Meshless
    Large Eddy Simulation of Multirotor Aircraft," *Doctoral Dissertation, Brigham
    Young University*. [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)

[^2]: E. J. Alvarez & A. Ning (2022), "Meshless Large Eddy Simulation of
    Rotor-Wing Interactions Through the Reformulated Vortex Particle Method," (in
    review).

[^3]: E. J. Alvarez & A. Ning (2020), "High-Fidelity Modeling of Multirotor
    Aerodynamic Interactions for Aircraft Design," *AIAA Journal*.
    [**[DOI]**](https://doi.org/10.2514/1.J059178)
    [**[PDF]**](https://scholarsarchive.byu.edu/facpub/4179/)

[^5]: J. Mehr, E. J. Alvarez, & A. Ning (2022), "Interactional Aerodynamics
    Analysis of a Multi-Rotor Energy Kite," *(in review)*.

[^6]: E. J. Alvarez & A. Ning (2022), "Reviving the Vortex Particle Method: A
    Stable Formulation for Meshless Large Eddy Simulation," *(in review)*.
    [**[PDF]**](https://arxiv.org/pdf/2206.03658.pdf)

[^7]: E. J. Alvarez, A. Schenk, T. Critchfield, and A. Ning (2020), “Rotor-on-Rotor
    Aeroacoustic Interactions of Multirotor in Hover,” *VFS 76th Forum*.
    [**[PDF]**](https://scholarsarchive.byu.edu/facpub/4053/)

[^8]: R. M. Erhard and J. J. Alonso (2022), "Comparison of Propeller Wake Models
    for Distributed Electric Propulsion and eVTOL Aircraft in Complex Flow
    Conditions," *AIAA SciTech Forum*.
    [**[PDF]**](https://www.researchgate.net/publication/357565378_A_Comparison_of_Propeller_Wake_Models_for_Distributed_Electric_Propulsion_and_eVTOL_Aircraft_in_Complex_Flow_Conditions)

[^9]: E. J. Alvarez & A. Ning (2022), "Meshless Large Eddy Simulation of
    Rotor-Wing Interactions with Reformulated Vortex Particle Method,"
    *(in review)*.
