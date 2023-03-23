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
  <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//weber-val00.png" alt="Pic here" style="width: 100%;"/>
</center>
```

## Rotor

#### Hover Case
DJI aero from rVPM paper. DJI acoustics from VFS paper.

#### Forward Flight Case
Rachael's AVIATION paper.

## Wind Turbine
Judd's validation

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
> $\mathrm{Re}_D = 6.5 \times 10^5$.

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
Beaver J sweep. Beaver incidence angle sweep. Propeller wake.

## Rotor-Rotor Interactions

## Rotor-Wing Interactions

#### Horizontal-Stabilizer w/ Tip-Mounted Propellers

#### Blown Wing

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
