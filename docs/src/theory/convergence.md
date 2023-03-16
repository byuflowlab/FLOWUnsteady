# Convergence

The following is a compilation of convergence studies found in the literature
using FLOWUnsteady.

## Wing Performance

*Reference: E. J. Alvarez, 2022*[^1]

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

[^1]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and Meshless
    Large Eddy Simulation of Multirotor Aircraft," *Doctoral Dissertation, Brigham
    Young University*. [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)


## Rotor Performance

## Rotor Wake

## Immersed Vorticity
