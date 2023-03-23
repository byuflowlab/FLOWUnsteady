# Quasi-Steady Solver

While unsteady simulations are resolved using the reformulated VPM,
FLOWUnsteady also provides a quasi-steady solver for low-fidelity
simulations.
The quasi-steady solver replaces the particle field with semi-infinite
rigid wakes in wings and blade-element momentum theory in rotors.

The quasi-steady solver is invoked by simply changing the line
```julia
VehicleType     = uns.UVLMVehicle
```
to
```julia
VehicleType   = uns.QVLMVehicle
```
in the previous sections.
The results of the quasi-steady solver are shown below, predicted
through blade-element momentum theory.

```@raw html
<center>
    <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//propeller-Jsweep-exampleBEMT.png" alt="Pic here" style="width: 100%;"/>
</center>
```

