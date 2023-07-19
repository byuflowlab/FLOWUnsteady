# Quasi-Steady Solver
The aerodynamic and aeroacoustic analysis can also be performed using the
quasi-steady solver (which uses BEMT for the aerodynamic solution), by
simply changing the following parameter in
[the aero solution](@ref rotorhoveraero):
```julia
VehicleType     = uns.QVLMVehicle
```
and this parameter when [calling PSU-WOPWOP](@ref rotorhovernoise):
```julia
const_solution  = true
n = 50   # <---- For some reason PSU-WOPWOP breaks with less blade elements
```
