# Generating components

## Rotor

FLOWunsteady uses a database of airfoil and rotor geometries
to automate the generation of rotors. A
[prepopulated database](https://github.com/byuflowlab/FLOWUnsteady/tree/master/database)
is found in the directory under `FLOWUnsteady.default_database`.
Alternatively, users can define their own database with custom rotors and
airfoils.

The following slides describe the structure of the database, using the DJI 9443
rotor as an example:


```@raw html
<div style="position:relative;padding-top:60%;">
<iframe style="position:absolute;left:0;top:0;height:100%;width:100%;" src="https://docs.google.com/presentation/d/e/2PACX-1vRsYbuuMFQdc05NRrQ3Db0RT4XKKoxEYDiUi0MpW58W6A-pp0sDHQI9mVqNFagPtQ/embed?start=true&loop=true&delayms=3000" frameborder="0" width="100%" height="100%" allowfullscreen="true" mozallowfullscreen="true" webkitallowfullscreen="true"></iframe>
</div>
```

Rotors can then be generated calling any of following functions:

```@docs
FLOWUnsteady.generate_rotor(::String)
FLOWUnsteady.generate_rotor(::Real, ::Real, ::Int, ::String)
FLOWUnsteady.generate_rotor(::Real, ::Real, ::Int, ::Array{Float64,2},
                            ::Array{Float64,2}, ::Array{Float64,2},
                            ::Array{Float64,2},
                            ::Array{Tuple{Float64,Array{Float64, 2},String},1})
FLOWUnsteady.vlm.rotate
```

## VLM Wing
```@docs
FLOWUnsteady.vlm.simpleWing
FLOWUnsteady.vlm.complexWing
```

## FLOWVLM Systems
```@docs
FLOWUnsteady.vlm.WingSystem

FLOWUnsteady.vlm.addwing
FLOWUnsteady.vlm.get_wing
FLOWUnsteady.vlm.setcoordsystem
FLOWUnsteady.vlm.get_m
FLOWUnsteady.vlm.get_mBlade
```
