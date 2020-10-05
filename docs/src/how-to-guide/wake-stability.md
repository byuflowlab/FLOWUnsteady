# How to Approach Wake Stability Issues

The vortex particle method does pretty well until a wake reaches turbulent breakdown, at which point the solver can become unstable.  There are several ways to try to maintain stability.  Some include inherent object physics, others have to do with simulation parameters, and others involve removing particles from the simulation.  The following are some things that the users have encountered and found helpful.

## Inherent Physics

The inherent physics of the problem may lead to turbulent breakdown happening sooner or later.  For example, there is a direct relationship between number of rotor blades and when breakdown happens.  The more blades there are, the sooner breakdown will occur.  In addition, the higher the tipspeed ratio, the sooner breakdown will occur.  Some instances of turbulent breakdown are therefore unavoidable, and the user should be aware of this limitation.

## Simulation Parameters

Sometimes, changing the simulation parameters can help keep things numerically stable for longer.  For example, the resoultion of the simulation, that is, the number and spacing of the horseshoes and particles can affect stability.

In addition, there are some relaxation parameters available in the various solvers that can improve or worsen stability.  Referring to the reference section of this documentation will reveal what parameters there are and what they are for. Some of the papers cited in the Theory section also indicate some recommended parameters.

## Wake Trimming

One method to maintain stability is to remove some of the wake particles.  This is done using an extra runtime function described in [How to Set up Run-time Functions](@ref).

Let's say you wanted to remove the particles after a certain point behind your vehicle. One way you could do this is to obtain the location of an object in your system, let's say a rotor, and then delete the particles behind a certain point relative to that location.

```julia
rotors = vcat(sim.vehicle.rotor_systems...) #get rotors from simulation
origin = rotors[1]._wingsystem.O            #choose an object origin
refxaxis = rotors[1]._wingsystem.Oaxis[1,:] #get the origin axis

#loop through all the particles and delete if behind cuttoff point
for i in vpm.get_np(pfield):-1:1
    particleposition = vpm.get_x(pfield, i) #obtain particle absolute position
    particlevector = particlepostition - origin #obtain particle vector relative to origin

    #delete if dinstance from orgin in x-direction is greater than cutoff
    if abs(dot(particlevector,refxaxis)) > cutoffcriteria
        vpm.delparticle(pfield, i)
    end
end
```

You could also just remove everything a certain distance from your origin of choice, which would take everything other than a sphere around that object.

Furthermore, you can define more advanced cutoff criteria, but special consideration may be in order.  For example, some systems rotate (like rotors and tilting systems) so their coordinate systems also rotate in time.  So if you wanted to delete particles outside a certain boundary from a rotor, you could do so for a cylindrical boundary.

```julia
refyaxis = rotors[1]._wingsystem.Oaxis[2, :] # reference y axis
refzaxis = rotors[1]._wingsystem.Oaxis[3, :] # reference z axis

#calculate the radial position of the particle
radialpos = (sqrt(dot(parvec,refyaxis)^2 + dot(parvec,refzaxis)^2)
```

The Reference section of this documentation will cover all the fields in the wing system objects available to the user. How they are used, and what cutoff criteria in needed depends on the specific use and the user's desires.