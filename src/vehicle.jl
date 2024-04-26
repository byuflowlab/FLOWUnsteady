abstract type AbstractVehicle end

#--- Vortex Lattice Vehicle ---#

struct VLMVehicle{TF,Formulation,ViscousScheme,SubFilterScale,TKernel,TUJ,Tintegration}
    vlm_system::vlm.System{TF}
    wake_system::vpm.ParticleField{TF,Formulation,ViscousScheme,SubFilterScale,TKernel,TUJ,Tintegration}
end

function solve!(vehicle::VLMVehicle)
    nothing  
end
