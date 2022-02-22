#=##############################################################################
# DESCRIPTION
    Vehicle with vortex-lattice method (VLM) and VPM-based propeller models
    shedding VPM wakes.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


################################################################################
# UNSTEADY VLM VEHICLE TYPE
################################################################################
"""
    `UVLMVehicle(system; optargs...)`

Type handling all geometries and subsystems that define a flight vehicle made
out of VLM (Wing, WingSystem, Rotor) components.

# ARGUMENTS
* `system::vlm.WingSystem`:        System of all FLOWVLM objects. This system
                                    is considered as the entire vehicle. Not all
                                    components in this system will be solved,
                                    but they will all be rotated and translated
                                    during maneuver.
# OPTIONAL ARGUMENTS
* `tilting_systems::Tuple(vlm.WingSystem, ...)`:   Tuple of all FLOWVLM
                                    tilting objects, where `tilting_systems[i]`
                                    contains the i-th FLOWVLM system of lifting
                                    surfaces and rotors that tilt together.
* `rotors_systems::Tuple(Array{vlm.Rotor,1}, ...)`:   Tuple of groups of Rotors
                                    that share a common RPM.
* `vlm_system::vlm.WingSystem`:    System of all FLOWVLM objects to be solved
                                    through the VLM solver.
* `wake_system::vlm.WingSystem`:   System of all FLOWVLM objects that will
                                    shed a VPM wake.
* `grids::Array{gt.GridTypes, 1}`: Array of grids that will be translated and
                                    rotated along with `system`.
"""
struct UVLMVehicle{N, M, R} <: AbstractVLMVehicle{N, M, R}

    # Required inputs
    system::vlm.WingSystem

    # Optional inputs
    tilting_systems::NTuple{N, vlm.WingSystem}
    rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}
    vlm_system::vlm.WingSystem
    wake_system::vlm.WingSystem
    grids::Array{gt.GridTypes, 1}

    # Internal properties
    V::Array{R, 1}                          # Current vehicle velocity
    W::Array{R, 1}                          # Current vehicle angular velocity
    prev_data::Array{Any, 1}                # Information about previous step
    grid_O::Array{Array{R, 1}, 1}           # Origin of every grid


    UVLMVehicle{N, M, R}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem(),
                    grids=Array{gt.GridTypes, 1}(),
                    V=zeros(3), W=zeros(3),
                    prev_data=[deepcopy(vlm_system), deepcopy(wake_system),
                                                    deepcopy(rotor_systems)],
                    grid_O=Array{Array{Float64, 1}, 1}(),
                ) where {N, M, R} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system,
                    grids,
                    V, W,
                    prev_data,
                    grid_O,
                )
end

# Implicit N and M constructor
UVLMVehicle(system::vlm.WingSystem;
        V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        grids=Array{gt.GridTypes, 1}(),
        optargs...
        ) where {N, M, R} = UVLMVehicle{N, M, R}( system;
                                V=V, W=W,
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                grids=Array{gt.GridTypes, 1}(grids),
                                grid_O=[zeros(R, 3) for i in 1:length(grids)],
                                optargs...)

# Alias
const VLMVehicle = UVLMVehicle

##### FUNCTIONS  ###############################################################
function shed_wake(self::VLMVehicle, Vinf::Function,
                            pfield::vpm.ParticleField, dt::Real, nt::Int; t=0.0,
                            unsteady_shedcrit=-1.0,
                            shed_starting=false,
                            p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing,
                            omit_shedding=[],
                            shed_boundarylayer=false,
                            prescribed_Cd=nothing, dipole_d=0.0,
                            )
    if nt!=0

        if shed_boundarylayer
            if dipole_d == 0
                error("Boundary layer shedding requested but no `d` was provided!")
            end

            # Shed only boundary layer (dragging line) particles
            VLM2VPM_draggingline(self.wake_system, _get_prev_wake_system(self),
                                    pfield, dt, Vinf,
                                    dipole_d;
                                    t=t,
                                    prescribed_Cd=prescribed_Cd,
                                    p_per_step=p_per_step,
                                    sigmafactor=sigmafactor,
                                    overwrite_sigma=overwrite_sigma,
                                )
        else
            # Shed trailing-circulation particles
            VLM2VPM(self.wake_system, pfield, dt, Vinf; t=t,
                        prev_system=_get_prev_wake_system(self),
                        unsteady_shedcrit=unsteady_shedcrit,
                        shed_starting=shed_starting && nt==1,
                        p_per_step=p_per_step, sigmafactor=sigmafactor,
                        overwrite_sigma=overwrite_sigma, check=false,
                        omit_shedding=omit_shedding)
        end
    end
end


function generate_static_particle_fun(pfield::vpm.ParticleField,
                                                self::VLMVehicle, sigma::Real;
                                                save_path=nothing, run_name="",
                                                suff="_staticpfield",
                                                ground_effect=true)

    if sigma<=0
        error("Invalid smoothing radius $sigma.")
    end

    flag = save_path!=nothing

    # Create auxiliary particle field used for saving vtks with static particles
    if flag
        maxparticles = vlm.get_m(self.vlm_system)
        for rotors in self.rotor_systems
            for rotor in rotors
                maxparticles += vlm.get_m(rotor)
            end
        end
        pfield_static = vpm.ParticleField(3*maxparticles; nt=0, t=0)
    end

    function static_particles_function(pfield, args...)

        # Particles from vlm system
        _static_particles(pfield, self.vlm_system, sigma)
        if flag; _static_particles(pfield_static, self.vlm_system, sigma); end;

        # Particles from rotor systems
        for rotors in self.rotor_systems
            for rotor in rotors
                _static_particles(pfield, rotor, sigma)
                if flag; _static_particles(pfield_static, rotor, sigma); end;
            end
        end

        altitude = self.altitude;

        #----------------Ground Effect Particles----------------------------------
        if ground_effect
            np = pfield.np;
            for i = 1:np
                X_i = pfield.particle[i].X;
                new_z = -2*(altitude + X_i[3]);        #Change z to flip over z_axis defined by altitude.

                X_i = [X_i[1], X_i[2], new_z];

                Γ_new = pfield.particle[i].Γ;
                Γ_new = [-Γ_new[1], -Γ_new[2], Γ_new[3]];       #Change direction of Γ
                
                pfield.add_particle(X_i, Γ_new, sigma); 
            end
        end

        ####
        pfield.np # number of particles
        pfield::vpm.ParticleField
        pfield.particles # array of ::Particle structs
        for i=1:pfield.np
            p_X = ...
            p_Gamma = ...
            p_sigma = ...
            vpm.add_particle(pfield, p_X, p_Gamma, p_sigma)
        end
        ####
        #------------------------------------------------------------

        # Save vtk with static particles
        if flag
            vpm.save(pfield_static, run_name*suff; path=save_path, add_num=true,
                                        overwrite_time=nothing)
            pfield_static.nt += 1
            pfield_static.t += 1
            for pi in vpm.get_np(pfield_static):-1:1
                vpm.remove_particle(pfield_static, pi)
            end
        end

        return nothing
    end

    return static_particles_function
end

save_vtk(self::VLMVehicle, args...;
                        optargs...) = save_vtk_base(self, args...; optargs...)

##### INTERNAL FUNCTIONS  ######################################################
function _static_particles(pfield::vpm.ParticleField,
                            system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
                            sigma::Real)

    # Adds a particle for every bound vortex of the VLM
    for i in 1:vlm.get_m(system)
        (Ap, A, B, Bp, _, _, _, Gamma) = vlm.getHorseshoe(system, i)
        for (i,(x1, x2)) in enumerate(((Ap,A), (A,B), (B,Bp)))
            vpm.add_particle(pfield, (x1+x2)/2, Gamma*(x2-x1), sigma;
                                    vol=0, circulation=abs(Gamma), static=true)
        end
    end
end
##### END OF VEHICLE ###########################################################
