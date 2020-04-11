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
                            pfield::vpm.ParticleField, dt::Real; t=0.0,
                            unsteady_shedcrit=-1.0, p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing)
    if t!=0
        VLM2VPM(self.wake_system, pfield, dt, Vinf; t=t,
                    prev_system=_get_prev_wake_system(self),
                    unsteady_shedcrit=unsteady_shedcrit,
                    p_per_step=p_per_step, sigmafactor=sigmafactor,
                    overwrite_sigma=overwrite_sigma, check=false)
    end
end


function generate_static_particle_fun(self::VLMVehicle, sigma::Real)

    if sigma<=0
        error("Invalid smoothing radius $sigma.")
    end

    function static_particles_function(args...)
        out = Array{Float64, 1}[]

        # Particles from vlm system
        _static_particles(self.vlm_system, sigma; out=out)

        # Particles from rotor systems
        for rotors in self.rotor_systems
            for rotor in rotors
                _static_particles(rotor, sigma; out=out)
            end
        end

        return out
    end

    return static_particles_function
end

save_vtk(self::VLMVehicle, args...;
                        optargs...) = save_vtk_base(self, args...; optargs...)

##### INTERNAL FUNCTIONS  ######################################################
function _static_particles(system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
                                    sigma::Real; out=Array{Float64, 1}[])

    # Adds a particle for every bound vortex of the VLM
    for i in 1:vlm.get_m(system)
        (Ap, A, B, Bp, _, _, _, Gamma) = vlm.getHorseshoe(system, i)
        for (i,(x1, x2)) in enumerate([(Ap,A), (A,B), (B,Bp)])
            push!(out, vcat((x1+x2)/2, Gamma*(x2-x1), sigma, 0))
        end
    end

    return out
end
##### END OF VEHICLE ###########################################################
