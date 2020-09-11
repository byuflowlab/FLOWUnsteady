#=##############################################################################
# DESCRIPTION
    Vehicle with vortex-lattice method (VLM) and blade-element momentum (BEM)
    models assuming quasi-steady wakes. This means that at every time step the
    wake of VLM models are represented as rigid, semi-infinite filaments, and
    wakes of BEM models are obtained from a conservation of momentum
    assumptions. In this approach only VLM-on-VLM and VLM-on-BEM interactions
    are accounted for, meaning that the wake of each BEM rotor has no effect
    on other rotors and wings.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

################################################################################
# QUASI-STEADY VLM VEHICLE TYPE
################################################################################
"""
    `QVLMVehicle(system; optargs...)`

Type handling all geometries and subsystems that define a flight vehicle made
out of VLM (Wing, WingSystem, Rotor) components.

NOTE: For the solver to work correctly, all components in `wake_system` (if any)
need to be also components of `vlm_system`.

NOTE: It is recommended that `wake_system` doesn't include any Rotor object.
Otherwise, blades will generate a wake going straight out of every blade
trailing edge pointing oposite to the direction of rotation instead of
generating a streamtube.

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
                                    have a rigid, semi-infinite VLM wake.
* `grids::Array{gt.GridTypes, 1}`: Array of grids that will be translated and
                                    rotated along with `system`.
"""
struct QVLMVehicle{N, M, R} <: AbstractVLMVehicle{N, M, R}

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


    QVLMVehicle{N, M, R}(
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
QVLMVehicle(system::vlm.WingSystem;
        V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        grids=Array{gt.GridTypes, 1}(),
        optargs...
        ) where {N, M, R} = QVLMVehicle{N, M, R}( system;
                                V=V, W=W,
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                grids=Array{gt.GridTypes, 1}(grids),
                                grid_O=[zeros(R, 3) for i in 1:length(grids)],
                                optargs...)


##### FUNCTIONS  ###############################################################
function shed_wake(self::QVLMVehicle, Vinf::Function, pfield::vpm.AbstractParticleField,
                                                dt::Real, nt::Int; optargs...)
    nothing
end

function generate_static_particle_fun(pfield, self::QVLMVehicle, sigma)

    function static_particles_function(args...)
        return nothing
    end

    return static_particles_function
end


function save_vtk(self::QVLMVehicle, filename; path=nothing, num=nothing,
                                                save_wopwopin=false, optargs...)

    # Filter optional arguments for wake
    optargs1 = [(sym, val) for (sym, val) in optargs
                         if !(sym in [:infinite_vortex, :save_horseshoes,
                                      :only_horseshoes, :only_infinite_vortex])]

    # Save wake
    strn = vlm.save(self.wake_system, filename; suff="_wake",
                                infinite_vortex=true, save_horseshoes=true,
                                only_horseshoes=true, only_infinite_vortex=true,
                                path=path, num=num, optargs1...)

    # Filter optional arguments of VLM
    optargs2 = [(sym, val) for (sym, val) in optargs if sym!=:infinite_vortex]

    # Save VLM
    strn *= save_vtk_base(self, filename; path=path, num=num,
                                    save_wopwopin=save_wopwopin,
                                    infinite_vortex=false, optargs2...)
    return strn
end
##### INTERNAL FUNCTIONS  ######################################################

##### END OF VEHICLE ###########################################################
