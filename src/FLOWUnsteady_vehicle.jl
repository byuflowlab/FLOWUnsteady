#=##############################################################################
# DESCRIPTION
    Vehicle type handling all defined geometries and their properties.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

################################################################################
# ABSTRACT VEHICLE TYPE
################################################################################
"""
    AbstractVehicle{N, M, R}

Type handling all geometries and subsystems that define a flight vehicle.

`N` indicates the number of tilting systems in the vehicle, while and `M`
indicates the number of rotor systems. `R` is a Real type.

Implementations must have the following properties:
    * `V::Vector{R}`          : Current velocity vector of the vehicle in the
                                    global coordinate system.
    * `W::Vector{R}`          : Current angular velocity vector of the vehicle
                                    in the global coordinate system.
"""
abstract type AbstractVehicle{N, M, R} end

##### FUNCTIONS REQUIRED IN IMPLEMENTATIONS ####################################
"""
    `add_dV(self::AbstractVehicle, dV)`
Increments the velocity of the vehicle by `dV`.
"""
function add_dV(self::AbstractVehicle, dV)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `add_dW(self::AbstractVehicle, dV)`
Increments the angular velocity of the vehicle by `dW`.
"""
function add_dW(self::AbstractVehicle, dW)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `set_V(self::AbstractVehicle, V)`
Set current vehicle velocity to `V`.
"""
function set_V(self::AbstractVehicle, V)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `set_W(self::AbstractVehicle, W)`
Set current vehicle angular velocity to `W`.
"""
function set_W(self::AbstractVehicle, W)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `nextstep_kinematic(self::AbstractVehicle, angles)`
Tilts every tilting system of this vehicle into its corresponding new angle,
where `angles[i]=[Ax, Ay, Az]` is the new angle of the i-th tilting system (in
degrees).
"""
function tilt_systems(self::AbstractVehicle, angles)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `nextstep_kinematic(self::AbstractVehicle, dt::Real)`
Translates and rotates the vehicle in a time step `dt` according to current
linear and angular velocity.
"""
function nextstep_kinematic(self::AbstractVehicle, dt::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `precalculations(self, pfield, t, dt)`
Precalculations before calling the solver and before shedding trialing wake.
"""
function precalculations(self::AbstractVehicle, Vinf::Function,
                                pfield::vpm.ParticleField, t::Real, dt::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
Shed VPM wake. If `unsteady_shedcrit<=0` it sheds wake due to loading
distribution, otherwise it will shed wake due to unsteady loading.
"""
function shed_wake(self::AbstractVehicle, Vinf::Function,
                            pfield::vpm.ParticleField, dt::Real; t=0.0,
                            unsteady_shedcrit=-1.0, p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing,
                            omit_shedding=[])
    error("$(typeof(self)) has no implementation yet!")
end

# """
# Solves the aerodynamics of this vehicle.
# """
# function solve(self::AbstractVehicle, Vinf::Function, pfield::vpm.ParticleField,
#                 wake_coupled::Bool, vpm_solver::String,  t::Real, dt::Real,
#                                                       rlx::Real, sigma::Real)
#     error("$(typeof(self)) has no implementation yet!")
# end

"""
Returns a function that generates an array of particles representing the surface
of the vehicle as a collection of vortex particles.
"""
function generate_static_particle_fun(self::AbstractVehicle,
                                            sigma_vlm::Real, sigma_rotor::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    save_vtk(self::AbstractVehicle, prefix; path="", optargs...)

Output VTK files with vehicle geometry and solution fields.
"""
function save_vtk(self::AbstractVehicle, filename;
                        path=nothing, num=nothing, optargs...)
    error("$(typeof(self)) has no implementation yet!")
end

##### COMMON FUNCTIONS  ########################################################
"""
    get_ntltsys(self::AbstractVehicle{N, M, R})

Returns the number of tilting systems `N`.
"""
get_ntltsys(::AbstractVehicle{N}) where {N} = N

"""
    get_nrtrsys(self::AbstractVehicle{N, M, R})

Returns the number of rotor systems `M`.
"""
get_nrtrsys(::AbstractVehicle{N, M}) where {N, M} = M

##### COMMON INTERNAL FUNCTIONS  ###############################################
##### END OF ABSTRACT VEHICLE ##################################################








################################################################################
# SYSTEM TYPE
################################################################################

abstract type AbstractSystem{C, N} end

Base.length(system::AbstractSystem{C, N}) where {C, N} = N

function Base.iterate(system::AbstractSystem{C, N}) where {C, N}
    if N>0
        return (get_component(system, 1), 1)
    else
        return nothing
    end
end

function Base.iterate(system::AbstractSystem{C, N}, state) where {C, N}
    if state==N
        return nothing
    else
        return (get_component(system, state+1), state+1)
    end
end

"""
    applytobottom(f::Function, system::AbstractSystem)

Iterates over each component of `system` applying `f(component)` if the
component is not an `AbstractSystem` or applying recursion otherwise.
"""
function applytobottom(f::Function, system::AbstractSystem)
    for component in system
        if component isa AbstractSystem
            applytobottom(f, component)
        else
            f(component)
        end
    end
end

# Supported component types
const ComponentTypes = Union{vlm.Wing, vlm.WingSystem, vlm.Rotor,
                                pnl.AbstractBody, AbstractSystem}

"""
    System{C, N}(components::Vector{C<:ComponentTypes}, names::NTuple{N, String})

    System(components::Vector{C}, names::NTuple{N, String})

    System(components::Vector{C}, names::Vector{String})

    System(components::Vector{C})

    System()

A system of N components of type `C`, where `names[i]` is the name of the i-th
component.

Components can be of any of the following types (or a collection of them):
`C <: $(ComponentTypes)`. Notice that components can also be of the type
`System`, allowing the user to create a system made out of
subsystems.
"""
struct System{C<:ComponentTypes, N} <: AbstractSystem{C, N}

    # User inputs
    components::Vector{C}               # System components
    names::NTuple{N, String}            # Name of each component

    function System{C, N}(components, names) where {C, N}

        @assert length(components)==length(names) ""*
            "Number of components different than number of names"

        @assert length(unique(names))==length(names) ""*
            "Repeated names in $(names)"

        return new(components, names)
    end
end

function System(components::Vector{C}, names::NTuple{N, String}
                                                ) where {C, N}

    return System{C, N}(components, names)

end

function System(components::Vector{<:ComponentTypes}, names::Vector{String})

    return System(components, tuple(names...))

end

function System(components::Vector{Any}, args...; kwargs...)

    types = [typeof(c) for c in components]
    typedcomponents = Union{types...}[components...]

    return System(typedcomponents, args...; kwargs...)
end

System(components::Vector{<:ComponentTypes}) = System(components, ntuple(i->"$i", length(components)))

# Empty initializer
System(; C=System) = System(C[])

function get_component(system::System, componentname::String)
    componenti = findfirst(x->x==componentname, system.names)

    @assert componenti!=nothing ""*
        "Inexistent component $(componentname). Available components: $(system.names)"

    return get_component(system, componenti)
end

function get_component(system::System{C, N}, componenti::Int) where {C, N}

    @assert 0<componenti<=N ""*
        "Requested invalid component $(componenti). Must be >0 and <=$(N)."

    return system.components[componenti]
end

function generate_wake_system(; flowvlm_subsystem=System(),
                                flowpanel_subsystem=System())
    return System([flowvlm_subsystem, flowpanel_subsystem], ("flowvlm", "flowpanel"))
end
