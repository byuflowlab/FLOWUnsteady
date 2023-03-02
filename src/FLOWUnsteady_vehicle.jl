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
    add_dV(self::AbstractVehicle, dV)

Increments the velocity of the vehicle by `dV`.
"""
function add_dV(self::AbstractVehicle, dV)
    self.V .+= dV
    return nothing
end

"""
    add_dW(self::AbstractVehicle, dV)

Increments the angular velocity of the vehicle by `dW`.
"""
function add_dW(self::AbstractVehicle, dW)
    self.W .+= dW
    return nothing
end

"""
    set_V(self::AbstractVehicle, V)

Set current vehicle velocity to `V`.
"""
function set_V(self::AbstractVehicle, V)
    self.V .= V
    return nothing
end

"""
    set_W(self::AbstractVehicle, W)

Set current vehicle angular velocity to `W`.
"""
function set_W(self::AbstractVehicle, W)
    self.W .= W
    return nothing
end

"""
    tilt_systems(self::AbstractVehicle, angles)

Tilts every tilting system of this vehicle into its corresponding new angle,
where `angles[i]=[Ax, Ay, Az]` is the new angle of the i-th tilting system (in
degrees).
"""
function tilt_systems(  self::AbstractVehicle{N},
                        angles::NTuple{N, Array{<:Real, 1}}
                        ) where N

    for (si, system) in enumerate(self.tilting_systems)

        Oaxis = gt.rotation_matrix2([-a for a in angles[si]]...)

        vlm.setcoordsystem(sys, sys.O, Oaxis)
    end

    return nothing
end

# tilt_systems(self::AbstractVehicle{0}, angles::Tuple) = nothing

"""
    nextstep_kinematic(self::AbstractVehicle, dt::Real)

Translates and rotates the vehicle in a time step `dt` according to current
linear and angular velocity.
"""
function nextstep_kinematic(self::AbstractVehicle, dt::Real)
    dX = dt*self.V                  # Translation
    dA = 180/pi * dt*self.W         # Angular rotation (degrees)

    O = self.system.O + dX          # New origin of the system (translation)
    M = gt.rotation_matrix2([-a for a in dA]...) # Rotation matrix
    Oaxis = M*self.system.Oaxis     # New axes of the system (rotation)

    # Save current state of vehicle
    _update_previousstep(self, deepcopy(self))

    # Translation and rotation
    vlm.setcoordsystem(self.system, O, Oaxis)

    # Translation and rotation of every grid
    for i in 1:length(self.grids)
        self.grid_O[i] .+= dX

        # Translation
        gt.lintransform!(self.grids[i], Array(1.0I, 3, 3) , dX)

        # Brings the grid back to the global origin
        gt.lintransform!(self.grids[i], Array(1.0I, 3, 3) , -self.grid_O[i])

        # Rotation and brings the grid back to its position
        gt.lintransform!(self.grids[i], M, self.grid_O[i])
    end

    return nothing
end

"""
    precalculations(self::AbstractVehicle, pfield, t, dt, nt)

Precalculations before calling the solver and before shedding trailing wake.

Calculates the kinematic velocity and adds it as a solution field
"""
function precalculations(self::AbstractVehicle, Vinf::Function,
                            pfield::vpm.ParticleField,
                            t::Real, dt::Real, nt::Int)

    if nt!=0
        # ---------- 1) Recalculate wake horseshoes with kinematic velocity
        # Calculate kinematic velocity at every control point
        Vkin = _Vkinematic_wake(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.wake_system, "Vkin", Vkin; t=t)

        Vkin = _Vkinematic_vlm(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.vlm_system, "Vkin", Vkin; t=t)

        # Recalculate horseshoes
        vlm.getHorseshoes(self.vlm_system; t=t, extraVinf=_extraVinf1)
        vlm.getHorseshoes(self.wake_system; t=t, extraVinf=_extraVinf1)

        # ---------- 2) Paste previous Gamma solution ----------------------
        for i in 1:length(self.wake_system.wings)
            wing = vlm.get_wing(self.wake_system, i)
            prev_wing = vlm.get_wing(_get_prev_wake_system(self), i)

            if typeof(prev_wing) != vlm.Rotor
                sol = deepcopy(prev_wing.sol["Gamma"])
            else
                sol = deepcopy(prev_wing._wingsystem.sol["Gamma"])
            end

            vlm._addsolution(wing, "Gamma", sol; t=t)
        end
        for i in 1:length(self.vlm_system.wings)
            wing = vlm.get_wing(self.vlm_system, i)
            prev_wing = vlm.get_wing(_get_prev_vlm_system(self), i)

            if typeof(prev_wing) != vlm.Rotor
                sol = deepcopy(prev_wing.sol["Gamma"])
            else
                sol = deepcopy(prev_wing._wingsystem.sol["Gamma"])
            end

            vlm._addsolution(wing, "Gamma", sol; t=t)
        end
    end

    return nothing
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



function save_vtk_base(self::AbstractVehicle, filename; path=nothing,
                                   num=nothing, save_wopwopin=false,
                                   infinite_vortex=false, optargs...)
    strn = vlm.save(self.system, filename; path=path, num=num,
                                    infinite_vortex=infinite_vortex, optargs...)

    for (i, grid) in enumerate(self.grids)
        strn *= gt.save(grid, filename*"_Grid$i"; format="vtk", path=path, num=num)
    end

    # Generate inputs for PSU-WOPWOP
    if save_wopwopin
        for (si, rotors) in enumerate(self.rotor_systems)
            # Compact patch for loading
            generate_vtkliftinglines(rotors, filename*"_Sys$(si)", path; num=num,
                                                                suf="_compact")
            # Loft for thickness
            # NOTE: this is not needed since FLOWNoise can read the lofted VTK
            # straight up
            # for (ri, rotors) enumerate(rotors)
            #     vlm.save(rotor, filename*"_Sys$(si)_Rotor$(ri)"; path=path, addtiproot=true,
            #                             wopwop=true, wopbin=true, wopv=1.0, num=num)
            # end

            # Loading file
            save_gammas = [[vlm.get_blade(rotor, i).sol["Gamma"]
                                      for i in 1:rotor.B] for rotor in rotors]
            # NOTE TO SELF: Ftot is a force per unit length
            save_Ftot = [[vlm.get_blade(rotor, i).sol["Ftot"]
                                      for i in 1:rotor.B] for rotor in rotors]
            loadfname = "loading_Sys$(si)"*(num!=nothing ? ".$num" : "")*".jld"
            JLD.save(joinpath(path, loadfname), "rotorgammas", save_gammas,
                                                        "Ftot", save_Ftot)
        end
    end

    return strn
end

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

    # Internal properties
    O::Vector{Float64}                  # Origin w.r.t. global (3-dim vector)
    Oaxis::Matrix{Float64}              # Coordinate system w.r.t global (3x3 matrix)

    function System{C, N}(components, names;
                            O=zeros(3), Oaxis=Array(1.0I, 3, 3)) where {C, N}

        @assert length(components)==length(names) ""*
            "Number of components different than number of names"

        @assert length(unique(names))==length(names) ""*
            "Repeated names in $(names)"

        return new(components, names, O, Oaxis)
    end
end

function System(components::Vector{C}, names::NTuple{N, String}; kwargs...
                                                ) where {C<:ComponentTypes, N}

    return System{C, N}(components, names; kwargs...)

end

function System(components::Vector{<:ComponentTypes}, names::Vector{String};
                                                                    kwargs...)
    return System(components, tuple(names...); kwargs...)

end

function System(components::Vector{Any}, args...; kwargs...)

    types = [typeof(c) for c in components]
    typedcomponents = Union{types...}[components...]

    return System(typedcomponents, args...; kwargs...)
end

function System(components::Vector{<:ComponentTypes}; kwargs...)

    names = ntuple(i->"Component$i", length(components))

    return System(components, names; kwargs...)
end

# Empty initializer
System(; C=System, kwargs...) = System(C[]; kwargs...)

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


"""
    set_coordinatesystem(system::System, O::Vector, Oaxis::Matrix)

Redefines the local coordinate system of the system, where `O` is the new origin
and `Oaxis` is the matrix of new unit vectors.
"""
function set_coordinatesystem(system::System, O::AbstractVector, Oaxis::AbstractMatrix)
    for component in system
        set_coordinatesystem(component, O, Oaxis)
    end

    system.O .= O
    system.Oaxis .= Oaxis

    return nothing
end


function set_coordinatesystem(flowvlm::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor}, O, Oaxis)
    vlm.setcoordsystem(flowvlm, O, Oaxis)
    return nothing
end

function set_coordinatesystem(flowpanel::pnl.AbstractBody, O, Oaxis)
    pnl.set_coordinatesystem(flowpanel, O, Oaxis)
    return nothing
end
