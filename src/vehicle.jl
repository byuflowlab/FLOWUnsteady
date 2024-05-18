#------- abstract types -------#

"""
    <:AbstractVehicle{Dynamic, Frame}

Vehicle type definition. All `<:AbstractVehicle` objects should contain the following fields:

* `state<:AbstractState`: representation of the high-level state(s) to be modeled and controlled
* `model<:AbstractModel`: lower level model to be used, e.g. a vortex lattice representation

"""
abstract type AbstractVehicle{Dynamic, Frame} end

"""
    kinematic_velocity(vehicle::AbstractVehicle)

Converts the kinematic velocity of the vehicle as defined by its `state` to an observed fluid velocity in its `model`.

"""
function kinematic_velocity!(vehicle::AbstractVehicle)
    kinematic_velocity!(vehicle.model, vehicle.state)
end

"""
    apply!(vehicle, freestream)

Applies freestream properties to the `vehicle`.

# Arguments

* `vehicle::AbstractVehicle`: vehicle object that is to receive a freestream
* `freestream::SimpleFreestream`: defines the freestream properties

"""
function apply!(vehicle::AbstractVehicle, freestream::SimpleFreestream, current_time)
    apply!(vehicle.model, freestream)
end

"""
    solve!(vehicle, dt)

Solves `vehicle.model` for a duration of `dt`.

# Arguments

* `vehicle::AbstractVehicle`: the vehicle to be solved
* `dt::Float64`: the duration of this timestep (useful for determining wake shed locations)

"""
function solve!(vehicle::AbstractVehicle, dt)
    solve!(vehicle.model, dt)
end

"""
    transform!(vehicle, dt)

Transform states of `vehicle.state` and `vehicle.model` according to state time derivatives using a forward Euler step of length `dt`. Second time derivative states such as `velocity_dot` and `angular_velocity_dot` are not modified.

# Arguments

* `vehicle::AbstractVehicle`: the vehicle to be transformed
* `dt::Number`: length of this timestep

"""
function transform!(vehicle::AbstractVehicle, dt, i_step)
    # transform states and the corresponding models
    transform!(vehicle.model, vehicle.state, dt)

    # transform the rest of the model (e.g. the wake)
    transform!(vehicle.model, dt, i_step)
end

"""
    forces!(vehicle)

Apply forces computed in `vehicle.model` to `vehicle.state` by modifying the time derivatives.

# Arguments

* `vehicle::AbstractVehicle`: the vehicle whose forces are to be applied

"""
function forces!(vehicle::AbstractVehicle)
    forces!(vehicle.state, vehicle.model)
end

"""
    visualize(vehicle, i, t; name_prefix, path)

Writes output files for visualization of `vehicle` in Paraview.

# Arguments

* `vehicle::AbstractVehicle`: the vehicle to be visualized
* `i::Union{Nothing,Int}`: the timestep index
* `t::Union{Nothing,Float64}`: simulation time

# Keyword Arguments

* `name_prefix::String`: string to be prepended to the name of all output files
* `path::String`: path where output files should be stored

"""
function visualize(vehicle::AbstractVehicle, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default_", path="./")
    visualize(vehicle.state, i, t; name_prefix, path)
    visualize(vehicle.model, i, t; name_prefix, path)
end

function initialize_verbose(vehicle::AbstractVehicle, v_lvl)
    println("\t"^v_lvl, "model: \t", typeof(vehicle.model))
    println("\t"^v_lvl, "state: \t", typeof(vehicle.state))
end

function initialize_history(vehicle::AbstractVehicle, save_steps)
    state_history, state_fields = initialize_history(vehicle.state, save_steps)
    model_history, model_fields = initialize_history(vehicle.model, save_steps)

    return merge_histories(state_history, state_fields, model_history, model_fields)
end

function update_history!(history, vehicle::AbstractVehicle, i_step)
    update_history!(history, vehicle.state, i_step)
    update_history!(history, vehicle.model, i_step)
end


#------- Coordinate Systems -------#

abstract type AbstractCoordinateSystem end

"""
    Aerodynamics <: AbstractCoordinateSystem

Standard aerodynamics coordinate system "back-right-up".

"""
struct Aerodynamics <: AbstractCoordinateSystem end

"""
    FlightDynamics <: AbstractCoordinateSystem

Standard flight dynamics coordinate system "front-right-down".

"""
struct FlightDynamics <: AbstractCoordinateSystem end

"""
    rotate!(model, old_coordinates, new_coordinates)

Rotates an `<:AbstractModel` from the `old_coordinates` reference frame to the `new_coordinates` reference frame. Convenience function for constructing an `<:AbstractVehicle` using a model constructed in a different frame than desired.

# Arguments

* `model::AbstractModel`: the model to be transformed into the new coordinates
* `old_coordinates::AbstractCoordinateSystem`: the coordinates used during construction of `model`
* `new_coordinates::AbstractCoordinateSystem`: the coordinates desired

"""
rotate!

function rotate!(model, old_coordinates::Aerodynamics, new_coordinates::FlightDynamics)
    q = Quaternion(SVector{3}(0,1.0,0), pi)
    transform!(model, map_all(model), zero(SVector{3,Float64}), q, zero(SVector{3,Float64}))
end

function rotate!(model, old_coordinates::FlightDynamics, new_coordinates::Aerodynamics)
    q = Quaternion(SVector{3}(0,1.0,0), pi)
    transform!(model, map_all(model), zero(SVector{3,Float64}), q, zero(SVector{3,Float64}))
end

function rotate!(model, old_coordinates::TC, new_coordinates::TC) where TC
    return nothing # no rotation required if coordinates are the same
end

function rotate(v, old_coordinates::Aerodynamics, new_coordinates::FlightDynamics)
    q = Quaternion(SVector{3}(0,1.0,0), pi)
    return rotate(v, q)
end

function rotate(v, old_coordinates::FlightDynamics, new_coordinates::Aerodynamics)
    q = Quaternion(SVector{3}(0,1.0,0), pi)
    return rotate(v, q)
end

function rotate(v, old_coordinates::TC, new_coordinates::TC) where TC
    return v
end

#------- RigidBodyVehicle -------#

"""
    RigidBodyVehicle <: AbstractVehicle

Vehicle defined by a system of rigid bodies and an `<:AbstractModel`.

# Fields

* `state::Array{<:RigidBodyState,0}`: recursive system of high-level rigid body states defining the vehicle
* `model::AbstractModel`: lower level model defining vehicle behavior

"""
struct RigidBodyVehicle{TF,TMap,TM,Dynamic,Frame} <: AbstractVehicle{Dynamic,Frame}
    state::Array{RigidBodyState{TF,TMap},0}
    model::TM
end

"""
    RigidBodyVehicle(model; kwargs...)

Constructor for `RigidBodyVehicle`.

# Arguments

* `model::AbstractModel`: the model to be used by the vehicle

# Keyword Arguments

* `dynamic::Bool=true`: whether or not 6-DOF dynamic equations of motion should be used to calculate state rates during simulation
* `frame::AbstractFrame`: the reference frame convention used by the vehicle
* `initial_state_args::NamedTuple`: keyword arguments used to construct the initial `DynamicState` of the vehicle; the `model` will be transformed to the initial state during construction of the vehicle
* `initial_state_derivative_args::NamedTuple`: keyword arguments used to construct the initial `DynamicState` time derivative of the vehicle
* `vehicle_coordinates::AbstractCoordinateSystem=Aerodynamic()`: the coordinate system adopted for this `AbstractVehicle`
* `model_coordinates::AbstractCoordinateSystem=Aerodynamic()`: the coordinate system adopted when creating `model`
* `model_origin::SVector{3,Float64}`: the location of the origin used during construction of the model expressed in the vehicle frame

"""
function RigidBodyVehicle(model::AbstractModel{TF,<:Any};
        full_vehicle_state_name="full_vehicle",
        dynamic=true, # whether or not to include forces in state propagation between timesteps
        # initial state
        position=SVector{3,Float64}(0.0,0,0),
        velocity=SVector{3,Float64}(1.0,0,0),
        orientation=zero(Quaternion{Float64}),
        angular_velocity=zero(SVector{3,Float64}),
        mass=1.0,
        inertia=SMatrix{3,3,Float64,9}(1.0,0,0,0,1.0,0,0,0,1.0),
        initial_state_derivative_args=(),
        # desired vehicle coordinates
        vehicle_coordinates=Aerodynamics(),
        # coordinates used during model definition
        model_coordinates=Aerodynamics(),
        model_origin=zero(SVector{3,Float64})
    ) where TF

    # rotate model into new coordinate system
    rotate!(model, model_coordinates, vehicle_coordinates)
    model_origin = rotate(model_origin, model_coordinates, vehicle_coordinates)

    # initial state
    initial_state = DynamicState(TF; position, velocity, orientation, angular_velocity, mass, inertia)

    # transform model to initial orientation
    q = initial_state.orientation
    Δx = initial_state.position - model_origin
    transform!(model, map_all(model), Δx, q, zero(SVector{3,Float64}))

    # initial state derivative
    initial_state_derivative = DynamicStateDerivative(TF; initial_state_derivative_args...)

    # state map
    full_vehicle_map = map_all(model)
    TM = typeof(full_vehicle_map)

    # empty vector of substates
    substates = RigidBodyState{TF,TM}[]

    # full vehicle state
    state = Array{RigidBodyState{TF,TM},0}(undef)
    state[] = RigidBodyState{TF,TM}(full_vehicle_state_name, full_vehicle_map, initial_state, initial_state_derivative, substates)

    # construct vehicle object
    return RigidBodyVehicle{TF,TM,typeof(model),dynamic,vehicle_coordinates}(state, model)
end

"""
    add_substate!(vehicle, parent_state_index, new_state_name, model_map, dynamic_state, dynamic_state_derivative)

Adds a substate to the state indicated by `parent_state_index`.

# Arguments

* `vehicle::RigidBodyVehicle{TF,TM,<:Bool}`: vehicle to which a new substate is to be added
* `parent_state_index::NTuple{N,TF}`: index of the recursive `vehicle.state` which is to receive a new substate; e.g., `parent_state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the 1st substate of `vehicle.state[]`, while `state_index=()` indicates `vehicle.state[]` itself
* `new_state_name::String`: name of the new substate
* `model_map::TM`: designates which part of `vehicle.model` the new substate refers to

# Optional Arguments

* `state_args::NamedTuple`: keyword arguments used to construct the initial `DynamicState` of the new substate
* `state_derivative_args::NamedTuple`: keyword arguments used to construct the initial time derivative of the `DynamicState` of the new substate
"""
function add_substate!(vehicle::RigidBodyVehicle{TF,TM,<:Bool}, parent_state_index::NTuple{N,Int64}, new_state_name, model_map::TM;
        state_args=(), state_derivative_args=()
    ) where {TF,TM,N}

    parent_state = get_substate(vehicle.state[], parent_state_index)
    push!(parent_state.substates, RigidBodyState{TF,TM}(
                new_state_name,
                model_map,
                DynamicState(TF; state_args...),
                DynamicState(TF; state_derivative_args...)
            )
        )
end

