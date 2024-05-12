#------- abstract types -------#

"""
    <:AbstractVehicle{Dynamic}

Vehicle type definition. All `<:AbstractVehicle` objects should contain the following fields:

* `state<:AbstractState`: representation of the high-level state(s) to be modeled and controlled
* `model<:AbstractModel`: lower level model to be used, e.g. a vortex lattice representation

"""
abstract type AbstractVehicle{Dynamic} end

"""
    kinematic_velocity(vehicle::AbstractVehicle)

Converts the kinematic velocity of the vehicle as defined by its `state` to an observed fluid velocity in its `model`.

"""
function kinematic_velocity!(vehicle::AbstractVehicle)
    kinematic_velocity!(vehicle.model, vehicle.state)
end

"""
    transform!(vehicle, dt)

Transform states of `vehicle.state` and `vehicle.model` according to state time derivatives using a forward Euler step of length `dt`. Second time derivative states such as `velocity_dot` and `angular_velocity_dot` are not modified.

# Arguments

* `vehicle::AbstractVehicle`: the vehicle to be transformed
* `dt::Number`: length of this timestep

"""
function transform!(vehicle::AbstractVehicle, dt)
    transform!(vehicle.model, vehicle.state, dt)
end

#------- RigidBodyVehicle -------#

"""
    RigidBodyVehicle <: AbstractVehicle

Vehicle defined by a system of rigid bodies and an `<:AbstractModel`.

# Fields

* `state::Array{<:RigidBodyState,0}`: recursive system of high-level rigid body states defining the vehicle
* `model::AbstractModel`: lower level model defining vehicle behavior

"""
struct RigidBodyVehicle{TF,TMap,TM,Dynamic} <: AbstractVehicle{Dynamic}
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
* `initial_state_args::NamedTuple`: keyword arguments used to construct the initial `DynamicState` of the vehicle
* `initial_state_derivative_args::NamedTuple`: keyword arguments used to construct the initial `DynamicState` time derivative of the vehicle

"""
function RigidBodyVehicle(model::AbstractModel{TF,<:Any};
        full_vehicle_state_name="full_vehicle",
        dynamic=true, # whether or not to include forces in state propagation between timesteps
        initial_state_args=(),
        initial_state_derivative_args=(),
    ) where TF

    # initial state
    initial_state = DynamicState(TF; initial_state_args...)

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
    return RigidBodyVehicle{TF,TM,typeof(model),dynamic}(state, model)
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

