abstract type AbstractState end

function Base.isnan(state::AbstractState)
    @error "isnan not defined for $(typeof(state))"
end

function reset!(state::AbstractState)
    @error "reset! not defined for $(typeof(state))"
end

function kinematic_velocity!(model::AbstractModel, state::AbstractState)
    @error "kinematic_velocity! not yet defined for $(typeof(state))"
end

function solve!(model::AbstractModel, state::AbstractState, dt)
    @error "solve! not defined for $(typeof(state))"
end

function transform!(model::AbstractModel, state::AbstractState, dt)
    @error "transform! not yet defined for $(typeof(state))"
end

function forces!(model::AbstractModel, state::AbstractState, dynamic::Bool)
    @error "forces! not yet defined for $(typeof(state))"
end

function initialize_history(state::AbstractState, save_steps)
    @error "initialize_history not defined for $(typeof(state))"
end

function (initializer::DefaultInitializer)(state::AbstractState, time, i_step)
    @error "initializer::DefaultInitializer not defined for $(typeof(state))"
end

#------- RigidBodyState -------#

"""
    RigidBodyState

Recursive structure of dynamic states defining a system of rigid bodies and their mapping to model states.

# Fields

* `name::String`: name of this state
* `state::DynamicState{<:Number}`: position, orientation, their derivatives, mass, and inertia properties of this state
* `state_derivative::DynamicStateDerivative{<:Number}`: derivative of the `state` properties
* `map::TM`: map from this state to all corresponding `model<:AbstractModel` states (e.g., all vortex lattice surfaces corresponding to a rotor defined by this state)
* `substates::Vector{<:RigidBodyState}`: vector of substates expressed in the frame of this state; therefore, `substates` implicitly inherit any changes to this state

"""
struct RigidBodyState{TF,TM} <: AbstractState
    name::String
    map::TM
    state::DynamicState{TF}
    state_derivative::DynamicStateDerivative{TF}
    substates::Vector{RigidBodyState{TF,TM}}
end

function Base.zero(::Type{RigidBodyState{TF,TM}}, name="", map=Int[]) where {TF,TM}
    @assert typeof(map) <: TM "requested map doesn't match requested map type"
    state = zero(DynamicState{TF})
    state_derivative = zero(DynamicStateDerivative{TF})
    substates = Vector{RigidBodyState{TF,TM}}(undef,0)
    return RigidBodyState{TF,TM}(name, map, state, state_derivative, substates)
end

function Base.isnan(state::Array{<:RigidBodyState,0})
    return isnan(state[])
end

function Base.isnan(state::RigidBodyState)
    flag = false
    for substate in state.substates
        if isnan(substate)
            flag = true
            throw("nan in RigidBodyState")
        end
    end
    return flag || isnan(state.state) || isnan(state.state_derivative)
end

"""
    add_substate!(state, substate_index, substate)

Adds a `RigidBodyState` to the substates of the `RigidBodyState` indicated by `substate_index`.

# Arguments

* `state::RigidBodyState`: the state to be updated
* `substate_index::NTuple{N,Int64}`: the `i` th member of `substate_index` designates which `::RigidBodyState` to which a new substate will be added; e.g., `substate_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of 1st substate of the input `state`, while `substate_index=()` indicates the top level `state`

"""
function add_substate!(state::RigidBodyState{TF,TM}, substate_index, substate::RigidBodyState{TF,TM}) where {TF,TM}
    push!(get_substate(state, substate_index).substates, substate)
    return nothing
end

"""
    add_state!(state, state_index, substate)

Adds a `RigidBodyState` to the substates of the `RigidBodyState` indicated by `state_index`.

# Arguments

* `state::Array{RigidBodyState,0}`: 0-dimensional array containing the state to be updated
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` to which a new substate will be added; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state`, while `state_index=(1,)` indicates the input `state[1]`

"""
function add_substate!(state::Array{<:RigidBodyState,0}, state_index::NTuple{N,Int64}, substate) where N
    add_substate!(state[], state_index[2:end], substate)
end

"""
    RigidBodyState(name, map, state, state_derivative, substates)

Constructor for `RigidBodyState` objects.

# Arguments

* `name::AbstractString`: name of the state
* `map::TM`: map to the `<:AbstractModel`
* `state::DynamicState{TF}`: the dynamic state properties

# Optional Arguments

* `state_derivative::DynamicStateDerivative{TF} = DynamicStateDerivative(TF)`: time derivative of the state (default is zeros)
* `substates::Vector{RigidBodyState{TF,TM}} = RigidBodyState{TF,TM}[]`: vector of substates (default is an empty vector)

"""
function RigidBodyState(name, map::TM, state::DynamicState{TF}, state_derivative::DynamicStateDerivative{TF}=DynamicStateDerivative(TF), substates=RigidBodyState{TF,TM}[]) where {TF,TM}
    return RigidBodyState(name, map, state, state_derivative, substates)
end

"""
    get_substate(state, substate_index)

Recursively navigates the `RigidBodyState` substates to return the state designated by `substate_index`.

# Arguments

* `state::RigidBodyState`: recursive rigid body state system to be accessed
* `substate_index::NTuple{N,Int64}`: the `i` th member of `substate_index` designates which `::RigidBodyState` to access; e.g., `substate_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the 1st substate of the top level `state`, while `substate_index=()` indicates the top level `state`

# Returns

* `substate::RigidBodyState`: the desired substate

"""
function get_substate(state::RigidBodyState, substate_index::NTuple{N,Int64}) where N
    if N > 0
        return get_substate(state.substates[substate_index[1]], substate_index[2:end])
    elseif N==0
        return state
    end
end


"""
    get_state(state, state_index)

Recursively navigates the `RigidBodyState` substates to return the state designated by `state_index`.

# Arguments

* `state::Array{<:RigidBodyState,0}`: recursive rigid body state system to be accessed
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` to access; e.g., `substate_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `substate_index=(1,)` indicates the top level `state[1]`

# Returns

* `substate::RigidBodyState`: the desired substate

"""
function get_state(state::Array{<:RigidBodyState,0}, state_index::NTuple{N,Int64}) where N
    @assert N > 0 "invalid state_index"
    return get_substate(state[], state_index[2:end])
end

"""
    set_substate!(state, substate_index, new_state)

Set a particular substate equal to `new_state`.

# Arguments

* `state::RigidBodyState`: the state whose substate is to be set
* `substate_index::NTuple{N,Int64}`: the `i` th member of `substate_index` designates which `::RigidBodyState` to set; e.g., `substate_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the 1st substate of the top level `state`, while `substate_index=()` indicates the top level `state`

"""
function set_substate!(state::RigidBodyState, substate_index::NTuple{N,Int64}, new_state::RigidBodyState) where N
    @assert N > 0 "invalid substate_index"

    if N > 1
        return set_substate!(state.substates[substate_index[1]], substate_index[2:end], new_state)
    else
        state.substates[substate_index[1]] = new_state
    end
end

"""
    set_state!(state, state_index, new_state)

Instance of `set_substate!` overloaded to accept a 0-dimension array containing a `::RigidBodyState` (in case the top level state needs to be set).

# Arguments

* `state::Array{<:RigidBodyState,0}`: the state whose substate is to be set
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` to set; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `substate_index=(1,)` indicates the top level `state[1]`

"""
function set_state!(state::Array{<:RigidBodyState,0}, state_index::NTuple{N,Int64}, new_state::RigidBodyState) where N
    if N > 1
        set_substate!(state[], state_index[2:end], new_state)
    else
        state[state_index[1]] = new_state
    end
end

"""
    set_dynamic_state_derivative!(state, state_index)

Sets the dynamic state derivative of the indicated state.

# Arguments

* `state::Array{<:RigidBodyState}`: the `::RigidBodyState` whose dynamic state derivative is to be set; a 0-dimensional array is required in case the top level state derivative should be set
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which subframe's dynamic state derivative to set; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `state_index=(1,)` indicates the top level `state[1]`
* `new_dynamic_state_derivative::DynamicStateDerivative{TF}`: the new state derivative

"""
function set_dynamic_state_derivative!(state::Array{<:RigidBodyState{TF,TM},0}, state_index::NTuple{N,Int64}, new_dynamic_state_derivative::DynamicStateDerivative{TF}) where {TF,TM,N}
    @assert N > 0 "invalid substate_index"

    current_state = get_state(state, state_index)
    new_state = RigidBodyState{TF,TM}(
                current_state.name,
                current_state.map,
                current_state.state,
                new_dynamic_state_derivative,
                current_state.substates
            )
    set_state!(state, state_index, new_state)
end

"""
    reset_dynamic_state_derivative!(state, state_index)

Zeros the dynamic state derivative of the indicated state.

# Arguments

* `state::Array{<:RigidBodyState,0}`: the `::RigidBodyState` whose dynamic state derivative is to be reset; a 0-dimensional array is required in case the top level state derivative should be reset
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which subframe's dynamic state derivative to reset; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `substate_index=(1,)` indicates the input `state[1]`

"""
function reset_dynamic_state_derivative!(state::Array{RigidBodyState{TF,<:Any},0}, state_index) where TF
    new_dynamic_state_derivative = DynamicStateDerivative(TF)
    set_dynamic_state_derivative!(state, state_index, new_dynamic_state_derivative)
end

"""
    set_dynamic_state!(state, state_index, new_dynamic_state)

Sets the dynamic state of the indicated state. Note that during a simulation, positional states (e.g. position and orientation) should NEVER be modified directly; rather, their first and second derivatives should be modified.

# Arguments

* `state::Array{<:RigidBodyState}`: the `::RigidBodyState` whose dynamic state is to be set; a 0-dimensional array is required in case the top level state should be set
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` dynamic state to set; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, and `substate_index=(1,)` indicates the input `state[1]`
* `new_dynamic_state::DynamicState{TF}`: the new state

"""
function set_dynamic_state!(state::Array{RigidBodyState{TF,TM},0}, state_index::NTuple{N,Int64}, new_dynamic_state::DynamicState) where {TF,TM,N}
    @assert N > 0 "invalid state_index"

    # get current state
    current_state = get_state(state, state_index)

    # set state
    new_state = RigidBodyState{TF,TM}(
                current_state.name,
                current_state.map,
                new_dynamic_state,
                current_state.state_derivative,
                current_state.substates
            )

    set_state!(state, state_index, new_state)
end

"""
    set_position!(state, state_index, new_position, dt)

Sets the position of the indicated state without modifying the dynamic state directly; rather, velocity is modified such that the desired position is achieved over a timestep of `dt`. This function is typically only used on substates (since the top level state derivatives will change due to dynamics) or with `<:AbstractVehicle{false}` vehicles, i.e. when dynamics are not simulated. Note that the time derivative of the velocity is set to 0 as a side effect of this function.

# Arguments

* `state::Array{<:RigidBodyState}`: the `::RigidBodyState` whose dynamic state is to be reset; a 0-dimensional array is required in case the top level state should be set
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` to update; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `state_index=(1,)` indicates the input `state[1]`
* `new_position::SVector{3,TF}`: the desired position at the end of this timestep
* `dt::Float64`: the timestep over which the desired dynamic state will be reached

"""
function set_position!(state::Array{RigidBodyState{TF,TM},0}, state_index::NTuple{N,Int64}, new_position::AbstractVector, dt) where {TF,TM,N}

    # get current state
    @assert N > 0 "invalid state_index"
    current_state = get_state(state, state_index)
    current_dynamic_state = current_state.state
    current_dynamic_state_derivative = current_state.state_derivative

    # calculate required derivative
    required_velocity = (new_position - current_dynamic_state.position) / dt

    # new dynamic state
    updated_dynamic_state = DynamicState{TF}(
            current_dynamic_state.position,
            required_velocity,
            current_dynamic_state.orientation,
            current_dynamic_state.angular_velocity,
            current_dynamic_state.mass,
            current_dynamic_state.inertia
        )

    # new dynamic state derivative (set velocity_dot=0)
    updated_dynamic_state_derivative = DynamicStateDerivative{TF}(
            zero(SVector{3,TF}),
            current_dynamic_state_derivative.angular_velocity_dot,
            current_dynamic_state_derivative.mass_dot,
            current_dynamic_state_derivative.inertia_dot
        )

    # update state
    new_state = RigidBodyState{TF,TM}(
            current_state.name,
            current_state.map,
            updated_dynamic_state,
            updated_dynamic_state_derivative,
            current_state.substates
        )
    set_state!(state, state_index, new_state)
end

"""
    set_orientation!(state, state_index, new_orientation, dt)

Sets the orientation of the indicated state without modifying the dynamic state directly; rather, angular velocity is modified such that the desired orientation is achieved over a timestep of `dt`. This function is typically only used on substates (since the top level state derivatives will change due to dynamics) or with `<:AbstractVehicle{false}` vehicles (i.e. when dynamics are not simulated). Note that the time derivative of the angular velocity is set to 0 as a side effect of this function.

# Arguments

* `state::Array{<:RigidBodyState}`: the `::RigidBodyState` whose dynamic state is to be reset; a 0-dimensional array is required in case the top level state should be set
* `state_index::NTuple{N,Int64}`: the `i` th member of `state_index` designates which `::RigidBodyState` to update; e.g., `state_index=(1,3,2)` indicates the 2nd substate of the 3rd substate of the input `state[1]`, while `substate_index=(1,)` indicates the input `state[1]`
* `new_orientation::SVector{3,TF}`: the desired orientation at the end of this timestep
* `dt::Float64`: the timestep over which the desired orientation will be reached

"""
function set_orientation!(state::Array{RigidBodyState{TF,TM},0}, state_index::NTuple{N,Int64}, new_orientation::Quaternion, dt) where {TF,TM,N}

    # get current state
    @assert N > 0 "invalid state_index"
    current_state = get_state(state, state_index)
    current_dynamic_state = current_state.state
    current_dynamic_state_derivative = current_state.state_derivative

    # calculate required derivative (see `subjects/quaternions.md` in my notebook)
    Δq = new_orientation * inv(current_state.orientation) # rotation from current orientation to desired
    θ = 2 * acos(Δq.real) # angle of required rotation
    axis = Δq / sin(θ/2) # axis of required rotation
    angular_velocity_required = θ/dt * axis

    # new dynamic state
    updated_dynamic_state = DynamicState{TF}(
            current_dynamic_state.position,
            current_dynamic_state.velocity,
            current_dynamic_state.orientation,
            angular_velocity_required,
            current_dynamic_state.mass,
            current_dynamic_state.inertia
        )

    # new dynamic state derivative (set angular_velocity_dot=0)
    updated_dynamic_state_derivative = DynamicStateDerivative{TF}(
            current_dynamic_state_derivative.velocity_dot,
            zero(SVector{3,TF}),
            current_dynamic_state_derivative.mass_dot,
            current_dynamic_state_derivative.inertia_dot
        )

    # update state
    new_state = RigidBodyState{TF,TM}(
            current_state.name,
            current_state.map,
            updated_dynamic_state,
            updated_dynamic_state_derivative,
            current_state.substates
        )
    set_state!(state, state_index, new_state)
end

"""
    quaternion_frame_2_top(state, substate_index)

Returns the quaternion representing the rotation of reference frame from the desired substate frame to the parent frame of `state`. Note that this quaternion does NOT represent the rotation of a vector about an axis by an angle, as the order of quaternion composition would change.

# Arguments

* `state::RigidBodyState`: the state into whose parent frame of reference we desire to rotate
* `substate_index::NTuple{N,Int64}`: the index of the substate from whose frame we desire to rotate; e.g., `substate_index = (1,)` indicates the first subframe of `state`, `substate_index = (1,2)` indicates the second substate of the first subframe of `state`, and `substate_index = ()` indicates the parent frame of `state[]`

"""
function quaternion_frame_2_top(state::RigidBodyState{TF,TM}, substate_index::NTuple{N,Int64}) where {TF,TM,N}
    if N > 0
        q2 = state.state.orientation
        q1 = quaternion_frame_2_top(state.substates[substate_index[1]], substate_index[2:end])
        return q1 * q2
    else
        return state.state.orientation
    end
end

function quaternion_frame_2_top(state::Array{RigidBodyState{TF,TM},0}, state_index::NTuple{N,Int64}) where {TF,TM,N}
    if N == 0
        return zero(Quaternion{TF})
    else
        return quaternion_frame_2_top(state[], state_index[2:end])
    end
end

"""
    transform_parent_2_top(point, state, substate_index)

Transforms a `point` expressed in the parent frame of `state` or one of its substates into the parent frame of `state`.

# Arguments

* `state::RigidBodyState`: the state into whose parent frame of reference we desire to rotate
* `substate_index::NTuple{N,Int64}`: the index of the substate in whose parent frame `point` is expressed; e.g., `substate_index = (1,)` indicates the frame of `state`, `substate_index = (2,1)` indicates the second substate of the input `state`, and `substate_index = ()` indicates the parent frame of the input `state`
* `point::AbstractVector`: the point expressed in the reference frame of the subframe indicated by `subframe_index`

# Returns

* `point::AbstractVector`: the point transformed into the top level reference frame

"""
function transform_parent_2_top(point, state::RigidBodyState, substate_index::NTuple{N,Int64}) where N
    if N == 0 # already in the top frame
        return point
    elseif N > 1
        return rotate_frame(transform_parent_2_top(point, state.substates[substate_index[1]], substate_index[2:end]), state.state.orientation) + state.state.position
    else
        return rotate_frame(point, state.state.orientation) + state.state.position
    end
end

function transform_2_top(point, state::Array{<:RigidBodyState,0}, state_index::NTuple{N,Int64}) where N
    if N == 0 # already in top frame
        return point
    else
        return transform_parent_2_top(point, state[], (state_index[2:end]..., 1))
    end
end

"""
    apply_force!(state, substate_index, force, location)

Apply a `force` vector at the specified `location` in the frame of the specified `substate`. This is done by modifying the top level `state` derivative in-place to reflect the addition of a force to a rigid body. Note that this function does not reset the state derivative, meaning its effect is cumulative. It is further assumed that `state` is expressed in an inertial frame, and that `state.state_derivative.inertia` has been updated.

# Arguments

* `state::Array{<:RigidBodyState,0}`: the state to which a force is to be applied
* `state_index::NTuple{N,Int64}`: the index of the substate in which the `force` vector and its `location` are expressed; e.g., `substate_index = (1,)` indicates the frame of `state[]`, `substate_index = (1,2)` indicates the second substate of the `state[]` frame, and `state_index = ()` indicates the inertial frame
* `force::AbstractVector`: the force vector to be applied
* `location::AbstractVector`: the location in `state` frame coordinates at which the force is to be applied

"""
function apply_force!(state::Array{<:RigidBodyState,0}, state_index::NTuple{<:Any,Int64}, force::AbstractVector, location::AbstractVector)

    # transform into inertial frame
    location_inertial_frame = transform_parent_2_top(location, state[], state_index)
    force_inertial_frame = rotate_frame(force, quaternion_frame_2_top(state[], state_index))

    # apply induced moment
    moment_inertial_frame = cross(location_inertial_frame, force_inertial_frame)
    apply_moment!(state, (), moment_inertial_frame)

    # express force in terms of dynamic state derivatives
    velocity_dot = force_inertial_frame / state[].state.mass

    # form new state derivative
    new_dynamic_state_derivative = DynamicStateDerivative(
            state[].state_derivative.velocity_dot + velocity_dot,
            state[].state_derivative.angular_velocity_dot,
            state[].state_derivative.mass_dot,
            state[].state_derivative.inertia_dot
        )

    # update top level state derivative
    state[] = RigidBodyState(
            state[].name,
            state[].map,
            state[].state,
            new_dynamic_state_derivative,
            state[].substates
        )

    return nothing
end

"""
    apply_moment!(state, state_index, moment)

Apply a `moment` vector in the frame of the specified `substate`. This is done by modifying the top level `state` derivative in-place to reflect the addition of a moment to a rigid body. Note that this function does not reset the state derivative, meaning its effect is cumulative. It is further assumed that `state` is expressed in an inertial frame, and that `state.state_derivative.inertia` has been updated.

# Arguments

* `state::Array{<:RigidBodyState,0}`: the state to which a force is to be applied
* `state_index::NTuple{N,Int64}`: the index of the substate in which the moment is expressed; e.g., `state_index = (1,)` indicates the frame of `state[]`, `state_index = (1,2)` indicates the second substate of the `state[]` frame, and `state_index = ()` indicates the inertial frame
* `moment::AbstractVector`: the moment vector to be applied expressed in the indicated frame coordinates

"""
function apply_moment!(state::Array{<:RigidBodyState,0}, state_index::NTuple{N,Int64}, moment::AbstractVector) where N

    # transform into inertial frame (if not already there)
    N > 0 && ( moment = rotate_frame(moment, quaternion_frame_2_top(state[], state_index)) )

    # rotate into full vehicle body frame
    moment_body_frame = rotate(moment, state[].state.orientation)

    # express moment in terms of dynamic state derivatives
    angular_velocity_body_frame = rotate(state[].state.angular_velocity, state[].state.orientation)
    angular_velocity_dot_body_frame = state[].state.inertia \ (moment_body_frame -
            state[].state_derivative.inertia_dot * angular_velocity_body_frame -
            cross(angular_velocity_body_frame, state[].state.inertia * angular_velocity_body_frame)
        )
    angular_velocity_dot_inertial_frame = rotate_frame(angular_velocity_dot_body_frame, state[].state.orientation)

    # form new state derivative
    new_dynamic_state_derivative = DynamicStateDerivative(
            state[].state_derivative.velocity_dot,
            state[].state_derivative.angular_velocity_dot + angular_velocity_dot_inertial_frame,
            state[].state_derivative.mass_dot,
            state[].state_derivative.inertia_dot
        )

    # update top level state derivative
    state[] = RigidBodyState(
            state[].name,
            state[].map,
            state[].state,
            new_dynamic_state_derivative,
            state[].substates
        )

    return nothing
end

function reset!(state::Array{RigidBodyState{TF,TM},0}) where {TF,TM}

    # form new state derivative
    new_dynamic_state_derivative = DynamicStateDerivative(
            zero(SVector{3,TF}),
            zero(SVector{3,TF}),
            state[].state_derivative.mass_dot,
            state[].state_derivative.inertia_dot
        )

    # update top level state derivative
    state[] = RigidBodyState(
            state[].name,
            state[].map,
            state[].state,
            new_dynamic_state_derivative,
            state[].substates
        )

end

"""
    kinematic_velocity!(model, state)

Dispatch of `kinematic_velocity!` for a `<:RigidBodyState`.

# Arguments

* `model::AbstractModel`: the model receiving the kinematic velocity
* `state::RigidBodyState`: instance of `RigidBodyState` describing the `model`

"""
function kinematic_velocity!(model::AbstractModel, state::Array{<:RigidBodyState,0})
    kinematic_velocity!(model, state[], ())
end

function kinematic_velocity!(model::AbstractModel, state::RigidBodyState, substate_index::NTuple{<:Any,Int64})

    # calculate the kinematic velocity
    this_substate = get_substate(state, substate_index)
    velocity = this_substate.state.velocity
    angular_velocity = this_substate.state.angular_velocity
    center_of_rotation = this_substate.state.position

    # rotate/transform into the global frame
    q = quaternion_frame_2_top(state, substate_index[1:end-1])
    velocity = rotate_frame(velocity, q)
    angular_velocity = rotate_frame(angular_velocity, q)
    center_of_rotation = transform_parent_2_top(center_of_rotation, state, substate_index)

    # apply to corresponding parts of the model
    for model_map in state.map
        kinematic_velocity!(model, model_map, velocity, angular_velocity, center_of_rotation)
    end

    # recurse over substate
    for (i,substate) in enumerate(state.substates)
        new_substate_index = (substate_index..., i)
        kinematic_velocity!(model, state, new_substate_index)
    end

end

function solve!(model::AbstractModel, state::Array{<:RigidBodyState,0}, dt)
    solve!(state, dt)
end

function transform!(model::AbstractModel, state::Array{<:RigidBodyState,0}, dt)
    transform!(model, state, (1,), dt)
end

function transform!(model, state::AbstractArray{<:RigidBodyState}, state_index::NTuple{N,Int64}, dt) where N

    # extract state
    this_state = get_state(state, state_index)

    # dynamic state
    dynamic_state = this_state.state
    position = dynamic_state.position
    velocity = dynamic_state.velocity
    orientation = dynamic_state.orientation
    angular_velocity = dynamic_state.angular_velocity
    mass = dynamic_state.mass
    inertia = dynamic_state.inertia

    # time derivative of dynamic state
    dynamic_state_derivative = this_state.state_derivative
    velocity_dot = dynamic_state_derivative.velocity_dot
    angular_velocity_dot = dynamic_state_derivative.angular_velocity_dot
    mass_dot = dynamic_state_derivative.mass_dot
    inertia_dot = dynamic_state_derivative.inertia_dot

    # translation in top level frame
    Δx = velocity * dt
    q_parent_2_top = quaternion_frame_2_top(state, state_index[1:end-1])
    translation_topframe = rotate_frame(Δx, q_parent_2_top)

    # rotation in top level frame
    ω = norm(angular_velocity)
    Δθ = ω * dt
    axis = ω > 0 ? angular_velocity / ω : SVector{3}(1.0,0,0)
    axis_topframe = rotate_frame(axis, q_parent_2_top)
    q_rotation_topframe = Quaternion(axis_topframe, Δθ)

    # center of rotation in top level frame
    substate_index = state_index[2:end]
    center_of_rotation_topframe = transform_parent_2_top(position, state[], substate_index)

    # transform model
    transform!(model, this_state.map, translation_topframe, q_rotation_topframe, center_of_rotation_topframe)

    # transform this state
    new_position = position + Δx
    Δq = Quaternion(axis, -Δθ) # should the angle be inverted?
    new_orientation = orientation * Δq # not sure about the order
    new_velocity = velocity + velocity_dot * dt
    new_angular_velocity = angular_velocity + angular_velocity_dot * dt
    new_mass = mass + mass_dot * dt
    new_inertia = inertia + inertia_dot * dt

    new_dynamic_state = DynamicState(
            new_position,
            new_velocity,
            new_orientation,
            new_angular_velocity,
            new_mass,
            new_inertia
        )
    set_dynamic_state!(state, state_index, new_dynamic_state)

    # recurse over substate
    for i in eachindex(this_state.substates)
        new_state_index = (state_index..., i)
        transform!(model, state, new_state_index, dt)
    end
end

"""
    forces!(state, model, dynamic)

Dispatches `forces!` for a `::RigidBodyState`.

# Arguments

* `state::Array{<:RigidBodyState,0}`: the state to which forces are to be applied
* `model::AbstractModel`: the model used to calculate forces
* `dynamic::Bool`: determines whether to apply forces and moments to the dynamic states

"""
function forces!(state::Array{<:RigidBodyState,0}, model, dynamic)
    center = state[].state.position
    f, m = force!(model, map_all(model), center)
    if dynamic
        apply_force!(state, (), f, center)
        apply_moment!(state, (), m)
    end
end

#--- visualize using VTK files ---#

function count_states(state::RigidBodyState)
    return 1 + count_substates(state) # start at 1 for the top level state
end

function count_substates(state::RigidBodyState)
    n_substates = length(state.substates)
    for substate in state.substates
        n_substates += count_substates(substate)
    end

    return n_substates
end

function update_grid!(origin_grid::AbstractArray, xhat_grid, yhat_grid, zhat_grid, level_grid, inverse_level_grid, velocity_grid, angular_velocity_grid, grid_i::Int, state::RigidBodyState, substate_index::NTuple)
    substate = get_substate(state, substate_index)

    # get origin
    origin = transform_parent_2_top(substate.state.position, state, substate_index)
    origin_grid[grid_i] = origin

    # unit vectors of coordinate system
    q = quaternion_frame_2_top(state, substate_index)
    xhat = rotate_frame(SVector{3}(1.0,0,0), q)
    yhat = rotate_frame(SVector{3}(0,1.0,0), q)
    zhat = rotate_frame(SVector{3}(0,0,1.0), q)

    # velocity and angular velocity
    q_parent = quaternion_frame_2_top(state, substate_index[1:end-1])
    velocity = rotate_frame(substate.state.velocity, q_parent)
    angular_velocity = rotate_frame(substate.state.angular_velocity, q_parent)

    xhat_grid[grid_i] = xhat
    yhat_grid[grid_i] = yhat
    zhat_grid[grid_i] = zhat
    level_grid[grid_i] = length(substate_index) + 1
    inverse_level_grid[grid_i] = 1/(length(substate_index) + 1)
    velocity_grid[grid_i] = velocity
    angular_velocity_grid[grid_i] = angular_velocity

    # recurse over substates
    for i in eachindex(substate.substates)
        new_substate_index = (substate_index...,i)
        grid_i = update_grid!(origin_grid, xhat_grid, yhat_grid, zhat_grid, level_grid, inverse_level_grid, velocity_grid, angular_velocity_grid, grid_i + 1, state, new_substate_index)
    end

    return grid_i
end

function visualize(state::Array{<:RigidBodyState,0}, args...; kwargs...)
    visualize(state[], args...; kwargs...)
end

function visualize(state::RigidBodyState, i=nothing, t=nothing; name_prefix="default", path="./")

    # file name
    name_suffix = "_state"
    file_name = name_prefix * name_suffix

    # preallocate containers
    n_datums = count_states(state)
    grid = zeros(SVector{3,Float64},n_datums,1,1)
    xhat_grid = zeros(SVector{3,Float64},n_datums,1,1)
    yhat_grid = zeros(SVector{3,Float64},n_datums,1,1)
    zhat_grid = zeros(SVector{3,Float64},n_datums,1,1)
    level_grid = zeros(n_datums, 1, 1)
    inverse_level_grid = zeros(n_datums, 1, 1)
    velocity_grid = zeros(SVector{3,Float64},n_datums,1,1)
    angular_velocity_grid = zeros(SVector{3,Float64},n_datums,1,1)

    # update with origin and coordinate systems
    update_grid!(grid, xhat_grid, yhat_grid, zhat_grid, level_grid, inverse_level_grid, velocity_grid, angular_velocity_grid, 1, state, ())

    # paraview collection file for timestepping
    !isnothing(t) && (pvd = WriteVTK.paraview_collection(joinpath(path,file_name); append=true))
    !isnothing(i) && (file_name = string(file_name, ".", i))

    # write vtk file
    vtk_grid(joinpath(path,file_name*".vts"), grid) do vtk
        vtk["xhat"] = xhat_grid
        vtk["yhat"] = yhat_grid
        vtk["zhat"] = zhat_grid
        vtk["level"] = level_grid
        vtk["inverse_level"] = inverse_level_grid
        vtk["velocity"] = velocity_grid
        vtk["angular velocity"] = angular_velocity_grid
        if !isnothing(i)
            vtk["i"] = i
        end
        if !isnothing(t)
            vtk["TimeValue"] = t
            pvd[t] = vtk
        end
    end

    !isnothing(t) && WriteVTK.vtk_save(pvd)
end

function initialize_history(state::Array{<:RigidBodyState,0}, save_steps, omit_fields)
    return initialize_history(state[], save_steps, omit_fields)
end

function initialize_history(state::RigidBodyState{<:Any,TM}, save_steps, omit_fields) where TM
    if !(:state in omit_fields)
        # tracking the entire state history is as easy as saving the whole state
        state_history = zeros(RigidBodyState{Float64,TM}, length(save_steps))

        # assemble output
        history = (
            state = state_history,
        )

        fields = (:state,)
    else
        history = NamedTuple()
        fields = ()
    end

    return history, fields
end

function convert_history(::Type{TF}, v::TF0) where {TF<:Number,TF0<:Number}
    if TF0 <: ForwardDiff.Dual
        return TF(ForwardDiff.value(v))
    else
        return TF(v)
    end
end

function convert_history(::Type{SVector{3,TF}}, v::SVector{3,TF0}) where {TF,TF0}
    if TF0 <: ForwardDiff.Dual
        return SVector{3,TF}(ForwardDiff.value(v[1]), ForwardDiff.value(v[2]), ForwardDiff.value(v[3]))
    else
        return SVector{3,TF}(v)
    end
end

function convert_history(::Type{SMatrix{3,3,TF,9}}, m::SMatrix{3,3,TF0,9}) where {TF,TF0}
    if TF0 <: ForwardDiff.Dual
        return SMatrix{3,3,TF,9}(ForwardDiff.value(m[i]) for i in 1:9)
    else
        return SMatrix{3,3,TF,9}(m)
    end
end

function convert_history(::Type{Quaternion{TF}}, q::Quaternion{TF0}) where {TF,TF0}

    # real part
    real = convert_history(TF, q.real)

    # pure part
    pure = convert_history(SVector{3,TF}, q.pure)

    # assemble quaternion
    return Quaternion(real, pure)
end

function convert_history(::Type{DynamicState{TF}}, state::DynamicState{TF0}) where {TF,TF0}
    position = convert_history(SVector{3,TF}, state.position)
    velocity = convert_history(SVector{3,TF}, state.velocity)
    orientation = convert_history(Quaternion{TF}, state.orientation)
    angular_velocity = convert_history(SVector{3,TF}, state.angular_velocity)
    mass = convert_history(TF, state.mass)
    inertia = convert_history(SMatrix{3,3,TF,9}, state.inertia)
    return DynamicState{TF}(position, velocity, orientation, angular_velocity, mass, inertia)
end

function convert_history(::Type{DynamicStateDerivative{TF}}, state::DynamicStateDerivative{TF0}) where {TF,TF0}
    velocity_dot = convert_history(SVector{3,TF}, state.velocity_dot)
    angular_velocity_dot = convert_history(SVector{3,TF}, state.angular_velocity_dot)
    mass_dot = convert_history(TF, state.mass_dot)
    inertia_dot = convert_history(SMatrix{3,3,TF,9}, state.inertia_dot)
    return DynamicStateDerivative{TF}(velocity_dot, angular_velocity_dot, mass_dot, inertia_dot)
end

function convert_history(::Type{RigidBodyState{TF,TM}}, state::RigidBodyState) where {TF,TM}
    name = state.name
    map = TM(state.map)
    dynamic_state = convert_history(DynamicState{TF}, state.state)
    dynamic_state_derivative = convert_history(DynamicStateDerivative{TF}, state.state_derivative)
    substates = Vector{RigidBodyState{TF,TM}}(undef,length(state.substates))
    for i_substate in eachindex(state.substates)
        substates[i_substate] = convert_history(RigidBodyState{TF,TM},state.substates[i_substate])
    end
    return RigidBodyState(name, map, dynamic_state, dynamic_state_derivative, substates)
end

function update_history!(history, state::Array{<:RigidBodyState,0}, i_step)
    update_history!(history, state[], i_step)
end

function update_history!(history, state::RigidBodyState{TF,TM}, i_step) where {TF,TM}
    if :state in keys(history)
        # convert type if necessary
        state_copy = convert_history(RigidBodyState{Float64,TM}, state)

        # update history vector
        history[:state][i_step] = state_copy
    end
end

function (initializer::DefaultInitializer)(state::Array{<:RigidBodyState,0}, time, i_step)
    initializer(state[], time, i_step)
end

function (initializer::DefaultInitializer)(state::RigidBodyState, time, i_step)
    return nothing
end
