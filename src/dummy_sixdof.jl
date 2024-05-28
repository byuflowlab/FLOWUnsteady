using LinearAlgebra

import Base.* # extend multiplication of quaternions
import Base.+ # extend addition of dynamic states
import Base.- # extend subtraction of dynamic states
import Base./ # extend division of dynamic states by a scalar
import Base.inv # extend inversion of quaternions
import Base.zero # extend definition of null quaternion

#--- quaternion math for easy rotations ---#

struct Quaternion{TF}
    real::TF
    pure::SVector{3,TF}
end

function Quaternion(axis, angle)
    @assert isapprox(norm(axis), 1.0; atol=1e-12) "axis must be a unit vector"
    stheta_over_2, ctheta_over_2 = sincos(angle/2)
    real_part = ctheta_over_2
    pure_part = axis * stheta_over_2

    return Quaternion(real_part, pure_part)
end

Base.zero(::Type{Quaternion{TF}}) where TF = Quaternion{TF}(TF(1.0), SVector{3,TF}(0,0,0))

function *(q::Quaternion, p::Quaternion)
    real = q.real * p.real - dot(q.pure, p.pure)
    pure = q.real * p.pure + p.real * q.pure + cross(q.pure, p.pure)
    return Quaternion(real, pure)
end

function inv(q::Quaternion{TF}) where TF
    return Quaternion{TF}(q.real, -q.pure)
end

"""
    rotate(v, axis, angle)

Rotates vector `v` about `axis` by `angle`. Alternatively, rotates the reference frame of `v` about `axis` by `-angle`.

# Arguments

* `v::Union{AbstractVector,AbstractMatrix}`: the vector or matrix of column vectors to be rotated
* `axis::AbstractVector`: normalized vector representing the axis of rotation
* `angle::Number`: angle about which the vector(s) is(are) to be rotated according to the right hand rule; alternatively, the negative angle about which the reference frame is to be rotated

# Returns

* `v_prime::Union{AbstractVector,AbstractMatrix}`: rotated `v`

"""
function rotate(v, axis, angle)
    return rotate(v, Quaternion(axis, angle))
end

"""
    rotate(v,q)

Rotates `v` by quaternion `q`. Alternatively, rotates the reference frame of `v` by quaternion `inv(q)`. Mathematically equivalent to computing ''q v q^* ''.

# Arguments

* `v::AbstractVector`: the vector to be rotated
* `q::Quaternion`: the quaternion defining the rotation

# Returns

* `v_prime::AbstractVector`: rotated `v`

"""
function rotate(v::AbstractVector, q::Quaternion)
    t = 2*cross(q.pure, v)
    return v + q.real*t + cross(q.pure,t)
end

"""
    rotate(m, q)

Rotates the column vectors of `m` by quaternion `q`. Alternatively, rotates the reference frame of the column vectors of `m` by quaternion `inv(q)`. Mathematically equivalent to computing ''q v q^* '' for each column vector ''v''.

# Arguments

* `m::AbstractMatrix`: the matrix of column vectors to be rotated
* `q::Quaternion`: the quaternion defining the rotation

# Returns

* `m_prime::SMatrix{3,3}`: rotated `m`

"""
function rotate(m::AbstractMatrix, q::Quaternion)
    t1 = 2*cross(q.pure, SVector{3}(m[1,1],m[2,1],m[3,1]))
    t1 = v + q.real*t2 + cross(q.pure,t1)
    t2 = 2*cross(q.pure, SVector{3}(m[1,2],m[2,2],m[3,2]))
    t2 = v + q.real*t2 + cross(q.pure,t2)
    t3 = 2*cross(q.pure, SVector{3}(m[1,3],m[2,3],m[3,3]))
    t3 = v + q.real*t3 + cross(q.pure,t3)
    return hcat(t1,t2,t3)
end

"""
    rotate_frame(v, axis, angle)

Rotates the reference frame of `v` about `axis` by `angle`. In other words, rotates `v` about `axis` by `-angle`. Mathematically equivalent to computing ''q^* v q ''.

# Arguments

* `v::AbstractVector`: the vector whose reference frame is to be rotated
* `axis::AbstractVector`: normalized vector representing the axis of rotation
* `angle::Number`: the angle about which the reference frame is to be rotated according to the right hand rule

# Returns

* `v_prime::AbstractVector`: `v` expressed in the rotated reference frame

"""
function rotate_frame(v, axis, angle)
    return rotate_frame(v, Quaternion(axis, angle))
end

"""
    rotate_frame(v, q)

Rotates the reference frame of `v` by quaternion `q`. In other words, rotates `v` by quaternion `inv(q)`.

# Arguments

* `v::AbstractVector`: the vector whose reference frame is to be rotated
* `q::Quaternion`: the quaternion defining the rotation

# Returns

* `v_prime::AbstractVector`: `v` expressed in the rotated reference frame

"""
function rotate_frame(v::AbstractVector, q::Quaternion)
    qstar = inv(q)
    return rotate(v, qstar)
end

"""
    rotate_frame(m, q)

Rotates the reference frame of the column vectors of `m` by quaternion `q`.

# Arguments

* `m::AbstractMatrix`: the matrix of column vectors whose reference frames are to be rotated
* `q::Quaternion`: the quaternion defining the rotation

# Returns

* `m_prime::SMatrix{3,3}`: `m` expressed in the rotated reference frame

"""
function rotate_frame(m::AbstractMatrix, q::Quaternion)
    qstar = inv(q)
    return rotate(m, qstar)
end

#--- dynamic state objects ---#

"""
    DynamicState

Contains all state information required for dynamics.

# Fields

* `position::Vector`: x, y, and z coordinates of the origin of the body frame in the parent frame
* `velocity::Vector`: x, y, and z components of the translational velocity of the body as measured in the parent frame, expresed in the parent frame coordinates
* `orientation::Quaternion`: quaternion describing the rotation between body and parent frames; constructed using the axis and angle (in body frame coordinates) that the body frame must be rotated to align with the parent frame coordinates; i.e., `rotate_frame(v_body_frame, q)` results in the same vector but expressed in the parent frame coordinates
* `angular_velocity::Vector`: x, y, and z components of the angular velocity of the body frame measured in the parent frame, expressed in the parent frame
* `mass::Float64`: mass of the body
* `inertia::Matrix`: the inertial tensor ''I_{ij}'' about the center of mass in the body frame

"""
struct DynamicState{TF}
    position::SVector{3,TF}
    velocity::SVector{3,TF}
    orientation::Quaternion{TF}
    angular_velocity::SVector{3,TF}
    mass::TF
    inertia::SMatrix{3,3,TF,9}
end

function DynamicState(TF=Float64;
        position = zero(SVector{3,TF}),
        velocity = zero(SVector{3,TF}),
        orientation = zero(Quaternion{TF}),
        angular_velocity = zero(SVector{3,TF}),
        mass = zero(TF),
        inertia = zero(SMatrix{3,3,TF,9}),
    )

    return DynamicState{TF}(position, velocity, orientation, angular_velocity, mass, inertia)
end

function Base.zero(::Type{DynamicState{TF}}) where TF
    position = zero(SVector{3,TF})
    velocity = zero(SVector{3,TF})
    orientation = zero(Quaternion{TF})
    angular_velocity = zero(SVector{3,TF})
    mass = zero(TF)
    inertia = zero(SMatrix{3,3,TF,9})
    return DynamicState{TF}(position, velocity, orientation, angular_velocity, mass, inertia)
end

function Base.isnan(state::DynamicState)
    for v in state.position
        if isnan(v)
            return true
        end
    end
    for v in state.velocity
        if isnan(v)
            return true
        end
    end
    for v in state.orientation.pure
        if isnan(v)
            return true
        end
    end
    if isnan(state.orientation.real)
        return true
    end
    for v in state.angular_velocity
        if isnan(v)
            flag = true
        end
    end
    if isnan(state.mass)
        return true
    end
    for v in state.inertia
        if isnan(v)
            return true
        end
    end
    return false
end

function -(state1::DynamicState{TF1}, state2::DynamicState{TF2}) where {TF1,TF2}
    TF = promote_type(TF1,TF2)
    difference = DynamicState{TF}(
            state1.position - state2.position,
            state1.velocity - state2.velocity,
            state1.orientation,
            state1.angular_velocity - state_2.angular_velocity,
            state1.mass - state2.mass,
            state1.inertia - state2.inertia
        )
    return difference
end

function /(state::DynamicState{TF1}, divisor::TF2) where {TF1,TF2}
    TF = promote_type(TF1,TF2)
    quotient = DynamicState{TF}(
            state1.position / divisor,
            state1.velocity / divisor,
            state1.orientation,
            state1.angular_velocity / divisor,
            state1.mass / divisor,
            state1.inertia / divisor
        )
    return quotient
end

"""
    DynamicStateDerivative

Contains the time derivatives of a `DynamicState` object.

# Fields

* `velocity_dot::SVector{3,TF}`: time rate of change of the corresponding `DynamicState` object's velocity as measured in the parent frame
* `angular_velocity_dot::SVector{3,TF}`: time rate of change of the corresponding `DynamicState` object's angular velocity as measured in the parent frame
* `mass_dot::TF`: time rate of change of the corresponding `DynamicState` object's mass
* `inertia_dot::SMatrix{3,3,TF,9}`: time rate of change of the corresponding `DynamicState` object's inertia tensor as measured in the body frame

"""
struct DynamicStateDerivative{TF}
    velocity_dot::SVector{3,TF}
    angular_velocity_dot::SVector{3,TF}
    mass_dot::TF
    inertia_dot::SMatrix{3,3,TF,9}
end

function DynamicStateDerivative(TF=Float64;
        velocity_dot = SVector{3,TF}(0,0,0),
        angular_velocity_dot = SVector{3,TF}(0,0,0),
        mass_dot = zero(TF),
        inertia_dot = zero(SMatrix{3,3,TF,9})
    )
    return DynamicStateDerivative{TF}(velocity_dot, angular_velocity_dot, mass_dot, inertia_dot)
end

function Base.zero(::Type{DynamicStateDerivative{TF}}) where TF
    velocity_dot = zero(SVector{3,TF})
    angular_velocity_dot = zero(SVector{3,TF})
    mass_dot = zero(TF)
    inertia_dot = zero(SMatrix{3,3,TF,9})
    return DynamicStateDerivative{TF}(velocity_dot, angular_velocity_dot, mass_dot, inertia_dot)
end

"""
    state_derivative(state, force, moment, inertia_dot=zeros(3,3))

Computes the time derivative of each state in `state` (with the exception of `state.orientation`).

# Arguments

* `state::DynamicState`: the state object to be differentiated
* `force::Vector{Float64}`: the force vector expressed in the inertial frame
* `moment::Vector{Float64}`: the moment vector expressed in the inertial frame
* `inertia_dot::Matrix{Float64}`: the body frame time rate of change of the inertia tensor expressed in the body frame

# Ouputs

* `state_derivative::DynamicStateDerivative`: state object containing the time derivatives of the input `state`

"""
function state_derivative(state::DynamicState{TF1}, force::AbstractVector{TF2}, moment::AbstractVector{TF2}, inertia_dot=zero(SMatrix{3,3,TF1,9})) where {TF1,TF2}
    # set type
    TF = promote_type(TF1,TF2)

    # trivial derivatives
    velocity_dot = F / state.mass

    # angular velocity derivative
    moment_body_frame = rotate_frame(moment, state.orientation)
    angular_velocity_body_frame = rotate_frame(state.angular_velocity, state.orientation)
    angular_velocity_dot = moment_body_frame - inertia_dot * angular_velocity_body_frame - cross(angular_velocity_body_frame, state.inertia * angular_velocity_body_frame)
    angular_velocity_dot = state.inertia \ angular_velocity_dot
    angular_velocity_dot = rotate(angular_velocity_dot, state.orientation)

    # assume mass remains constant
    mass_dot = zero(TF)

    return DynamicStateDerivative(TF; velocity_dot, angular_velocity_dot, mass_dot, inertia_dot)
end

function Base.isnan(state::DynamicStateDerivative)
    for v in state.velocity_dot
        if isnan(v)
            return true
        end
    end
    for v in state.angular_velocity_dot
        if isnan(v)
            flag = true
        end
    end
    if isnan(state.mass_dot)
        return true
    end
    for v in state.inertia_dot
        if isnan(v)
            return true
        end
    end
    return false
end

