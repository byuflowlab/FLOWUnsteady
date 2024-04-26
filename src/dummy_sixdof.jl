#--- quaternion math for easy rotations ---#

struct Quaternion{TF}
    real::TF
    pure::SVector{3,TF}
end

function rotate(v::AbstractVector, q::Quaternion)
    t = 2*cross(q.pure, v)
    return v + q.real*t + cross(q.pure,t)
end

"perform the reverse rotation"
function antirotate(v::AbstractVector, q::Quaternion)
    q = Quaternion(q.real, -q.pure)
    return rotate(v, q)
end

function rotate(m::AbstractMatrix, q::Quaternion)
    t1 = 2*cross(q.pure, SVector{3}(m[1,1],m[2,1],m[3,1]))
    t1 = v + q.real*t2 + cross(q.pure,t1)
    t2 = 2*cross(q.pure, SVector{3}(m[1,2],m[2,2],m[3,2]))
    t2 = v + q.real*t2 + cross(q.pure,t2)
    t3 = 2*cross(q.pure, SVector{3}(m[1,3],m[2,3],m[3,3]))
    t3 = v + q.real*t3 + cross(q.pure,t3)
    return hcat(t1,t2,t3)
end

"perform the reverse rotation"
function antirotate(m::AbstractMatrix, q::Quaternion)
    q = Quaternion(q.real, -q.pure)
    return rotate(m, q)
end

function get_quaternion(axis, angle)
    @assert isapprox(norm(axis), 1.0; atol=1e-12) "axis must be a unit vector"
    stheta_over_2, ctheta_over_2 = sincos(angle/2)
    real_part = ctheta_over_2
    pure_part = axis * stheta_over_2
    quat = Quaternion(real_part, pure_part)
    return quat
end

function rotate(v, axis, angle)
    return rotate(v, get_quaternion(axis, angle))
end

function antirotate(v, axis, angle)
    return rotate(v, get_quaternion(axis, -angle))
end

#--- dynamic state objects ---#

abstract type AbstractDynamicState end

"""
    RigidBodyState

Dynamic state of a rigid body. Could contain nested rigid bodies with fixed connections.

# Fields

* `position::Vector`: x, y, and z coordinates of the center of mass of the body frame in the parent frame coordinates
* `velocity::Vector`: x, y, and z components of the translational velocity of the body frame in the parent frame coordinates
* `orientation::Quaternion`: quaternion describing the rotational deviation of the body frame from its parent frame
* `angular_velocity::Vector`: x, y, and z components of the angular velocity of the body frame in the parent frame coordinates
* `mass::Float64`: mass of the rigid body, including its substates
* `inertia::Matrix`: the inertial tensor ''I_{ij}'' about the center of mass in the body frame
* `substates::Vector{RigidBodyState}`: vector of substates describing rigid bodies attached to this one 

"""
struct RigidBodyState{TF} <: AbstractDynamicState
    center_of_mass::SVector{3,TF}
    center_of_rotation::SVector{3,TF}
    velocity::SVector{3,TF}
    orientation::Quaternion{TF}
    angular_velocity::SVector{3,TF}
    mass::TF
    inertia::SMatrix{3,3,TF,9}
    substates::Vector{RigidBodyState{TF}}
end

function RigidBodyState(TF=Float64;
        position = zero(SVector{3,TF}),
        velocity = zero(SVector{3,TF}),
        orientation = Quaternion{TF}(zero(TF),zero(SVector{3,TF})),
        angular_velocity = zero(SVector{3,TF}),
        mass = zero(TF),
        inertia = zero(SMatrix{3,3,TF,9}),
        substates = RigidBodyState{TF}[]
    )

    return RigidBodyState{TF}(position, velocity, orientation, angular_velocity, mass, inertia, substates)
end

struct RigidBodyDelta{TF} <: AbstractDynamicDelta
    translation::SVector{3,TF}
    center_of_rotation::SVector{3,TF}
    rotation::Quaternion{TF}
    subdeltas::Vector{RigidBodyDelta{TF}}
end

function RigidBodyDelta(TF=Float64;
        translation = zero(SVector{3,TF}),
        center_of_rotation = zero(SVector{3,TF}),
        rotation = Quaternion{TF}(zero(TF), zero(SVector{3,TF}))
    )

    return RigidBodyDelta{TF}(translation, center_of_rotation, rotation)
end
