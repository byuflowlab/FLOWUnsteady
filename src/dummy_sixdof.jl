struct Quaternion{TF}
    real::TF
    pure::SVector{3,TF}
end

function rotate(v, q::Quaternion)
    t = 2*cross(q.pure, v)
    return v + q.real*t + cross(q.pure,t)
end

"perform the reverse rotation"
function antirotate(v, q::Quaternion)
    q = Quaternion(q.real, -q.pure)
    return rotate(v, q)
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

abstract type AbstractDynamicState end

"""
    RigidBodyState

Dynamic state of a rigid body. Could contain nested rigid bodies with fixed connections.

# Fields

* `position::Vector`: x, y, and z coordinates of the center of mass in the global frame
* `velocity::Vector`: x, y, and z components of the translational velocity in the _ frame
* `orientation::Matrix`: 3x3 matrix whose columns describe the unit vectors of the _ frame in global coordinates
* `angular_velocity::Vector`: x, y, and z components of the angular velocity in the _ frame
* `mass::Float64`: mass of the rigid body, including its substates
* `inertia::Matrix`: the inertial matrix ''I_{ij}'' about the center of mass in the body frame
* `substates::Vector{RigidBodyState}`: vector of substates describing rigid bodies attached at fixed points

"""
struct RigidBodyState{TF} <: AbstractDynamicState
    position::SVector{3,TF}
    velocity::SVector{3,TF}
    orientation::SMatrix{3,3,TF,9}
    angular_velocity::SVector{3,TF}
    mass::TF
    inertia::SMatrix{3,3,TF,9}
    substates::Vector{RigidBodyState{TF}}
end

function RigidBodyState(TF=Float64;
    position = zero(SVector{3,TF}),
    velocity = zero(SVector{3,TF}),
    orientation = zero(SMatrix{3,3,TF,9}),
    angular_velocity = zero(SVector{3,TF}),
    mass = zero(TF),
    inertia = zero(SMatrix{3,3,TF,9}),
    substates = RigidBodyState{TF}[]
)

    return RigidBodyState{TF}(position, velocity, orientation, angular_velocity, mass, inertia, substates)
end

