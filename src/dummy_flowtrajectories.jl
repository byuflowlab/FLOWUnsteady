abstract type AbstractController end

"""
    PrescribedKinematics <: AbstractController

Controller used to fix the kinematics of a vehicle.

# Fields

* `actuate!::Function`: function with the signature `actuate!(plant, u)`; applies control vector `u::AbstractVector` to `plant` by updating time derivative state information in-place
* `state_function::Function`: function with the signature `state = state_function(t)`, where `state` is of the same type as the `vehicle.state`

"""
struct PrescribedKinematics{TA,TS} <: AbstractController
    actuate!::TA
    state_function::TS
end

"Convenience constructor for a controller that maintains constant translational and rotational velocity."
function PrescribedKinematics(state_function::Function)

    function actuate!()
        return nothing
    end

    return PrescribedKinematics(actuate!, state_function)
end

function control!(controller::PrescribedKinematics, vehicle, current_time, next_time)

    # get state and state derivative at current time
    state_dot = controller.state_function(time)

    # construct control input, which completely defines the kinematic state and its derivative
    control_input = SVector{9}(state_dot.center_of_mass..., state_dot.velocity..., state_dot.angular_velocity...)

    return control_input
end

