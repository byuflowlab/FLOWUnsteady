abstract type AbstractController end

"""
    PrescribedKinematics <: AbstractController

Controller used to fix the kinematics of a vehicle.

# Fields

* `state_function::Function`: a function called with the syntax 
        state::AbstractDynamicState = state_function(time::Float)

"""
struct PrescribedKinematics{TSF} <: AbstractController
    state_function::TSF 
end

function PrescribedKinematics(maintain_state::RigidBodyState)
    
    function sf(time) let state = maintain_state
           position = state.position + state.velocity * time
           orientation = rotate(state.orientation, state.angular_velocity/norm(state.angular_velocity), norm(state.angular_velocity))
           new_state = RigidBodyState(position, state.velocity, orientation, state.angular_velocity, state.mass, state.inertia, state.substates)
           return new_state
       end
    end

    return PrescribedKinematics(sf)
end
