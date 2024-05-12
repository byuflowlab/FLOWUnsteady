abstract type AbstractTimeIntegrator end

struct ForwardEuler <: AbstractTimeIntegrator end

function integrate_time!(state, vehicle, controller, current_time, next_time, ::ForwardEuler)

    # timestep
    dt = next_time - current_time

    # time derivative
    state_dot = state_derivatives!(state, vehicle, controller, current_time)

    # next state
    next_state = state + state_dot * dt

    return next_state
end
