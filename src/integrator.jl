abstract type AbstractTimeIntegrator end

struct ForwardEuler <: AbstractTimeIntegrator end

function integrate_time!(vehicle, freestream, controller, postprocessor, current_time, next_time, i_step, integrator::ForwardEuler)

    # timestep
    dt = next_time - current_time

    # control
    control!(vehicle, controller, current_time, next_time)

    # apply freestream velocity to the model
    apply!(vehicle, freestream, current_time)

    # apply kinematic velocity to the model
    kinematic_velocity!(vehicle)

    # solve models
    solve!(vehicle, dt)

    # apply forces to state
    forces!(vehicle)

    # postprocessing
    postprocessor(vehicle, controller, current_time, i_step)

    # forward euler step for positional states in the vehicle; also convects the wake
    transform!(vehicle, dt, i_step)

end

function initialize_verbose(integrator::AbstractTimeIntegrator, v_lvl)
    println("\t"^v_lvl, "Type: \t", typeof(integrator))
end
