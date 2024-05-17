abstract type AbstractTimeIntegrator end

struct ForwardEuler <: AbstractTimeIntegrator end

function integrate_time!(vehicle, freestream, controller, postprocessor, current_time, next_time, i_step, ::ForwardEuler)

    # timestep
    dt = next_time - current_time

    # control
    control!(vehicle, controller, current_time, next_time)

    # apply freestream velocity to the model
    apply!(vehicle, freestream, current_time)

    # apply kinematic velocity to the model
    kinematic_velocity!(vehicle)

    # solve models
    DEBUG[] && println("Pre-solve!")
    solve!(vehicle, dt)
    DEBUG[] && println("Post-solve!")

    # apply forces to state
    forces!(vehicle)
    DEBUG[] && println("Post-forces!")

    # postprocessing
    postprocessor(vehicle, controller, current_time, i_step)
    DEBUG[] && println("Post-postprocessor")

    # forward euler step for positional states in the vehicle; also convects the wake (not necessarily forward Euler) for models with unsteady aerodynamics
    transform!(vehicle, dt)
    DEBUG[] && println("post transform")
end

function initialize_verbose(integrator::AbstractTimeIntegrator, v_lvl)
    println("\t"^v_lvl, "Type: \t", typeof(integrator))
end
