abstract type AbstractTimeIntegrator end

struct ForwardEuler <: AbstractTimeIntegrator end

function integrate_time!(vehicle, freestream, controller, postprocessor, current_time, next_time, i_step, run_name, path, integrator::ForwardEuler)

    # timestep
    dt = next_time - current_time

    # reset vehicle
    reset!(vehicle)
    if isnan(vehicle)
        throw("Found nans! after reset!")
    end

    # control
    control!(vehicle, controller, current_time, next_time)
    if isnan(vehicle)
        throw("Found nans! after control!")
    end

    # apply freestream velocity to the model
    apply!(vehicle, freestream, current_time)
    if isnan(vehicle)
        throw("Found nans! after apply!")
    end

    # apply kinematic velocity to the model
    kinematic_velocity!(vehicle)
    if isnan(vehicle)
        throw("Found nans! after kinematic_velocity!")
    end

    # solve models
    solve!(vehicle, dt, i_step)
    if isnan(vehicle)
        throw("Found nans! after solve!")
    end

    # apply forces to state
    forces!(vehicle)
    if isnan(vehicle)
        throw("Found nans! after forces!")
    end

    # postprocessing
    postprocessor(vehicle, controller, current_time, i_step, run_name, path)
    if isnan(vehicle)
        throw("Found nans! after postprocessor!")
    end

    # forward euler step for positional states in the vehicle; also convects the wake
    transform!(vehicle, dt, i_step)
    if isnan(vehicle)
        throw("Found nans! after transform!")
    end

end

function initialize_verbose(integrator::AbstractTimeIntegrator, v_lvl)
    println("\t"^v_lvl, "Type: \t", typeof(integrator))
end
