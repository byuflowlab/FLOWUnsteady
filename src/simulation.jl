struct Simulation{TV<:AbstractVehicle, TC<:AbstractController, TCM<:AbstractControlMap, TI<:AbstractTimeIntegrator}
    vehicle::TV
    controller::TC
    control_map::TCM
    integrator::TI
    simulation_time::Array{Float64,0}
end

function simulate!(simulation::Simulation, time_range)

    #--- unpack simulation ---#

    vehicle = simulation.vehicle
    controller = simulation.controller
    integrator = simulation.integrator
    time = simulation.time

    #--- initialize ---#

    state = initialize!(vehicle, time_range[1])

    #--- loop over time ---#

    last_time = time_range[1]
    for current_time in skip_first(time_range)

        #--- time integration scheme ---#

        state = integrate_time!(state, vehicle, controller, last_time, current_time, integrator)

        # update t_last
        last_time = current_time
    end

    return simulation
end

