abstract type AbstractController end

function initialize_verbose(controller::AbstractController, v_lvl)
    println("\t"^v_lvl, "Controller: \t", typeof(controller))
end

"""
    initialize_history(controller, save_steps)

Used to initialize a `History` object.

# Arguments

* `controller::AbstractController`: the controller used in the simulation
* `save_steps`: iterable of timestep indices at which we desire to save the history

# Returns

* `control_history`: the preallocated control history

"""
function initialize_history(controller::AbstractController, save_steps)
    @error "initialize_history not defined for $(typeof(controller))"
end

"""
    update_history!(history, controller, i_step)

Updates the history relevant to `controller`.

# Arguments

* `history::NamedTuple`: the history to be updated
* `i_step::Int`: the timestep at which the history is to be updated

"""
function update_history!(history, controller::AbstractController, i_step)
    @error "update_history! not defined for $(typeof(controller))"
end

"""
    PrescribedKinematics <: AbstractController

Controller used to fix the kinematics of a vehicle.

"""
struct PrescribedKinematics <: AbstractController end

function control!(vehicle, ::PrescribedKinematics, current_time, next_time)
    @assert typeof(vehicle) <: AbstractVehicle{false} "dynamic vehicle is incompatible with a PrescribedKinematics controller; set `AbstractVehicle{false}`"
    return nothing
end

function initialize_history(controller::PrescribedKinematics, save_steps)
    return (), () # no control performed for PrescribedKinematics
end

function update_history!(history, controller::PrescribedKinematics, i_step)
    return nothing
end

function (initializer::DefaultInitializer)(controller::AbstractController, time, i_step)
    @error "initializer::DefaultInitializer not defined for $(typeof(controller))"
end


function (initializer::DefaultInitializer)(controller::PrescribedKinematics, time, i_step)
    return nothing
end
