"""
    AbstractInitializer

Objects inheriting from `AbstractInitializer` are functors that initialize an `<:AbstractVehicle` as `myinitializer(vehicle::AbstractVehicle, time_zero)`. They set up the "initial value" of the initial value problem before the ODE is solved.

"""
abstract type AbstractInitializer end

"""
    DefaultInitializer

Default initialization object does nothing.

"""
struct DefaultInitializer <: AbstractInitializer end

function (initializer::DefaultInitializer)(simulation, time, i_step)

    initializer(simulation.controller, time, i_step)
    initializer(simulation.vehicle, time, i_step)

    return nothing
end

