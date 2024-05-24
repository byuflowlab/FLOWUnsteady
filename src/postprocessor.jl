"""
    AbstractPostprocessor

Objects inheriting from `AbstractPostprocessor` are functors that perform any desired preprocessing after completing each timestep. Called as a functor as `(::AbstractPostprocessor)(vehicle::AbstractVehicle, controller::AbstractController, time, i_step)`.

"""
abstract type AbstractPostprocessor end

#------- History -------#

"""
    History <: AbstractPostprocessor

Saves relevant fields specific to the `vehicle` and `controller`.

# Fields

* `time_range`: the range of timesteps saved
* `save_steps`: collection of timestep indices for which data should be saved"
* `bson_every::Int`: save `.bson` files every `bson_every` steps
* `history::NamedTuple`: history of values relevant to the `vehicle` and `controller`
* `fields::Vector{Symbol}`: names of fields found in `history`

"""
struct History{TS,TH,N} <: AbstractPostprocessor
    time_range::Vector{Float64}
    save_steps::TS
    bson_every::Int
    history::TH
    fields::NTuple{N,Symbol}
end

function merge_histories(history1, field1, history2, field2)
    fields = (field1..., field2...)
    @assert length(unique(fields)) == length(field1) + length(field2) "duplicate field names found"
    history = merge(history1, history2)
    return history, fields
end

function History(vehicle::AbstractVehicle, controller::AbstractController, save_steps;
        bson_every=1,
    )

    vehicle_history, vehicle_fields = initialize_history(vehicle, save_steps)
    control_history, control_fields = initialize_history(controller, save_steps)

    history, fields = merge_histories(vehicle_history, vehicle_fields, control_history, control_fields)

    time_range = zeros(Float64, length(save_steps))

    return History(time_range, save_steps, bson_every, history, fields)
end

function (history::History)(vehicle, controller, time, i_step, run_name, path)
    if i_step in history.save_steps

        # update history
        i = indexin(i_step, history.save_steps)[]
        history.time_range[i] = time
        update_history!(history.history, vehicle, i)
        update_history!(history.history, controller, i)

        # save bson files
        if i % history.bson_every == 0
            BSON.@save joinpath(path, run_name * "_history.bson") history
        end
    end
end

#------- Paraview Output -------#

"""
    ParaviewOutput <: AbstractPostprocessor

Functor used to trigger generation of output files for visualization in paraview.

# Fields

* `save_steps`: collection of timestep indices for which output files should be saved

"""
struct ParaviewOutput{TS} <: AbstractPostprocessor
    save_steps::TS
end

function (postprocessor::ParaviewOutput)(vehicle, controller, time, i_step, run_name, path)
    if i_step in postprocessor.save_steps
        i = indexin(i_step, postprocessor.save_steps)[]
        visualize(vehicle, i_step, time; name_prefix=run_name, path)
    end
end

#------- MultiPostprocessor -------#

"""
    MultiPostprocessor <: AbstractPostprocessor

A collection of `<:AbstractPostprocessor` objects.

# Fields

* `postprocessors::Tuple`: `::Tuple` of `<:AbstractPostprocessor` objects

"""
struct MultiPostprocessor{TP} <: AbstractPostprocessor
    postprocessors::TP
end

function (postprocessor::MultiPostprocessor)(vehicle, controller, time, i_step, run_name, path)
    for p in postprocessor.postprocessors
        p(vehicle, controller, time, i_step, run_name, path)
    end
end
