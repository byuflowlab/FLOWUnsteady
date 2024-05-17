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

* `time_range`: the range of timesteps simulated
* `save_steps`: collection of timestep indices for which data should be saved
* `name_prefix::String`: name prefix used when saving `.bson` files
* `path::String`: path to which `.bson` files are saved
* `bson_every::Int`: save `.bson` files every `bson_every` steps
* `history::NamedTuple`: history of values relevant to the `vehicle` and `controller`
* `fields::Vector{Symbol}`: names of fields found in `history`

"""
struct History{TT,TS,TH,N} <: AbstractPostprocessor
    time_range::TT
    save_steps::TS
    name_prefix::String
    path::String
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

function History(vehicle::AbstractVehicle, controller::AbstractController, time_range;
        name_prefix="default_",
        path="",
        bson_every=1,
        save_steps=1:length(time_range)
    )

    vehicle_history, vehicle_fields = initialize_history(vehicle, save_steps)
    control_history, control_fields = initialize_history(controller, save_steps)

    history, fields = merge_histories(vehicle_history, vehicle_fields, control_history, control_fields)

    return History(time_range[save_steps], save_steps, name_prefix, path, bson_every, history, fields)
end

function (postprocessor::History)(vehicle, controller, time, i_step)
    if i_step in postprocessor.save_steps

        # update history
        i = indexin(i_step, postprocessor.save_steps)[]
        update_history!(postprocessor.history, vehicle, i)
        update_history!(postprocessor.history, controller, i)

        # save bson files
        if i % postprocessor.bson_every == 0
            BSON.@save joinpath(postprocessor.path, postprocessor.name_prefix * "history.$(i_step).bson") postprocessor
        end
    end
end

#------- Paraview Output -------#

"""
    ParaviewOutput <: AbstractPostprocessor

Functor used to trigger generation of output files for visualization in paraview.

# Fields

* `save_steps`: collection of timestep indices for which output files should be saved
* `name_prefix::String`: name prefix for output files
* `path::String`: path where output files will be saved

"""
struct ParaviewOutput{TS} <: AbstractPostprocessor
    save_steps::TS
    name_prefix::String
    path::String
end

function (postprocessor::ParaviewOutput)(vehicle, controller, time, i_step)
    if i_step in postprocessor.save_steps
        i = indexin(i_step, postprocessor.save_steps)[]
        visualize(vehicle, i_step, time; name_prefix=postprocessor.name_prefix, path=postprocessor.path)
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

function (postprocessor::MultiPostprocessor)(vehicle, controller, time, i_step)
    for p in postprocessor.postprocessors
        p(vehicle, controller, time, i_step)
    end
end
