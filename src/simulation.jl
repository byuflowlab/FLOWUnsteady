#------- preprocessor -------#

"""
    AbstractPreprocessor

Objects inheriting from `AbstractPreprocessor` are functors that perform any desired preprocessing before commencing each timestep.

"""
abstract type AbstractPreprocessor end

"""
    DefaultPreprocessor <: AbstractPreprocessor

Default preprocessing object does nothing.

"""
struct DefaultPreprocessor <: AbstractPreprocessor end

(::DefaultPreprocessor)(simulate, time, run_name, path) = nothing

"""
    Simulation

Object containing all data structures required for a simulation.

# Fields

* `vehicle::AbstractVehicle`: the vehicle to be simulated
* `freestream::AbstractFreestream`: defines freestream properties during the simulation in the north-east-down frame
* `controller::AbstractController`: the controller used during the simulation
* `preprocessor`: function or functor called as `preprocessor(simulation, current_time)` to perform any preprocessing operation before each timestep
* `integrator::AbstractTimeIntegrator`: object used to dispatch the time integration scheme
* `postprocessor`: function or functor called as `preprocessor(simulation, current_time)` to perform any postprocessing operations at the end of each timestep

"""
struct Simulation{TV<:AbstractVehicle, TF<:AbstractFreestream, TC<:AbstractController, TIZ<:AbstractInitializer, TPR<:AbstractPreprocessor, TI<:AbstractTimeIntegrator, TPO<:AbstractPostprocessor}
    vehicle::TV
    freestream::TF
    controller::TC
    initializer::TIZ
    preprocessor::TPR
    integrator::TI
    postprocessor::TPO
    i_step::Array{Int,0}
end

function Simulation(vehicle, time_range;
            freestream=SimpleFreestream(;velocity=SVector{3}(-1.0,0,0)),
            controller=PrescribedKinematics(),
            initializer=DefaultInitializer(),
            preprocessor=DefaultPreprocessor(),
            integrator=ForwardEuler(),
            postprocessor=DefaultPostprocessor(vehicle, time_range),
            i_step=0
        )
    i_step_container = Array{Int,0}(undef)
    i_step_container[] = i_step
    return Simulation(vehicle, freestream, controller, initializer, preprocessor, integrator, postprocessor, i_step_container)
end

function simulate!(simulation::Simulation, time_range;
        run_name="FLOWUnsteady_default",
        path="", overwrite_path=true,
        verbose=true,
    )

    #--- unpack simulation ---#

    vehicle = simulation.vehicle
    freestream = simulation.freestream
    controller = simulation.controller
    initializer = simulation.initializer
    preprocessor = simulation.preprocessor
    integrator = simulation.integrator
    postprocessor = simulation.postprocessor
    i_step = simulation.i_step[]

    #--- prepare save path ---#

    if splitdir(path)[2] != ""
        isdir(path) && overwrite_path && rm(path; force=true, recursive=true)
        !isdir(path) && mkdir(path)
    end

    #--- initialize ---#

    initializer(simulation, time_range[1], i_step)

    verbose && (time_beginning = initialize_verbose(simulation, time_range, run_name, path))

    #--- loop over time ---#

    current_time = time_range[1]

    for next_time in time_range[2:end]

        #--- preprocessing ---#

        preprocessor(simulation, current_time, run_name, path)

        #--- time integration scheme and postprocessing after solving, before advancing the timestep ---#

        integrate_time!(vehicle, freestream, controller, postprocessor, current_time, next_time, i_step, run_name, path, integrator)

        #--- increment time ---#

        current_time = next_time
        i_step += 1

        #--- verbose output ---#

        verbose && verbose_print(i_step, current_time)

    end

    # postprocessing after the last timestep
    postprocessor(vehicle, controller, time_range[end], i_step, run_name, path)

    #--- update simulation ---#

    simulation.i_step[] = i_step

    verbose && (finalize_verbose(run_name, path, time_beginning))

    return nothing
end

function initialize_verbose(simulation::Simulation, time_range, run_name, path)

    time_beginning = Dates.DateTime(Dates.now())
    line1 = "*"^73
    line2a = "#-------"
    line2b = "-------#"
    line3 = "-"^73
    println(line1)
    println(line3)
    println("\tBEGIN FLOWUnsteady SIMULATION")
    println(line3)
    println("\n\t", line2a, "  VEHICLE   ", line2b)#, "\n")
    initialize_verbose(simulation.vehicle, 2)
    println("\n\t", line2a, " FREESTREAM ", line2b)#, "\n")
    initialize_verbose(simulation.freestream, 2)
    println("\n\t", line2a, " CONTROLLER ", line2b)#, "\n")
    initialize_verbose(simulation.controller, 2)
    println("\n\t", line2a, " INTEGRATOR ", line2b)#, "\n")
    initialize_verbose(simulation.integrator, 2)
    println("\n", line3)
    println("\tSTART $(joinpath(path,run_name))")
    println("\n\t\t$time_beginning")
    println(line3)
    println(line1)
    println("\n\tstep | time   ")
    println(  "\t---- | ------ ")
    verbose_print(0, time_range[1])
    return time_beginning
end

function verbose_print(i_step, current_time)
    time_string = string(current_time)
    length(time_string) < 6 && (time_string = rpad(time_string, 6, "0"))
    println("\t",lpad(i_step,4,"0"), " | ", time_string[1:6])
end

function timeformat(time_beg, time_end)
    time_delta = Dates.value(time_end)-Dates.value(time_beg)
    hrs = Int(floor(time_delta/1000/60/60))
    mins = Int(floor((time_delta-hrs*60*60*1000)/1000/60))
    secs = Int(floor((time_delta-hrs*60*60*1000-mins*1000*60)/1000))
    return hrs,mins,secs
end

function finalize_verbose(run_name, path, time_beginning)

    time_end = Dates.DateTime(Dates.now())
    hrs, mins, secs = timeformat(time_beginning, time_end)
    line1 = "*"^73
    line3 = "-"^73
    println("\n", line1)
    println(line3)
    println("\n\tEND $(joinpath(path,run_name))")
    println("\n\t\t","ELAPSED TIME: $hrs hours $mins minutes $secs seconds\n")
    println(line3)
    println(line1)

end

function visualize(sim::Simulation, i::Union{Nothing,Int}=nothing; name_prefix="default", path="")
    visualize(sim.vehicle, i, sim.time[]; name_prefix, path)
end

