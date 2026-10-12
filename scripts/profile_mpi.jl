#!/usr/bin/env julia
# Sample the complete, unchanged DJI hover pilot after a same-case warmup.
# Example: mpirun -np 3 julia --project=environments/mpi scripts/profile_mpi.jl \
#   --mode mpi --gc-policy deferred --output /tmp/hover-profile
# Additional options: --profile-delay 0.003 --profile-buffer 2000000
# Profile stack counts/fractions are statistical diagnostics, not CPU time. Use the
# ordinary benchmark for uninstrumented elapsed-time comparisons.
using Profile, Serialization
include(joinpath(@__DIR__, "..", "examples", "mpi", "rotorhover.jl"))

function profile_options(args)
    iseven(length(args)) || error("Arguments must be --name value pairs")
    pilot_args = String[]
    delay, buffer = 0.003, 2_000_000
    for i in 1:2:length(args)
        if args[i] == "--profile-delay"
            delay = parse(Float64, args[i + 1])
        elseif args[i] == "--profile-buffer"
            buffer = parse(Int, args[i + 1])
        else
            append!(pilot_args, args[i:i + 1])
        end
    end
    isfinite(delay) && delay > 0 && buffer > 0 || error("Profile delay and buffer must be positive")
    config = parse_options(pilot_args)
    config.warmup_steps == config.nsteps || error("Profiling requires one complete same-case warmup")
    return config, delay, buffer
end

# Worker backtraces need not contain their spawning pass function. Include the
# corresponding worker/kernel names as well as the top-level FMM pass names.
function sample_phase(frames)
    # Julia's "sleeping" metadata does not mark blocked GC service threads or
    # every scheduler wait. Identify explicit waits first, because their outer
    # frames can still name an FMM pass waiting for its workers to finish.
    for frame in frames
        func, file = string(frame.func), string(frame.file)
        (occursin("pthread_cond_wait", func) || occursin("pthread_cond_timedwait", func) ||
         occursin("uv_cond_wait", func) || occursin("uv_sem_wait", func) ||
         occursin("epoll_wait", func) || occursin("jl_task_get_next", func) ||
         occursin("jl_gc_wait_for_the_world", func)) && return "runtime_wait"
        if (endswith(file, "task.jl") || endswith(file, "condition.jl")) &&
                func in ("wait", "_wait", "wait_forever")
            return "runtime_wait"
        end
    end
    for frame in frames
        func, file = string(frame.func), string(frame.file)
        occursin("source_to_buffer", func) && return "source_conversion"
        (occursin("upward_pass", func) || occursin("body_to_multipole", func) ||
         occursin("multipole_to_multipole", func) || occursin("execute_m2m", func)) && return "upward"
        (occursin("nearfield", func) || occursin("execute_assignment", func) ||
         (func == "direct!" && (occursin("FastMultipole", file) || occursin("FLOWVPM", file)))) && return "nearfield"
        (occursin("horizontal_pass", func) || occursin("execute_m2l", func) ||
         occursin("multipole_to_local", func)) && return "horizontal"
        (occursin("downward_pass", func) || occursin("execute_l2l", func) ||
         occursin("local_to_local", func) || occursin("local_to_body", func)) && return "downward"
        (occursin("build_interaction", func) || occursin("sort_by_target", func) ||
         occursin("interaction_list", file)) && return "interaction_lists"
        occursin("buffer_to_target", func) && return "target_export"
        occursin("FastMultipole", file) && endswith(file, "tree.jl") && return "tree_construction"
    end
    any(frame -> occursin("FastMultipole", string(frame.file)), frames) && return "fmm_other"
    any(frame -> occursin("/MPI/", string(frame.file)) ||
                 occursin("MPI_", string(frame.func)), frames) && return "mpi"
    return "other"
end

function phase_samples(data, lidict)
    # The portable Profile.retrieve data contains four metadata words and two
    # terminating zeros per snapshot in the pinned Julia 1.13 runtime. Decode
    # using Profile's own offsets/end-marker predicate to retain thread IDs and
    # distinguish metadata-marked sleeping workers from the sampled call stacks.
    counts = Dict{Tuple{Int,String},Int}()
    start = firstindex(data)
    for stop in eachindex(data)
        Profile.is_block_end(data, stop) || continue
        thread = Int(data[stop - Profile.META_OFFSET_THREADID])
        sleeping = data[stop - Profile.META_OFFSET_SLEEPSTATE] == 2
        frames = Base.StackTraces.StackFrame[]
        for ip in view(data, start:stop - Profile.nmeta - 2)
            ip == 0 && continue
            append!(frames, get(lidict, ip, Base.StackTraces.StackFrame[]))
        end
        phase = sleeping ? "sleeping" : sample_phase(frames)
        key = (thread, phase)
        counts[key] = get(counts, key, 0) + 1
        start = stop + 1
    end
    return counts
end

function save_profile(output, rank, data, lidict, delay, buffer, seconds, peak_rss)
    directory = joinpath(output, @sprintf("profile-rank-%03d", rank))
    mkpath(directory)
    serialize(joinpath(directory, "samples.jls"), (; data, lidict, julia_version=string(VERSION)))
    # C frames identify MPI waits and GC/native work. The Julia-only report is
    # easier to scan for source conversion and the FMM's threaded pass kernels.
    for (filename, cframes, format, grouping) in
            (("flat.txt", false, :flat, :none), ("flat-c.txt", true, :flat, :none),
             ("flat-by-thread.txt", true, :flat, :thread), ("tree.txt", true, :tree, :none))
        open(joinpath(directory, filename), "w") do io
            Profile.print(IOContext(io, :displaysize => (24, 240)), data, lidict;
                format, C=cframes, groupby=grouping, sortedby=:count,
                maxdepth=40, mincount=5)
        end
    end
    counts = phase_samples(data, lidict)
    total_samples = sum(values(counts); init=0)
    thread_samples = Dict(thread => sum(value for ((tid, _), value) in counts if tid == thread)
        for thread in unique(first(key) for key in keys(counts)))
    write_csv(joinpath(directory, "phase-samples.csv"),
        ["thread", "phase", "samples", "requested_delay_times_samples", "fraction_thread_samples", "fraction_all_samples"],
        ((thread, phase, count, delay * count, count / thread_samples[thread],
            total_samples == 0 ? 0.0 : count / total_samples)
            for ((thread, phase), count) in sort(collect(counts); by=first)))
    metadata = Dict(
        "rank" => rank, "hostname" => gethostname(), "local_seconds" => seconds,
        "threads" => Threads.nthreads(), "profile_delay_seconds" => delay,
        "worker_thread_bindings" => isnothing(benchmark_thread_bindings[]) ? [] : benchmark_thread_bindings[],
        "profile_buffer_instruction_pointers_per_thread" => buffer,
        "profile_buffer_full" => Profile.is_buffer_full(),
        "profile_snapshot_words" => length(data), "total_samples" => total_samples,
        "peak_rss_bytes_before_profile_export" => peak_rss,
        "julia_version" => string(VERSION),
        "profile_script_sha256" => bytes2hex(sha256(read(@__FILE__))),
        "phase_measurement" => "Exclusive statistical stack classification, including metadata-marked sleeping threads and explicit scheduler/runtime waits. Counts and fractions describe sampled stacks, not active CPU time. Requested delay times sample count is a nominal weight, not measured elapsed time; actual sampling intervals may differ. MPI calls may compute or wait. Flat reports contain inclusive overlapping function counts.",
        "sampling_scope" => "Complete warmed simulation including deferred-GC cleanup; excludes construction, artifact export and post-run MPI statistics.")
    open(joinpath(directory, "profile.toml"), "w") do io
        TOML.print(io, metadata; sorted=true)
    end
end

function profile_main(args=ARGS)
    MPI.Init()
    try
        config, delay, buffer = profile_options(args)
        rank, nranks = MPI.Comm_rank(MPI.COMM_WORLD), MPI.Comm_size(MPI.COMM_WORLD)
        config.mode == "serial" && nranks != 1 && error("Serial mode requires one process")
        BLAS.set_num_threads(1)
        configure_thread_pinning!(config, MPI.COMM_WORLD)
        function backend()
            config.mode == "mpi" || return nothing
            # Keep the original target-partition profile runnable against the
            # baseline checkout, whose constructor has no optimization keywords.
            options = (; check_state=config.check_state)
            if hasproperty(config, :partition) && config.partition != "targets"
                options = merge(options, (; partition=Symbol(config.partition)))
            end
            if hasproperty(config, :target_groups) && config.target_groups != 1
                options = merge(options, (; target_groups=config.target_groups))
            end
            if hasproperty(config, :source_order) && config.source_order != "contiguous"
                options = merge(options, (; source_order=Symbol(config.source_order)))
            end
            if hasproperty(config, :nearfield) && config.nearfield != "static"
                options = merge(options, (; nearfield=Symbol(config.nearfield)))
            end
            if hasproperty(config, :callback_sync) && config.callback_sync != "full"
                options = merge(options, (; callback_sync=Symbol(config.callback_sync)))
            end
            if hasproperty(config, :rotor_parallel) && config.rotor_parallel
                options = merge(options, (; rotor_parallel=true))
            end
            return uns.ReplicatedMPI(; options...)
        end
        warmup = make_case(config, config.warmup_steps)
        warm_backend = backend()
        MPI.Barrier(MPI.COMM_WORLD)
        warmup_seconds = @elapsed run_case(warmup, config.warmup_steps, warm_backend, config.gc_policy)
        warmup_seconds = MPI.Allreduce(warmup_seconds, max, MPI.COMM_WORLD)
        warmup = nothing
        case = make_case(config, config.nsteps)
        timed_backend = backend()
        Profile.init(; n=buffer, delay)
        Profile.clear()
        GC.gc()
        MPI.Barrier(MPI.COMM_WORLD)
        pfield = nothing
        local_seconds = @elapsed Profile.@profile pfield = run_case(case, config.nsteps, timed_backend, config.gc_policy)
        peak_rss = Int(Sys.maxrss())
        local_profile = (; rank, seconds=local_seconds, maxrss_bytes=peak_rss,
            hostname=gethostname(), threads=Threads.nthreads(),
            worker_thread_bindings=isnothing(benchmark_thread_bindings[]) ? [] : benchmark_thread_bindings[],
            phase_profile_available=!isnothing(timed_backend),
            evaluations=isnothing(timed_backend) ? 0 : timed_backend.evaluations,
            evaluation_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.evaluation_seconds,
            exchange_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.exchange_seconds,
            sfs_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.sfs_seconds,
            probe_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.probe_seconds,
            callback_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.callback_seconds,
            rotor_seconds=isnothing(timed_backend) || !hasproperty(timed_backend, :rotor_seconds) ?
                0.0 : timed_backend.rotor_seconds)
        profiles = MPI.gather(local_profile, MPI.COMM_WORLD; root=0)
        seconds = MPI.Allreduce(local_seconds, max, MPI.COMM_WORLD)
        stats = isnothing(timed_backend) ? nothing : uns.parallel_statistics(timed_backend)
        # Retrieve after leaving the deferred-GC region and with all FMM workers
        # finished. Each rank writes an independent directory on shared storage.
        data, lidict = Profile.retrieve()
        save_profile(config.output, rank, data, lidict, delay, buffer, local_seconds, peak_rss)
        rank == 0 && save_result(config, case, pfield, warmup_seconds, seconds, stats, profiles)
        MPI.Barrier(MPI.COMM_WORLD)
        MPI.Finalize()
    catch exception
        showerror(stderr, exception, catch_backtrace())
        println(stderr)
        MPI.Abort(MPI.COMM_WORLD, 1)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    profile_main()
end
