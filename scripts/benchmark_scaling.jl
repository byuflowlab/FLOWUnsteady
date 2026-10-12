#!/usr/bin/env julia
# Reuse imported processes for tuning iterations. Each layout still warms a
# complete case and measures a new case. Confirm promising results separately
# with the ordinary fresh-process rotorhover harness before reporting scaling.
include(joinpath(@__DIR__, "..", "examples", "mpi", "rotorhover.jl"))

function batch_options(args)
    options = Dict("repeats" => "3", "partitions" => "all", "ranks-per-node" => "1",
        "schedulers" => "static")
    case_args = String[]
    iseven(length(args)) || error("Arguments must be --name value pairs")
    for i in 1:2:length(args)
        name, value = args[i], args[i + 1]
        startswith(name, "--") || error("Expected --name, received $name")
        if haskey(options, name[3:end])
            options[name[3:end]] = value
        else
            append!(case_args, [name, value])
        end
    end
    repeats = parse(Int, options["repeats"])
    ranks_per_node = parse(Int, options["ranks-per-node"])
    repeats > 0 && ranks_per_node > 0 || error("Repeats and ranks per node must be positive")
    partitions = options["partitions"]
    partitions in ("all", "both", "targets", "sources", "hybrid") || error("--partitions must be all, both, targets, sources or hybrid")
    schedulers = options["schedulers"]
    schedulers in ("both", "static", "dynamic") || error("--schedulers must be both, static or dynamic")
    return (; config=parse_options(case_args), repeats, ranks_per_node, partitions, schedulers)
end

function layout_ranks(hosts, nodes, ranks_per_node; serial=false)
    serial && return [0]
    host_order = unique(hosts)
    if length(host_order) >= 3
        ranks = Int[]
        for host in host_order[1:nodes]
            candidates = findall(==(host), hosts)
            length(candidates) >= ranks_per_node || error("Host $host has too few ranks")
            append!(ranks, candidates[1:ranks_per_node] .- 1)
        end
        return ranks
    elseif length(host_order) == 1
        # Local regression pilot: logical node groups share the same host.
        count = nodes * ranks_per_node
        count <= length(hosts) || error("Launch at least $(3 * ranks_per_node) ranks")
        return collect(0:count-1)
    end
    error("Use three distinct cluster hosts, or one host for a local pilot")
end

function rank_profile(backend, local_seconds, comm)
    allowed_cpus = "unavailable"
    if isfile("/proc/self/status")
        lines = filter(line -> startswith(line, "Cpus_allowed_list:"), readlines("/proc/self/status"))
        isempty(lines) || (allowed_cpus = strip(split(first(lines), ':'; limit=2)[2]))
    end
    return (; rank=MPI.Comm_rank(comm), seconds=local_seconds,
        maxrss_bytes=Int(Sys.maxrss()), hostname=gethostname(), cpus_allowed_list=allowed_cpus,
        worker_thread_bindings=isnothing(benchmark_thread_bindings[]) ? [] : benchmark_thread_bindings[],
        threads=Threads.nthreads(), phase_profile_available=!isnothing(backend),
        evaluations=isnothing(backend) ? 0 : backend.evaluations,
        evaluation_seconds=isnothing(backend) ? 0.0 : backend.evaluation_seconds,
        exchange_seconds=isnothing(backend) ? 0.0 : backend.exchange_seconds,
        sfs_seconds=isnothing(backend) ? 0.0 : backend.sfs_seconds,
        probe_seconds=isnothing(backend) ? 0.0 : backend.probe_seconds,
        callback_seconds=isnothing(backend) ? 0.0 : backend.callback_seconds,
        rotor_seconds=isnothing(backend) ? 0.0 : backend.rotor_seconds)
end

function batch_backend(config, comm)
    config.mode == "serial" && return nothing
    nearfield = hasproperty(config, :nearfield) ? Symbol(config.nearfield) : :static
    callback_sync = hasproperty(config, :callback_sync) ? Symbol(config.callback_sync) : :full
    return uns.ReplicatedMPI(comm; check_state=config.check_state,
        partition=Symbol(config.partition), target_groups=config.target_groups,
        source_order=Symbol(config.source_order), nearfield, callback_sync,
        rotor_parallel=config.rotor_parallel)
end

function batch_layout(config, comm, benchmark_metadata)
    warmup = make_case(config, config.warmup_steps)
    warm_backend = batch_backend(config, comm)
    MPI.Barrier(comm)
    warmup_seconds = @elapsed run_case(warmup, config.warmup_steps, warm_backend, config.gc_policy)
    warmup_seconds = MPI.Allreduce(warmup_seconds, max, comm)
    isnothing(warm_backend) || close(warm_backend)
    case = make_case(config, config.nsteps)
    backend = batch_backend(config, comm)
    try
        GC.gc()
        MPI.Barrier(comm)
        local_seconds = @elapsed pfield = run_case(case, config.nsteps, backend, config.gc_policy)
        profile = rank_profile(backend, local_seconds, comm)
        profiles = MPI.gather(profile, comm; root=0)
        seconds = MPI.Allreduce(local_seconds, max, comm)
        stats = isnothing(backend) ? nothing : uns.parallel_statistics(backend)
        if MPI.Comm_rank(comm) == 0
            save_result(config, case, pfield, warmup_seconds, seconds, stats, profiles;
                comm, benchmark_metadata)
            flush(stdout)
        end
    finally
        isnothing(backend) || close(backend)
    end
    return nothing
end

function scaling_main(args=ARGS)
    MPI.Init()
    world = MPI.COMM_WORLD
    try
        options = batch_options(args)
        config = options.config
        rank, nranks = MPI.Comm_rank(world), MPI.Comm_size(world)
        nranks >= 3 * options.ranks_per_node || error("Launch at least $(3 * options.ranks_per_node) ranks")
        BLAS.set_num_threads(1)
        configure_thread_pinning!(config, world)
        hosts = MPI.bcast(MPI.gather(gethostname(), world; root=0), world; root=0)
        layouts = [(; mode="serial", partition="targets", nearfield="static", nodes=1)]
        partitions = options.partitions == "all" ? ("sources", "targets", "hybrid") :
            options.partitions == "both" ? ("sources", "targets") : (options.partitions,)
        for nodes in (3, 1, 2), partition in partitions, nearfield in ("static", "dynamic")
            options.schedulers in ("both", nearfield) || continue
            push!(layouts, (; mode="mpi", partition, nearfield, nodes))
        end
        outputs = String[]
        harness_hash = bytes2hex(sha256(read(@__FILE__)))
        for repeat in 1:options.repeats
            order = circshift(layouts, 1 - repeat)
            for (position, layout) in enumerate(order)
                active_ranks = layout_ranks(hosts, layout.nodes, options.ranks_per_node;
                    serial=layout.mode == "serial")
                active = rank in active_ranks
                label = "$(layout.mode)-$(layout.partition)-$(layout.nearfield)-$(layout.nodes)-r$(length(active_ranks))-rep$repeat"
                output = joinpath(config.output, label)
                push!(outputs, output)
                run_config = merge(config, (; mode=layout.mode, partition=layout.partition,
                    target_groups=layout.partition == "hybrid" ? options.ranks_per_node : 1,
                    source_order=layout.partition in ("sources", "hybrid") ? config.source_order : "contiguous",
                    nearfield=layout.nearfield, output))
                metadata = Dict{String,Any}(
                    "benchmark_process_reuse" => true,
                    "benchmark_harness_sha256" => harness_hash,
                    "benchmark_repeat" => repeat, "benchmark_layout_position" => position,
                    "benchmark_requested_nodes" => layout.nodes,
                    "benchmark_world_ranks" => nranks,
                    "benchmark_active_world_ranks" => join(active_ranks, ','),
                    "benchmark_local_pilot" => length(unique(hosts)) == 1,
                    "benchmark_context" => "Tuning with reused processes; full warmup and fresh case per layout; cumulative lifetime RSS; confirm with ordinary fresh-process runs")
                rank == 0 && println("SCALING layout=$label active_ranks=$active_ranks")
                rank == 0 && flush(stdout)
                MPI.Barrier(world)
                group = MPI.Comm_split(world, active ? 0 : nothing, rank)
                if active
                    batch_layout(run_config, group, metadata)
                    MPI.free(group)
                end
                MPI.Barrier(world)
            end
        end
        if rank == 0
            length(outputs) > 1 || error("No candidate layouts were produced")
            script = joinpath(@__DIR__, "compare_mpi_rotorhover.py")
            comparison = joinpath(config.output, "comparison.json")
            run(`python3 $script $(first(outputs)) $(outputs[2:end]) --output $comparison`)
            println("Strict scaling comparisons passed: $comparison")
        end
        MPI.Barrier(world)
        MPI.Finalize()
    catch exception
        showerror(stderr, exception, catch_backtrace())
        println(stderr)
        MPI.Abort(world, 1)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    scaling_main()
end
