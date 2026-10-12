using Test
include(joinpath(@__DIR__, "..", "..", "examples", "mpi", "rotorhover.jl"))

mutable struct ReadonlyTestState
    time::Float64
    values::Vector{Float64}
end

function callback_test_field()
    field = vpm.ParticleField(8, Float64; SFS=vpm.SFS_none)
    vpm.add_particle(field, [0.1, 0.2, 0.3], [0.01, -0.02, 0.03], 0.1)
    return field
end

function pilot_profile(config, case, field, backend)
    profile = (; rank=MPI.Comm_rank(MPI.COMM_WORLD), seconds=1.0,
        maxrss_bytes=Int(Sys.maxrss()), hostname=gethostname(), threads=Threads.nthreads(),
        phase_profile_available=!isnothing(backend),
        evaluations=isnothing(backend) ? 0 : backend.evaluations,
        evaluation_seconds=isnothing(backend) ? 0.0 : backend.evaluation_seconds,
        exchange_seconds=isnothing(backend) ? 0.0 : backend.exchange_seconds,
        sfs_seconds=isnothing(backend) ? 0.0 : backend.sfs_seconds,
        probe_seconds=isnothing(backend) ? 0.0 : backend.probe_seconds,
        callback_seconds=isnothing(backend) ? 0.0 : backend.callback_seconds)
    profiles = MPI.gather(profile, MPI.COMM_WORLD; root=0)
    stats = isnothing(backend) ? nothing : uns.parallel_statistics(backend)
    MPI.Comm_rank(MPI.COMM_WORLD) == 0 &&
        save_result(config, case, field, 0.0, 1.0, stats, profiles)
end

MPI.Init()
try
    comm = MPI.COMM_WORLD
    rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)
    BLAS.set_num_threads(1)
    @testset "Read-only callback contract ($nranks ranks)" begin
        @test uns.ReplicatedMPI().callback_sync === :full
        @test_throws ArgumentError uns.ReplicatedMPI(; callback_sync=:invalid)

        @testset "Serializable state comparison" begin
            state = ReadonlyTestState(0.0, [NaN, -0.0])
            value = (state, Dict("values" => state.values), [1, 2])
            before = uns._callback_state(value)
            @test uns._same_callback_state(before, uns._callback_state(value))
            state.values[2] = 0.0
            @test !uns._same_callback_state(before, uns._callback_state(value))
        end

        @testset "Root ownership, stopping and external shedding control" begin
            backend = uns.ReplicatedMPI(; root=nranks-1, callback_sync=:readonly, check_state=true)
            field = callback_test_field()
            state = ReadonlyTestState(0.0, [1.0])
            omitted, calls = Int[], Ref(0)
            function callback(sim, pf, t, dt; kwargs...)
                calls[] += 1
                @test !backend.active
                @test sim.time == 0.0 && pf.np == 1
                append!(omitted, [2, 3])
                return true
            end
            @test uns._parallel_callback(backend, callback, state, field, 0.0, 0.1;
                sync_from=1, sync_omit_shedding=omitted)
            @test calls[] == (rank == backend.root ? 1 : 0)
            @test omitted == [2, 3]
            @test backend.active
            @test backend.callback_seconds > 0
            @test !uns._parallel_callback(backend,
                (args...; kwargs...) -> (empty!(omitted); false), state, field, 0.0, 0.1;
                sync_omit_shedding=omitted)
            @test isempty(omitted)
            @test !uns._parallel_callback(backend, uns._runtime_noop, state, field, 0.0, 0.1)
        end

        @testset "Callback failures are collective and restore evaluator policy" begin
            for callback in ((args...; kwargs...) -> error("readonly callback failure"),
                             (args...; kwargs...) -> 1)
                backend = uns.ReplicatedMPI(; callback_sync=:readonly, check_state=true)
                @test_throws ErrorException uns._parallel_callback(backend, callback,
                    ReadonlyTestState(0.0, [1.0]), callback_test_field(), 0.0, 0.1)
                @test backend.active
            end
            if nranks == 1
                backend = uns.ReplicatedMPI(; callback_sync=:readonly)
                backend.active = false
                @test !uns._parallel_callback(backend, uns._runtime_noop,
                    ReadonlyTestState(0.0, [1.0]), callback_test_field(), 0.0, 0.1)
                @test !backend.active
            end
        end

        @testset "Checked mode rejects solver-state mutations" begin
            callbacks = (
                (sim, pf, args...; kwargs...) -> (sim.time += 1; false),
                (sim, pf, args...; kwargs...) -> (sim.values[1] += 1; false),
                (sim, pf, args...; kwargs...) -> (pf.particles[vpm.GAMMA_INDEX[1], 1] += 1; false),
                (sim, pf, args...; kwargs...) -> (pf.t += 1; false),
                (sim, pf, args...; kwargs...) -> (pf.fmm = vpm.FMM(; p=3); false),
                (sim, pf, args...; kwargs...) -> (vpm.add_particle(pf, zeros(3), ones(3), 0.1); false),
                (sim, pf, args...; kwargs...) -> (vpm.remove_particle(pf, 1); false))
            for callback in callbacks
                backend = uns.ReplicatedMPI(; callback_sync=:readonly, check_state=true)
                @test_throws ErrorException uns._parallel_callback(backend, callback,
                    ReadonlyTestState(0.0, [1.0]), callback_test_field(), 0.0, 0.1)
                @test backend.active
            end
            if nranks > 1
                backend = uns.ReplicatedMPI(; callback_sync=:readonly, check_state=true)
                field = callback_test_field()
                rank == 1 && (field.particles[1, 1] += 1)
                @test_throws ArgumentError uns._parallel_callback(backend, uns._runtime_noop,
                    ReadonlyTestState(0.0, [1.0]), field, 0.0, 0.1)
            end
        end

        if !("--skip-pilot" in ARGS)
            @testset "Complete DJI hover pilot against unchanged numerical acceptance" begin
                directory = MPI.bcast(rank == 0 ? mktempdir(; prefix="flow-readonly-", cleanup=false) : nothing,
                    comm; root=0)
                config(mode, partition, path) = merge(parse_options(["--mode", mode,
                    "--n", "8", "--revolutions", "4", "--steps-per-rev", "36",
                    "--p-per-step", "2", "--check-state", "true", "--gc-policy", "deferred",
                    "--output", path]), (; partition, callback_sync="readonly"))
                serial_config = config("serial", "targets", joinpath(directory, "serial"))
                serial_case = make_case(serial_config, serial_config.nsteps)
                reference = run_case(serial_case, serial_config.nsteps, nothing, "deferred")
                # The serial baseline is independently computed on every rank;
                # only its root artifact represents the single-process layout.
                if rank == 0
                    profile = (; rank=0, seconds=1.0, maxrss_bytes=Int(Sys.maxrss()),
                        hostname=gethostname(), threads=Threads.nthreads(),
                        phase_profile_available=false, evaluations=0,
                        evaluation_seconds=0.0, exchange_seconds=0.0, sfs_seconds=0.0,
                        probe_seconds=0.0, callback_seconds=0.0)
                    save_result(serial_config, serial_case, reference, 0.0, 1.0, nothing, [profile])
                    # save_result normally reads COMM_WORLD size. This test ran
                    # independent serial references on each process; the exported
                    # root reference represents one serial simulation/process.
                    metadata_path = joinpath(serial_config.output, "metadata.toml")
                    metadata = TOML.parsefile(metadata_path)
                    metadata["ranks"] = 1
                    metadata["test_reference_scope"] = "One independent serial simulation on rank zero; other ranks compute separate local references"
                    open(metadata_path, "w") do io
                        TOML.print(io, metadata; sorted=true)
                    end
                    write_csv(joinpath(serial_config.output, "metadata.csv"), ["key", "value"],
                        ((key, value isa AbstractString ? value : string(value))
                            for (key, value) in sort(collect(metadata); by=first)
                            if value isa Union{Number,AbstractString}))
                end
                candidates = String[]
                for partition in ("targets", "sources")
                    candidate_config = config("mpi", partition, joinpath(directory, partition))
                    candidate_case = make_case(candidate_config, candidate_config.nsteps)
                    backend = uns.ReplicatedMPI(; callback_sync=:readonly, check_state=true,
                        partition=Symbol(partition))
                    field = run_case(candidate_case, candidate_config.nsteps, backend, "deferred")
                    @test field.np == reference.np
                    @test field.nt == reference.nt && field.t == reference.t
                    @test field.particles[:, 1:field.np] ≈ reference.particles[:, 1:reference.np] rtol=1e-5 atol=1e-12
                    @test candidate_case.simulation.t == serial_case.simulation.t
                    @test candidate_case.simulation.nt == serial_case.simulation.nt
                    pilot_profile(candidate_config, candidate_case, field, backend)
                    push!(candidates, candidate_config.output)
                end
                MPI.Barrier(comm)
                if rank == 0
                    comparator = joinpath(@__DIR__, "..", "..", "scripts", "compare_mpi_rotorhover.py")
                    report = joinpath(directory, "comparison.json")
                    process = run(ignorestatus(`python3 $comparator $(serial_config.output) $candidates --output $report`))
                    @test success(process)
                    println("Read-only complete-pilot comparison artifacts: $directory")
                end
                MPI.Barrier(comm)
            end
        end
    end
    rank == 0 && println("Read-only callback checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(MPI.COMM_WORLD, 1)
end
