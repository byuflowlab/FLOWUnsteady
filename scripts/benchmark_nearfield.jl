#!/usr/bin/env julia
# Frozen-wake diagnostic. Simulated ranks execute sequentially on this host;
# timings contain FMM evaluation only, with no MPI exchange or coupled stepping.
# julia --project=environments/mpi scripts/benchmark_nearfield.jl RESULT_DIR OUTPUT_CSV [SAMPLES] [SIMULATED_NODES]
using Statistics
include(joinpath(@__DIR__, "..", "examples", "mpi", "rotorhover.jl"))

function frozen_field(directory)
    metadata = TOML.parsefile(joinpath(directory, "metadata.toml"))
    rows = uns.CSV.File(joinpath(directory, "particles.csv"))
    fmm = vpm.FMM(; p=metadata["fmm_p"], ncrit=metadata["fmm_ncrit"],
        min_ncrit=metadata["fmm_ncrit"], relative_tolerance=metadata["fmm_relative_tolerance"],
        absolute_tolerance=metadata["fmm_absolute_tolerance"],
        autotune_p=false, autotune_ncrit=false)
    pf = vpm.ParticleField(length(rows) + 64, Float64; fmm)
    for (i, row) in enumerate(rows), j in axes(pf.particles, 1)
        pf.particles[j, i] = row[j + 1]
    end
    pf.np = length(rows)
    pf.t, pf.nt = metadata["particle_time"], metadata["particle_step"]
    # Match the normal reset=true particle interaction, with unchanged sources.
    vpm._reset_particles(pf)
    return pf, metadata
end

function frozen_benchmark(directory, output, samples, simulated_nodes)
    samples > 0 && simulated_nodes > 0 || error("Counts must be positive")
    BLAS.set_num_threads(1)
    pf, metadata = frozen_field(directory)
    before = copy(pf.particles)
    config = parse_options(["--output", "/tmp/unused-nearfield-case",
        "--n", string(metadata["n"]), "--revolutions", string(metadata["revolutions"]),
        "--steps-per-rev", string(metadata["steps_per_rev"]),
        "--p-per-step", string(metadata["p_per_step"])])
    case = make_case(config, config.nsteps)
    vlm.setVinf(case.simulation.vehicle.system, case.options.Vinf)
    vlm.setRPM(case.rotor, case.RPM)
    probe_positions = hcat(uns._get_midXs(case.simulation.vehicle.rotor_systems)...)
    records = NamedTuple[]
    for kind in (:particles, :probes), nodes in unique((1, simulated_nodes)),
            partition in (nodes == 1 ? (:targets,) : (:targets, :sources)), rank in 0:nodes-1
        ntargets = kind === :particles ? pf.np : size(probe_positions, 2)
        owned = partition === :targets ? uns._owned_targets(ntargets, rank, nodes) : (1:ntargets)
        sources = partition === :sources ? uns._owned_targets(pf.np, rank, nodes) : (1:pf.np)
        target = kind === :particles ? uns._interaction_targets(pf, owned) :
            uns._InteractionTargets(probe_positions[:, owned], zeros(length(owned)), zeros(3, length(owned)))
        times = Dict(:static => Float64[], :dynamic => Float64[])
        expected = nothing
        leaves, pairs = 0, 0
        for sample in -1:samples
            # Alternate order after both paths have compiled and warmed.
            for scheduler in (iseven(sample) ? (:static, :dynamic) : (:dynamic, :static))
                GC.gc()
                fill!(target.values, 0)
                previous = GC.enable(false)
                result = nothing
                elapsed = try
                    @elapsed result = uns._evaluate_targets!(target, pf, vpm.UJ_fmm;
                        gradient=kind === :particles, source_indices=sources, nearfield=scheduler)
                finally
                    GC.enable(previous)
                end
                if isnothing(expected)
                    expected = copy(target.values)
                else
                    isequal(target.values, expected) || error("Scheduler changed $kind values on $partition rank $rank")
                end
                sample > 0 && push!(times[scheduler], elapsed)
                if !isnothing(result)
                    pairs = length(result[6])
                    leaves = length(unique(pair[1] for pair in result[6]))
                    all(pair -> result[3].branches[pair[1]].n_branches == 0, result[6]) ||
                        error("Near-field list contains overlapping non-leaf targets")
                end
                isequal(pf.particles, before) || error("Frozen sources were modified")
            end
        end
        for scheduler in (:static, :dynamic)
            record = (; kind=string(kind), partition=string(partition), simulated_nodes=nodes,
                logical_rank=rank, scheduler=string(scheduler), threads=Threads.nthreads(),
                targets=length(owned), sources=length(sources), target_leaves=leaves,
                direct_pairs=pairs, samples, median_seconds=median(times[scheduler]),
                min_seconds=minimum(times[scheduler]), max_seconds=maximum(times[scheduler]),
                bitwise_equal=true)
            push!(records, record)
            println(record)
        end
    end
    mkpath(dirname(abspath(output)))
    write_csv(output, string.(keys(first(records))), (values(record) for record in records))
    open(output * ".toml", "w") do io
        TOML.print(io, Dict("frozen_result_directory" => abspath(directory),
            "particles_sha256" => bytes2hex(sha256(read(joinpath(directory, "particles.csv")))),
            "script_sha256" => bytes2hex(sha256(read(@__FILE__))),
            "scope" => "Sequential simulated ranks; FMM evaluation only; no MPI exchange or coupling",
            "threads" => Threads.nthreads(), "samples" => samples,
            "simulated_nodes" => simulated_nodes, "particles" => pf.np,
            "probe_geometry" => "DJI9443 blade midpoints at initial azimuth, reconstructed from the saved case settings",
            "fmm_p" => metadata["fmm_p"], "fmm_ncrit" => metadata["fmm_ncrit"],
            "fmm_relative_tolerance" => metadata["fmm_relative_tolerance"],
            "fmm_absolute_tolerance" => metadata["fmm_absolute_tolerance"],
            "all_static_dynamic_values_bitwise_equal" => true))
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    length(ARGS) in 2:4 || error("Usage: RESULT_DIR OUTPUT_CSV [SAMPLES] [SIMULATED_NODES]")
    frozen_benchmark(ARGS[1], ARGS[2], length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 5,
                     length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 3)
end
