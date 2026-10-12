#!/usr/bin/env julia
# Headless DJI 9443 hover comparison, derived from the published very-low-fidelity
# case: https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-aero/
# Run directly for serial, or on every rank with the configured MPI launcher.
# Example: julia --project=environments/mpi examples/mpi/rotorhover.jl 
#   --mode serial --n 8 --revolutions 4 --steps-per-rev 36 --p-per-step 2 --output /tmp/hover-serial
using LinearAlgebra, Printf, TOML, SHA, Pkg
using Sockets: gethostname
import FLOWUnsteady as uns
const MPI, vlm, vpm = uns.MPI, uns.vlm, uns.vpm

function parse_options(args)
    options = Dict("mode" => "mpi", "n" => "8", "revolutions" => "4",
        "steps-per-rev" => "36", "p-per-step" => "2", "output" => "",
        "warmup-steps" => "", "check-state" => "false", "gc-policy" => "automatic")
    iseven(length(args)) || error("Arguments must be --name value pairs")
    for i in 1:2:length(args)
        startswith(args[i], "--") || error("Expected --name, received $(args[i])")
        name = args[i][3:end]
        haskey(options, name) || error("Unknown option --$name")
        options[name] = args[i + 1]
    end
    mode = options["mode"]
    mode in ("serial", "mpi") || error("--mode must be serial or mpi")
    n = parse(Int, options["n"])
    revolutions = parse(Float64, options["revolutions"])
    steps_per_rev = parse(Int, options["steps-per-rev"])
    p_per_step = parse(Int, options["p-per-step"])
    n >= 2 && steps_per_rev >= 4 && p_per_step >= 1 && revolutions > 0 ||
        error("Require n>=2, steps-per-rev>=4, p-per-step>=1, revolutions>0")
    nsteps = round(Int, revolutions * steps_per_rev)
    isapprox(nsteps, revolutions * steps_per_rev; atol=1e-10, rtol=0) && nsteps >= 1 ||
        error("revolutions * steps-per-rev must be a positive integer")
    warmup_steps = isempty(options["warmup-steps"]) ? nsteps : parse(Int, options["warmup-steps"])
    warmup_steps >= 2 || error("Use at least two warmup steps")
    isempty(options["output"]) && error("Provide a unique --output directory")
    gc_policy = options["gc-policy"]
    gc_policy in ("automatic", "deferred") || error("--gc-policy must be automatic or deferred")
    return (; mode, n, revolutions, steps_per_rev, p_per_step, nsteps,
        warmup_steps, gc_policy, output=abspath(options["output"]),
        check_state=parse(Bool, options["check-state"]))
end

function make_case(config, nsteps)
    RPM, J = 5400.0, 0.0001
    rho, mu, sound_spd = 1.071778, 1.85508e-5, 342.35
    R, B = uns.read_rotor("DJI9443.csv"; data_path=uns.def_data_path)[[1, 3]]
    rotor = uns.generate_rotor("DJI9443.csv"; pitch=0.0, n=config.n,
        CW=false, blade_r=1/10, altReD=[RPM, J, mu/rho], xfoil=false,
        read_polar=vlm.ap.read_polar2, data_path=uns.def_data_path,
        verbose=false, plot_disc=false)
    system, wake_system = vlm.WingSystem(), vlm.WingSystem()
    vlm.addwing(system, "Rotor", rotor)
    vlm.addwing(wake_system, "Rotor", rotor)
    vehicle = uns.UVLMVehicle(system; rotor_systems=([rotor],), wake_system)
    maneuver = uns.KinematicManeuver((), (t -> 1.0,), t -> zeros(3), t -> zeros(3))
    ttot = nsteps / config.steps_per_rev / (RPM / 60)
    simulation = uns.Simulation(vehicle, maneuver, 0.0, RPM, ttot;
        Vinit=zeros(3), Winit=zeros(3))
    omit_shedding = Int[]
    histories = Vector{NamedTuple}()
    blade_histories = Vector{NamedTuple}()
    function callback(sim, pfield, t, dt; kwargs...)
        thrust, torque = vlm.calc_thrust_torque(rotor)
        CT, CQ = vlm.calc_thrust_torque_coeffs(rotor, rho)
        push!(histories, (; step=sim.nt, time=t, dt, particles=pfield.np,
            thrust, torque, CT, CQ, hub_omitted=count(>(0), omit_shedding)))
        for blade in 1:B, element in 1:config.n
            push!(blade_histories, (; step=sim.nt, time=t, blade, element,
                Gamma=vlm.get_blade(rotor, blade).sol["Gamma"][element],
                Np=rotor.sol["Np"]["field_data"][blade][element],
                Tp=rotor.sol["Tp"]["field_data"][blade][element]))
        end
        # Preserve the tutorial's after-step timing of hub suppression. The MPI
        # driver must synchronize this externally captured mutable keyword vector
        # after its root-only callback, as well as simulation/particle state.
        if sim.nt == 1
            for i in 1:vlm.get_m(rotor)
                CP = vlm.getHorseshoe(rotor, i)[5]
                norm(CP - vlm._get_O(rotor)) <= 0.35R && push!(omit_shedding, i)
            end
        end
        sim.nt == 3 * config.steps_per_rev && (omit_shedding .= -1)
        return false
    end
    Vinf = (X, t) -> [J * RPM / 60 * 2R, 0.0, 0.0]
    fmm = vpm.FMM(; p=12, ncrit=64, min_ncrit=64,
        relative_tolerance=1e-7, absolute_tolerance=1e-10,
        autotune_p=false, autotune_ncrit=false)
    options = (; Vinf, rho, mu, sound_spd, p_per_step=config.p_per_step,
        max_particles=((2 * config.n + 1) * B) * (nsteps + 1) * config.p_per_step + 4 * config.n * B + 16,
        vpm_integration=vpm.euler, vpm_viscous=vpm.Inviscid(), vpm_SFS=vpm.SFS_none,
        vpm_fmm=fmm, sigma_vlm_surf=R/10, sigma_rotor_surf=R/10,
        sigma_vpm_overwrite=2.125 * 2pi * R / (config.steps_per_rev * config.p_per_step),
        sigmafactor_vpmonvlm=1.0, vlm_rlx=0.5,
        hubtiploss_correction=((0.4, 5, 0.1, 0.05), (2, 1, 0.25, 0.05)),
        shed_starting=false, shed_unsteady=true, unsteady_shedcrit=0.001,
        omit_shedding, extra_runtime_function=callback,
        save_path=nothing, save_wopwopin=false, prompt=false, verbose=false,
        raisewarnings=false)
    return (; simulation, rotor, histories, blade_histories, options, R, B, RPM, rho)
end

function run_case(case, nsteps, parallel, gc_policy)
    gc_policy == "automatic" && return uns.run_simulation(case.simulation, nsteps;
        case.options..., parallel)
    # Opt-in workaround for PyCall foreign-object finalizers during threaded
    # work. Appropriate only for bounded pilots: deferred garbage accumulates
    # during the complete run. Restore policy and collect on the main thread
    # before returning, so cleanup remains included in the measured interval.
    previous = GC.enable(false)
    try
        return uns.run_simulation(case.simulation, nsteps; case.options..., parallel)
    finally
        GC.enable(previous)
        previous && GC.gc()
    end
end

function write_csv(path, header, rows)
    open(path, "w") do io
        println(io, join(header, ','))
        for row in rows
            for (i, value) in enumerate(row)
                i > 1 && print(io, ',')
                if value isa AbstractString
                    print(io, '"', replace(value, "\"" => "\"\""), '"')
                elseif value isa Integer
                    print(io, value)
                else
                    @printf(io, "%.17g", value)
                end
            end
            println(io)
        end
    end
end

function save_result(config, case, pfield, warmup_seconds, seconds, stats, profiles)
    mkpath(config.output)
    write_csv(joinpath(config.output, "particles.csv"),
        vcat(["particle"], ["state_$i" for i in axes(pfield.particles, 1)]),
        ((i, view(pfield.particles, :, i)...) for i in 1:pfield.np))
    write_csv(joinpath(config.output, "history.csv"),
        ["step", "time", "dt", "particles", "thrust", "torque", "CT", "CQ", "hub_omitted"],
        (values(row) for row in case.histories))
    write_csv(joinpath(config.output, "blade_history.csv"),
        ["step", "time", "blade", "element", "Gamma", "Np", "Tp"],
        (values(row) for row in case.blade_histories))
    write_csv(joinpath(config.output, "blade.csv"), ["blade", "element", "Gamma", "Np", "Tp"],
        ((blade, element, vlm.get_blade(case.rotor, blade).sol["Gamma"][element],
            case.rotor.sol["Np"]["field_data"][blade][element],
            case.rotor.sol["Tp"]["field_data"][blade][element])
            for blade in 1:case.B for element in 1:config.n))
    write_csv(joinpath(config.output, "rank_times.csv"),
        string.(collect(keys(first(profiles)))), (values(profile) for profile in profiles))
    repository = normpath(joinpath(@__DIR__, "..", ".."))
    metadata = Dict{String,Any}(
        "format_version" => 1, "mode" => config.mode, "ranks" => MPI.Comm_size(MPI.COMM_WORLD),
        "threads_per_rank" => Threads.nthreads(), "blas_threads" => BLAS.get_num_threads(),
        "nodes" => length(unique(profile.hostname for profile in profiles)),
        "max_rank_peak_rss_bytes" => maximum(profile.maxrss_bytes for profile in profiles),
        "sum_rank_peak_rss_bytes" => sum(profile.maxrss_bytes for profile in profiles),
        "rss_scope" => "Process lifetime peak including imports, compilation, warmup and selected GC policy; sampled before export",
        "seconds" => seconds, "warmup_seconds" => warmup_seconds,
        "warmup_steps" => config.warmup_steps, "n" => config.n,
        "revolutions" => config.revolutions, "steps_per_rev" => config.steps_per_rev,
        "p_per_step" => config.p_per_step, "nsteps" => config.nsteps,
        "particles" => pfield.np, "particle_fields" => size(pfield.particles, 1),
        "particle_time" => pfield.t, "particle_step" => pfield.nt,
        "simulation_time" => case.simulation.t, "simulation_step" => case.simulation.nt,
        "radius" => case.R, "blades" => case.B, "RPM" => case.RPM, "rho" => case.rho,
        "J" => 0.0001, "integration" => "Euler", "viscous" => "Inviscid", "SFS" => "none",
        "fmm_p" => 12, "fmm_ncrit" => 64, "fmm_relative_tolerance" => 1e-7,
        "fmm_absolute_tolerance" => 1e-10, "check_state" => config.check_state,
        "gc_policy" => config.gc_policy,
        "julia_version" => string(VERSION), "comparison_l2" => 1e-5,
        "comparison_max" => 5e-5, "comparison_scale_floor" => 1e-12,
        "tutorial" => "https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-aero/",
        "script_sha256" => bytes2hex(sha256(read(@__FILE__))),
        "solver_sha256" => bytes2hex(sha256(vcat([vcat(codeunits(basename(path)), UInt8[0], read(path))
            for path in sort(filter(path -> endswith(path, ".jl"),
                readdir(joinpath(repository, "src"); join=true)))]...))))
    metadata["packages"] = Dict(info.name => Dict("version" => string(info.version),
        "tree_hash" => string(info.tree_hash)) for info in values(Pkg.dependencies())
        if info.name in ("FLOWUnsteady", "FLOWVLM", "FLOWVPM", "FastMultipole", "MPI", "MPIPreferences"))
    # Scalar CSV permits comparison using any Python 3 standard library; TOML
    # additionally retains package identity and structured phase statistics.
    write_csv(joinpath(config.output, "metadata.csv"), ["key", "value"],
        ((key, value isa AbstractString ? value : string(value))
            for (key, value) in sort(collect(metadata); by=first)
            if value isa Union{Number,AbstractString}))
    metadata["statistics"] = isnothing(stats) ? Dict("available" => false) :
        Dict{String,Any}("available" => true, (string(key) => value for (key, value) in pairs(stats))...)
    metadata["mpi_library_version"] = MPI.Get_library_version()
    metadata["rank_profiles"] = [Dict(string(key) => value for (key, value) in pairs(profile))
        for profile in profiles]
    open(joinpath(config.output, "metadata.toml"), "w") do io
        TOML.print(io, metadata; sorted=true)
    end
    println("mode,ranks,threads_per_rank,steps,particles,warmup_seconds,seconds,output")
    @printf("%s,%d,%d,%d,%d,%.6f,%.6f,%s\n", config.mode,
        MPI.Comm_size(MPI.COMM_WORLD), Threads.nthreads(), config.nsteps,
        pfield.np, warmup_seconds, seconds, config.output)
end

function main(args=ARGS)
    MPI.Init()
    try
        config = parse_options(args)
        config.mode == "serial" && MPI.Comm_size(MPI.COMM_WORLD) != 1 &&
            error("Serial mode requires one process")
        BLAS.set_num_threads(1)
        backend() = config.mode == "mpi" ? uns.ReplicatedMPI(; check_state=config.check_state) : nothing
        warmup = make_case(config, config.warmup_steps)
        warm_backend = backend()
        MPI.Barrier(MPI.COMM_WORLD)
        warmup_seconds = @elapsed run_case(warmup, config.warmup_steps, warm_backend, config.gc_policy)
        warmup_seconds = MPI.Allreduce(warmup_seconds, max, MPI.COMM_WORLD)
        case = make_case(config, config.nsteps)
        timed_backend = backend()
        GC.gc()
        MPI.Barrier(MPI.COMM_WORLD)
        local_seconds = @elapsed pfield = run_case(case, config.nsteps, timed_backend, config.gc_policy)
        # Sys.maxrss() reports bytes, and is a lifetime high-water mark rather
        # than timed-phase-only usage. Capture it before CSV/state export.
        peak_rss = Int(Sys.maxrss())
        allowed_cpus = "unavailable"
        if isfile("/proc/self/status")
            lines = filter(line -> startswith(line, "Cpus_allowed_list:"), readlines("/proc/self/status"))
            isempty(lines) || (allowed_cpus = strip(split(first(lines), ':'; limit=2)[2]))
        end
        local_profile = (; rank=MPI.Comm_rank(MPI.COMM_WORLD), seconds=local_seconds,
            maxrss_bytes=peak_rss, hostname=gethostname(), cpus_allowed_list=allowed_cpus,
            threads=Threads.nthreads(), phase_profile_available=!isnothing(timed_backend),
            evaluations=isnothing(timed_backend) ? 0 : timed_backend.evaluations,
            evaluation_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.evaluation_seconds,
            exchange_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.exchange_seconds,
            sfs_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.sfs_seconds,
            probe_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.probe_seconds,
            callback_seconds=isnothing(timed_backend) ? 0.0 : timed_backend.callback_seconds)
        profiles = MPI.gather(local_profile, MPI.COMM_WORLD; root=0)
        seconds = MPI.Allreduce(local_seconds, max, MPI.COMM_WORLD)
        stats = isnothing(timed_backend) ? nothing : uns.parallel_statistics(timed_backend)
        if MPI.Comm_rank(MPI.COMM_WORLD) == 0
            save_result(config, case, pfield, warmup_seconds, seconds, stats, profiles)
        end
        MPI.Finalize()
    catch exception
        showerror(stderr, exception, catch_backtrace())
        println(stderr)
        MPI.Abort(MPI.COMM_WORLD, 1)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
