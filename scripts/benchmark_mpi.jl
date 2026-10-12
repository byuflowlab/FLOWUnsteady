#!/usr/bin/env julia
using Random, Printf, LinearAlgebra
import FLOWUnsteady as uns
const MPI, vpm = uns.MPI, uns.vpm
MPI.Init()
rank, nranks = MPI.Comm_rank(MPI.COMM_WORLD), MPI.Comm_size(MPI.COMM_WORLD)
BLAS.set_num_threads(1)

try
    mode = length(ARGS) >= 1 ? ARGS[1] : "mpi"
    n = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 10000
    samples = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 5
    mode in ("serial", "mpi") || error("Mode must be serial or mpi")
    mode == "serial" && nranks != 1 && error("Run the serial baseline with one process")
    n > 0 && samples > 0 || error("Particle and sample counts must be positive")
    backend = uns.ReplicatedMPI()
    uj = mode == "mpi" ? uns._parallel_interaction(backend, vpm.UJ_fmm, vpm.Inviscid()) : vpm.UJ_fmm
    fmm = vpm.FMM(; p=8, ncrit=64, min_ncrit=64, relative_tolerance=1e-3,
        absolute_tolerance=1e-6, autotune_p=false, autotune_ncrit=false)
    pf = vpm.ParticleField(n, Float64; UJ=uj, fmm)
    rng = MersenneTwister(20261012)
    for i in 1:n
        # A deterministic elongated field for computational scaling, not a
        # validated aerodynamic wake or a choice of physical resolution.
        X = [20.0 * i / n, 0.2 * randn(rng), 0.2 * randn(rng)]
        vpm.add_particle(pf, X, 0.001 .* randn(rng, 3), 0.03)
    end
    for _ in 1:2
        uj(pf)
    end
    backend.evaluation_seconds = backend.exchange_seconds = backend.sfs_seconds = 0.0
    backend.probe_seconds = backend.callback_seconds = 0.0
    backend.evaluations = 0
    times = Float64[]
    for _ in 1:samples
        GC.gc()
        MPI.Barrier(MPI.COMM_WORLD)
        elapsed = @elapsed uj(pf)
        push!(times, MPI.Allreduce(elapsed, max, MPI.COMM_WORLD))
    end
    stats = uns.parallel_statistics(backend)
    if rank == 0
        println("mode,ranks,threads_per_rank,particles,samples,min_seconds,mean_seconds,evaluation_seconds,exchange_seconds")
        @printf("%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n", mode, nranks,
            Threads.nthreads(), n, samples, minimum(times), sum(times)/samples,
            stats.evaluation_seconds, stats.exchange_seconds)
    end
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(MPI.COMM_WORLD, 1)
end
