# Headless coupled example. Run on every rank with scripts/launch_mpi.jl.
using LinearAlgebra, Printf
import FLOWUnsteady as uns
const MPI = uns.MPI
MPI.Init()
BLAS.set_num_threads(1)

function make_simulation()
    wing = uns.vlm.simpleWing(2.0, 6.0, 1.0, 0.0, 20.0, 0.0; n=16)
    system = uns.vlm.WingSystem()
    uns.vlm.addwing(system, "Wing", wing)
    vehicle = uns.UVLMVehicle(system; vlm_system=system, wake_system=system)
    maneuver = uns.KinematicManeuver((), (), t -> zeros(3), t -> zeros(3))
    return uns.Simulation(vehicle, maneuver, 0.0, 0.0, 0.4)
end

try
    mode = length(ARGS) >= 1 ? ARGS[1] : "mpi"
    nsteps = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 32
    mode in ("serial", "mpi") || error("Mode must be serial or mpi")
    mode == "serial" && MPI.Comm_size(MPI.COMM_WORLD) != 1 && error("Serial mode requires one process")
    nsteps > 0 || error("nsteps must be positive")
    options = (Vinf=(X, t) -> [10.0, 0.0, 0.5], sigma_vlm_surf=0.05,
        sigma_rotor_surf=0.05, sigma_vpm_overwrite=0.1, vlm_init=true,
        max_particles=128 * (nsteps+1), prompt=false, verbose=false,
        vpm_fmm=uns.vpm.FMM(; p=8, ncrit=64, min_ncrit=64,
            relative_tolerance=1e-3, absolute_tolerance=1e-6,
            autotune_p=false, autotune_ncrit=false))
    # Warm the same path; construct fresh simulation and backend for measurement.
    uns.run_simulation(make_simulation(), nsteps; options...,
        parallel=mode == "mpi" ? uns.ReplicatedMPI() : nothing)
    backend = mode == "mpi" ? uns.ReplicatedMPI() : nothing
    simulation = make_simulation()
    GC.gc()
    MPI.Barrier(MPI.COMM_WORLD)
    elapsed = @elapsed pfield = uns.run_simulation(simulation, nsteps;
        options..., parallel=backend)
    elapsed = MPI.Allreduce(elapsed, max, MPI.COMM_WORLD)
    stats = isnothing(backend) ? nothing : uns.parallel_statistics(backend)
    if MPI.Comm_rank(MPI.COMM_WORLD) == 0
        println("mode,ranks,threads_per_rank,steps,particles,seconds")
        @printf("%s,%d,%d,%d,%d,%.6f\n", mode, MPI.Comm_size(MPI.COMM_WORLD),
            Threads.nthreads(), nsteps, pfield.np, elapsed)
        isnothing(stats) || println(stats)
    end
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(MPI.COMM_WORLD, 1)
end
