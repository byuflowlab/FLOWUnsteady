# MPI execution of one simulation

`ReplicatedMPI` divides interaction targets between MPI ranks while retaining the
complete wake on each rank. The first implementation uses `UVLMVehicle` and
supports the existing inviscid rVPM integrators,
relaxation, constant/dynamic SFS, wake shedding, and wing/rotor coupling. FMM uses
Float64; direct interactions also support Float32. This does not increase the
particle capacity of an individual node.

## Reproducible environment

The application environment in `environments/mpi` pins FLOWVPM and FastMultipole
by commit, together with the remaining dependencies. It was tested with Julia
1.13.1, Python 3.12 and MPI.jl 0.20.27. A Julia package version alone does not
identify the compatible FLOWVPM/FastMultipole API revisions.

From the repository root, prepare a Python environment and install the solver:

```bash
python3.12 -m venv environments/mpi/.venv
environments/mpi/.venv/bin/python -m pip install -r environments/mpi/requirements.txt
export PYTHON="$PWD/environments/mpi/.venv/bin/python"
export MPLBACKEND=Agg
julia --startup-file=no scripts/setup_mpi.jl
```

`uv venv --python 3.12 environments/mpi/.venv` and `uv pip install --python
environments/mpi/.venv/bin/python -r environments/mpi/requirements.txt` are
equivalent when `uv` is available. Keep the environment and code identical on all
nodes. Do not run package installation concurrently from the compute ranks.

## Simulation API

Execute the same simulation script on every MPI rank:

```julia
import FLOWUnsteady as uns
const MPI = uns.MPI
MPI.Init()

backend = uns.ReplicatedMPI(; check_state=true)
# Build the same fresh vehicle, maneuver and simulation on every rank.
pfield = uns.run_simulation(simulation, nsteps;
    parallel=backend,
    sigma_vlm_surf=sigma_vlm_surf,
    sigma_rotor_surf=sigma_rotor_surf,
    prompt=false,
    save_path="results")

statistics = uns.parallel_statistics(backend) # collective: call on every rank
MPI.Comm_rank(MPI.COMM_WORLD) == 0 && println(statistics)
MPI.Finalize()
```

Normal serial usage continues to use `parallel=nothing` and does not initialize
MPI. Wing/rotor velocity queries use separate targets on the pinned backend;
they no longer add probes or recompute wake-target derivatives. Custom interaction
functions and older serial installations retain their original query path.

The root rank alone writes files and executes `extra_runtime_function`. Particle
additions, deletions and edits, coupled simulation state, scheme state and the
callback's stop decision are propagated to the other ranks.
The driver's `omit_shedding` vector is also synchronized after callbacks, so the
tutorial's initial hub-wake suppression and subsequent release apply on all ranks.
Functions and native
airfoil/Python objects stay local to each rank. Callbacks must keep those models,
freestream functions and vehicle topology fixed. During that callback,
additional interaction queries run serially on the root. Callbacks must return a
`Bool`; they must not invoke their own MPI collectives. Worker state is restored
into its existing object graph so vehicle aliases remain intact. The active
particle field is replicated; callbacks should not
modify unused capacity slots or replace its interaction evaluator.

Set `check_state=true` while validating a case. It checks active particle values
before each collective evaluation; disable it for production timings to avoid
the extra full-field scan. All ranks must use identical target positions and
solver settings. An unexpected error during a multi-rank simulation aborts that
MPI job, preventing peers from remaining blocked in a collective.

## Numerical choices and limitations

- Velocity and all nine velocity-gradient components are gathered before SFS
  evaluation. RK3 and dynamic SFS use the same collective evaluator at their
  internal stages; synchronization is not deferred to the end of the timestep.
- FMM SFS uses the original full-field target tree's near-field neighborhoods.
  Partitioning targets must not change those SFS neighborhoods.
- Distributed FMM settings are fixed. Select expansion order, leaf size and
  tolerances from the serial baseline; distributed ranks do not independently
  autotune them. Different target trees can change summation order and FMM
  approximation error, so validate loads and wake observables rather than require
  bitwise equality.
- Core spreading/RBF redistribution, GPU execution, experimental mirrored
  ground effect, `sigmafactor_vpmonvlm != 1`, and arbitrary custom UJ functions are
  not supported by this MPI path. Unsupported options raise an error. The existing
  wake-file warm start remains a warm start; this adds no complete checkpoint.
- Quasi-steady vehicles are outside this first MPI implementation. The original
  quasi-steady driver currently rejects rotor-correction keywords passed by
  `run_simulation`; this is also present before the MPI changes.
- Replicated source trees, particle integration and vehicle solves still cost
  work on every rank. Root callbacks and output remain serial. More ranks are not
  guaranteed to improve elapsed time; measure the intended case and node layout.

## Local verification and cluster launch

Run the regression suite with the MPI launcher matching MPI.jl:

```bash
JULIA_NUM_THREADS=1 julia --startup-file=no --project=environments/mpi scripts/test_mpi.jl
# Also exercise threaded FMM within each rank:
JULIA_NUM_THREADS=2 julia --startup-file=no --project=environments/mpi scripts/test_mpi.jl 2
```

Tests cover direct/FMM velocity and gradients, uneven and empty partitions,
static sources, RK3, constant/dynamic SFS, surface queries, root callbacks,
particle mutation, collective stopping, divergence detection and a coupled wing
run with one output owner.

On ParallelCluster, configure MPI.jl to the system MPI that matches the launcher
and EFA installation, using `MPIPreferences.use_system_binary`. Reinstantiate and
restart Julia after changing preferences. Verify the chosen library and network
transport before performance measurements. Begin with one rank per node and
explicit CPU binding, then compare rank/thread layouts across sockets. Do not
assume the bundled local MPICH launcher uses the cluster's EFA transport.

Use fresh processes with the same thread count and fixed accuracy settings:

```bash
# Frozen-particle FMM timings, with compilation excluded:
JULIA_NUM_THREADS=1 julia --project=environments/mpi scripts/benchmark_mpi.jl serial 10000 5
JULIA_NUM_THREADS=1 julia --project=environments/mpi scripts/launch_mpi.jl 2 scripts/benchmark_mpi.jl mpi 10000 5

# Compare old whole-field queries with separate surface targets:
JULIA_NUM_THREADS=4 julia --project=environments/mpi scripts/benchmark_probes.jl 5000 128 5

# A complete coupled wing simulation:
JULIA_NUM_THREADS=2 julia --project=environments/mpi examples/mpi/wing.jl serial 32
JULIA_NUM_THREADS=2 julia --project=environments/mpi scripts/launch_mpi.jl 2 examples/mpi/wing.jl mpi 32
```

The synthetic particle benchmark measures the interaction kernel; it is not a
validated aerodynamic wake. The wing example includes coupling, shedding and
state synchronization. Both warm up before timing and report the slowest rank.
Equal-resource comparisons also need a threaded serial run with the same total
core allocation. Local multi-process timings are not multi-node performance
results. Cluster runs remain deferred until cluster access is supplied.

See [local validation results](mpi-validation.md) for checks performed and
known failures in the pre-existing experimental validation suite.
