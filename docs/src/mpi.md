# MPI execution of one simulation

`ReplicatedMPI` divides interaction targets, sources, or both between MPI ranks while
retaining the complete wake on each rank. The implementation uses `UVLMVehicle` and
supports the existing inviscid rVPM integrators,
relaxation, wake shedding, and wing/rotor coupling. The default target partition
also supports constant/dynamic SFS; source/hybrid partitions require `SFS_none`. FMM uses
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
close(backend) # collective cleanup; harmless for target/source partitions
MPI.Finalize()
```

Normal serial usage continues to use `parallel=nothing` and does not initialize
MPI. Wing/rotor velocity queries use separate targets on the pinned backend;
they no longer add probes or recompute wake-target derivatives. Custom interaction
functions and older serial installations retain their original query path.

`partition=:targets` is the default: each rank evaluates disjoint targets against
all sources, then gathers velocity and gradient outputs. With
`partition=:sources`, each rank evaluates its disjoint source range against all
targets, then sums contributions with `Allreduce`. The same choice applies to
surface velocity queries. Sources are divided for computation; the full particle
field, integration and wake shedding remain replicated. Source partitioning
requires `vpm_SFS=uns.vpm.SFS_none` and rejects SFS evaluation.

`source_order=:cyclic` optionally assigns source indices `s+1:S:N` to source
group `s`, spreading chronological wake ages across groups. Here `S` is the rank
count for source partitioning and the source-group count for hybrid partitioning.
The default `:contiguous` retains contiguous source blocks. This choice applies
to wake and surface evaluations and leaves target partitioning unchanged. It can
change summation order and FMM trees; validate the complete coupled case with the
same accuracy settings before using it for timings.

For `partition=:hybrid`, specify `target_groups=T` where `T` divides the rank count
`R`. The source-group count is `S=R÷T`; rank `s*T+t` owns source group `s` and target
block `t`. Each rank evaluates about `N/S` sources on `N/T` targets, sums partial
results across its source column, then gathers the target blocks across its target
row. Every rank receives the complete velocity/gradient arrays. Surface queries
use the same grid. `T=1` is source partitioning and `T=R` is target partitioning;
hybrid mode currently rejects SFS for all grid shapes.

With host-contiguous ranks, `T=ranks_per_node` puts the target-row gather within
each node and source-column reductions across nodes. The backend does not infer
hostname topology or choose factors automatically. For six ranks, for example,
`ReplicatedMPI(; partition=:hybrid, target_groups=2)` uses a three-by-two grid.

Hybrid child communicators are created lazily. Call `close(backend)` on every rank
outside runtime callbacks and before `MPI.Finalize()` to release them. Close is
idempotent; the caller's main communicator remains valid, and a later collective
interaction recreates the child communicators. Close before changing a grid's
factorization. A callback's attempted close is rejected before it can block its
workers. Partition, target-group and source-order agreement are checked before evaluation,
even with `check_state=false`.

All MPI partitions reuse the relative regularization cutoff, which depends only
on the fixed tolerance, once per source adapter. An absolute-cutoff solve is
skipped only when a residual bound, with a roundoff margin, proves that the
existing minimum is the relative cutoff. Boundary cases retain the original
Brent solve. Particle strengths, core sizes, kernels and accuracy settings are
unchanged; source-buffer equivalence is covered by regression checks.

`nearfield=:dynamic` optionally distributes disjoint target-body ranges across
local Julia threads. Each range retains its complete source sequence, preserving
each body's accumulation order. This permits several workers to process one
target leaf, including small surface queries. The default `nearfield=:static`
retains the pinned scheduler; unsupported interaction-list methods also use it.
Measure both schedulers for the intended rank/thread layout.

`rotor_parallel=true` optionally distributes independent blade-element polar
preparation for the standard non-debug, wake-coupled lookup rotor solve. It
requires a supplied `sound_spd`; one-rank execution, debug mode, missing sound
speed, unsupported keyword options and other solver paths retain the original
solver. Mach, 3D correction, extrapolation, force and circulation calculations
use the unchanged pinned FLOWVLM functions. MPI transfers spline coefficients
and scalar metadata; each rank reconstructs local spline workspaces. Python and
native object handles remain local. This option defaults to `false`.

The root rank alone writes files and executes `extra_runtime_function`. Particle
additions, deletions and edits, coupled simulation state, scheme state and the
callback's stop decision are propagated to the other ranks.
The driver's `omit_shedding` vector is also synchronized after callbacks, so the
tutorial's initial hub-wake suppression and subsequent release apply on all ranks.
Functions and native airfoil/Python objects stay local to each rank. Callbacks must keep those models,
freestream functions and vehicle topology fixed. During that callback,
additional interaction queries run serially on the root. Callbacks must return a
`Bool`; they must not invoke their own MPI collectives. Worker state is restored
into its existing object graph so vehicle aliases remain intact. The active
particle field is replicated; callbacks should not
modify unused capacity slots or replace its interaction evaluator.

For a callback that only reads simulation, particle and scheme state, opt in with
`ReplicatedMPI(callback_sync=:readonly)`. The root still executes the callback
serially, and its stop flag and the driver's `omit_shedding` vector synchronize.
Each rank retains the state it independently integrated, solved and shed; the
full simulation and new-wake broadcast is skipped. This contract requires
identical deterministic solver functions on every rank. External monitor/history
data may change on the root, as may the explicitly synchronized shedding mask.
Callbacks that modify solver state must use the default `callback_sync=:full`.

With both `callback_sync=:readonly` and `check_state=true`, root snapshots check
serializable simulation/scheme state and active particles before and after the
callback, and all ranks check particle replicas afterward. These diagnostics
cannot detect changes hidden in native/Python handles or captured closures.
Validate the complete coupled case before disabling checks for timing; read-only
mode with checks enabled intentionally retains the snapshot/scan overhead.

Set `check_state=true` while validating a case. It checks active particle values
before each collective evaluation; disable it for production timings to avoid
the extra full-field scan. All ranks must use identical target positions and
solver settings. An unexpected error during a multi-rank simulation aborts that
MPI job, preventing peers from remaining blocked in a collective.

## Numerical choices and limitations

- Velocity and all nine velocity-gradient components are gathered or summed at
  every interaction evaluation. With target partitioning, SFS evaluation follows
  that exchange. RK3 and dynamic SFS use the same collective evaluator at their
  internal stages; synchronization is not deferred to the end of the timestep.
- FMM SFS uses the original full-field target tree's near-field neighborhoods.
  Partitioning targets must not change those SFS neighborhoods.
- Distributed FMM settings are fixed. Select expansion order, leaf size and
  tolerances from the serial baseline; distributed ranks do not independently
  autotune them. Different source/target trees and reductions can change summation order and FMM
  approximation error, so validate loads and wake observables rather than require
  bitwise equality.
  The tolerance applies within each source tree; summing source columns does not
  establish a global error bound. Validate the complete coupled case against its
  serial reference.
- Core spreading/RBF redistribution, GPU execution, experimental mirrored
  ground effect, `sigmafactor_vpmonvlm != 1`, and arbitrary custom UJ functions are
  not supported by this MPI path. Unsupported options raise an error. The existing
  wake-file warm start remains a warm start; this adds no complete checkpoint.
- Quasi-steady vehicles are outside this first MPI implementation. The original
  quasi-steady driver currently rejects rotor-correction keywords passed by
  `run_simulation`; this is also present before the MPI changes.
- Target partitioning replicates source-tree work; source partitioning replicates
  target-tree work. Particle integration and most vehicle-solve work remain on
  every rank. Root callbacks and output remain serial. More ranks are not
  guaranteed to improve elapsed time; measure the intended case and node layout.

## Local verification and cluster launch

The [ParallelCluster log](mpi-cluster-validation.md) records cluster setup,
matched hover cases, fixed numerical thresholds, failures and results. The
headless rotor case exports all active particle fields and every-step
thrust/torque/blade histories:

```bash
JULIA_NUM_THREADS=8 julia --project=environments/mpi examples/mpi/rotorhover.jl \
  --mode serial --n 8 --revolutions 4 --steps-per-rev 36 --p-per-step 2 \
  --gc-policy deferred --output /tmp/hover-serial
JULIA_NUM_THREADS=8 julia --project=environments/mpi scripts/launch_mpi.jl 2 \
  examples/mpi/rotorhover.jl --mode mpi --n 8 --revolutions 4 \
  --steps-per-rev 36 --p-per-step 2 --gc-policy deferred \
  --check-state true --output /tmp/hover-mpi
python3 scripts/compare_mpi_rotorhover.py /tmp/hover-serial /tmp/hover-mpi \
  --output /tmp/hover-comparison.json
```

`deferred` postpones Julia garbage collection during each bounded simulation to
avoid a PyCall finalizer/threading defect; collection afterward is included in
elapsed time. The example defaults to automatic GC. Deferred collection increases
memory use and is not a production solution for unrestricted long runs. Both
baselines must use the same policy. See the cluster log for the observed failure
and memory measurements.

Run the regression suite with the MPI launcher matching MPI.jl:

```bash
JULIA_NUM_THREADS=1 julia --startup-file=no --project=environments/mpi scripts/test_mpi.jl
# Also exercise threaded FMM within each rank:
JULIA_NUM_THREADS=2 julia --startup-file=no --project=environments/mpi scripts/test_mpi.jl 2
```

Tests cover direct/FMM velocity and gradients, uneven and empty partitions,
source-cutoff equivalence, nearfield scheduling, static sources, RK3,
constant/dynamic SFS with target partitioning, surface queries, root callbacks,
read-only mutation rejection, distributed rotor polars, collective stopping,
divergence detection and coupled simulations with one output owner.

On ParallelCluster, configure MPI.jl to the system MPI that matches the launcher
and EFA installation, using `MPIPreferences.use_system_binary`. Reinstantiate and
restart Julia after changing preferences. Verify the chosen library and network
transport before performance measurements. Begin with one rank per node and
explicit CPU binding, then compare rank/thread layouts across sockets. Do not
assume the bundled local MPICH launcher uses the cluster's EFA transport.

Reserve enough Slurm task slots for the largest rank count per node being tested.
Open MPI's process slots and CPU binding are separate controls; resource-manager
slot limits still cap explicit host slots. Use `--map-by ppr:R:node:PE=T` and
`--bind-to core` to assign `R` ranks per node and `T` physical cores per rank,
then inspect `--report-bindings` and each process's allowed CPU list. See the
[Open MPI launcher documentation](https://docs.open-mpi.org/en/v5.0.6/man-openmpi/man1/mpirun.1.html).
`scripts/cluster/layout_smoke.sbatch` checks these layouts before solver timing.

Keep the physical cores per node and numerical accuracy settings fixed while
comparing rank/thread layouts. Use fresh processes for final measurements:

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
validated aerodynamic wake. The wing and rotor cases also include coupling,
integration, shedding, callbacks and cleanup. Warm up the complete measured case,
exclude artifact export, and report the slowest rank over repeated runs. Report
kernel scaling and whole-simulation scaling separately.

For node scaling, compare with the fastest validated one-node layout using the
same per-node core budget, case, precision, tolerances and GC policy. Include both
threaded serial and optimized one-node MPI layouts; a computational improvement
already available on one node is not an additional node-scaling gain. Keep the
original serial comparison as a separate reference. Local multi-process timings
do not establish multi-node performance.

`scripts/benchmark_scaling.jl` and `scripts/cluster/scaling_batch.sbatch` reuse
processes to screen layouts, with a complete warmup and fresh simulation per
layout. Their process history and lifetime peak RSS include earlier layouts.
Confirm selected layouts with repeated fresh-process runs from
`scripts/cluster/scaling.sbatch` before reporting final scaling. The
[ParallelCluster log](mpi-cluster-validation.md) contains the cluster evidence.

The summary script keeps fresh, reused and unknown process histories in separate
timing groups and requires matching provenance and harnesses for scaling
baselines. Older ordinary hover exports omit the reuse marker. For inputs whose
launch logs establish fresh processes, pass `--process-provenance fresh`; the
script rejects this declaration if any input explicitly records process reuse.

See [local validation results](mpi-validation.md) for checks performed and
known failures in the pre-existing experimental validation suite.

## Optional thread placement for Linux benchmarks

Pass `--pin-threads true` to the hover, scaling or profiling harness to pin each
Julia default-pool worker to one allowed logical CPU on a distinct physical
core, before warmup. Keep MPI rank binding enabled: the helper selects cores
within each worker's existing mask and does not coordinate ownership across
ranks. Interactive, GC, BLAS, Python and MPI helper threads are left untouched.
The default is `false`.

Placement failures are agreed collectively before warmup; a local pinning
failure restores the workers' original masks. Inspect every rank's
`worker_thread_bindings` in `metadata.toml` under `rank_profiles` to verify
worker IDs, native thread IDs, original and selected CPU masks, and physical
package/core IDs. Compare pinned runs against matched pinned one-node baselines
using the same physical-core budget and numerical settings.
