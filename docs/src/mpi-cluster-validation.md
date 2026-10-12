# ParallelCluster validation log

Objective: reduce elapsed time for one coupled simulation. Compare a matched
single-node run with two and three compute nodes. Numerical agreement is required
within declared tolerances; bitwise equality is not required.

Case source: [DJI 9443 rotor in hover](https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-aero/).
The pilot keeps the tutorial's mid-low-fidelity model (Euler, inviscid, no SFS,
near-zero advance ratio, published hub/tip correction and initial hub-wake
suppression), with fewer blade elements and particle sheds. This checks the
implementation; it does not establish aerodynamic accuracy or converged hover.

## 2026-10-13: preparation

- Connected to the supplied head node. Its host key was previously unknown;
  SSH `StrictHostKeyChecking=accept-new` enrolled it without overriding a changed
  key. Noninteractive SSH does not load Slurm/modules into PATH, so subsequent
  commands use `/opt/slurm/bin` and source `/etc/profile.d/modules.sh`.
- Queue `compute` contains three dynamic `c6a.4xlarge` nodes, each advertising
  16 Slurm CPUs and 31,129 MB memory. Initially all were powered down and no jobs
  were queued. `/home` is exported by the head node to compute nodes.
- Isolated task directory: `/home/ubuntu/flowunsteady-mpi-20261013`.
  Existing source changes are copied into `code`; the application has a private
  Julia depot/runtime and Python environment. No existing cluster workspace is
  overwritten.
- Available MPI modules: Open MPI 4.1.7, Open MPI 5.0.9, Intel MPI 2021.18.
  Initial choice: Open MPI 4.1.7, configured explicitly through MPIPreferences.
  Installation/precompilation and simulation jobs run through `sbatch`.
- Found and fixed a callback synchronization gap: the tutorial mutates the
  externally captured `omit_shedding` vector. The MPI driver now broadcasts this
  known solver control after the root callback, preserving its local identity.
  Without this fix workers would shed a different wake on the next step.

Acceptance thresholds, fixed before examining cluster results:

- Particle counts, clocks and discrete/static flags agree exactly; all numeric
  state is finite.
- Each physical state group and load/circulation history must satisfy both a
  normalized L2 error <= `1e-5` and a scaled maximum absolute error <= `5e-5`.
  The comparison script records the units, scales and zero-reference floors.
- Audit all 46 active particle rows, with separate position, strength, core,
  velocity, gradient and auxiliary groups. Do not combine fields with different
  units into one error norm.
- Use the same case, integration, precision and fixed FMM settings in all runs.
  If a comparison fails, investigate the implementation or tighten the common
  FMM accuracy settings and repeat every baseline; do not loosen acceptance.

Timing excludes compilation warmup and artifact export, and includes coupling,
integration, shedding and runtime callbacks. Report the slowest rank's elapsed
time and repeat measured runs. A threaded ordinary serial run is the primary
single-node baseline; a one-rank MPI run measures adapter overhead.

Further job IDs, observed outcomes and numerical/performance results are appended
below as testing proceeds.

- Setup job `2` started a compute node but exited before installation: the batch
  shell's initial module search path did not include `/opt/amazon/modules/modulefiles`.
  Both Slurm scripts now explicitly add that directory. This was an environment
  initialization issue, not a solver failure.
- Local two-rank/two-thread regressions passed after the shedding-mask fix
  (106/104 assertions); the intentional root-output failure also terminated
  without leaving workers blocked.
- Setup job `3` confirmed AMD EPYC 7R13, eight physical cores and two hardware
  threads per core (16 vCPUs). Its Python setup stopped because the node lacks
  `ensurepip`. The task now uses an isolated `uv` executable to create/install the
  environment, avoiding system package changes. MPI runs use eight Julia threads
  per rank, bound to the eight physical cores.
- Slurm accounting storage is disabled on this cluster; job state/logs and
  application-recorded rank timings/RSS provide the run evidence instead of
  `sacct` history.
- Local rotor pilot (two ranks, two threads) exposed a Python/PyCall garbage
  collection crash during a longer warmup, in `pydecref` / NumPy deallocation.
  This matches the failure class in [PyCall issue 1072](https://github.com/JuliaPy/PyCall.jl/issues/1072).
  The upstream GIL/finalizer fix is still an unmerged pull request. For this
  bounded pilot, an explicit `--gc-policy deferred` option postpones Julia GC
  during each simulation and collects afterward on the initiating thread; the
  cleanup cost remains included in elapsed time. Use the same policy in every
  baseline and record peak RSS. This is an integration workaround, not a physics
  change or a demonstrated policy for long production runs.
- All source work is on branch `feat/mpi-single-simulation`.
- Setup job `4` installed the isolated Python environment and selected system
  Open MPI, then stopped because copied Conda build metadata retained the local
  workstation path. Setup now rebuilds Conda and PyCall in the cluster depot
  before precompilation; downloaded package sources/artifacts remain pinned.
- Setup job `5` exposed an MPI ABI collision: HDF5_jll imports OpenMPI_jll 5,
  whose Fortran products attempted to load alongside system Open MPI 4.1.7
  (`undefined symbol: ompi_instance_count`). It was cancelled after this failure
  was identified. Setup now selects installed Open MPI 5.0.9 and explicitly
  overrides all four OpenMPI_jll library products plus its launcher to that same
  installation. `OPAL_PREFIX` also points at the system installation, preventing
  bundled component discovery. MPI preferences remain local and uncommitted.
- Setup job `6` revealed that importing MPIPreferences with the previous system
  selection had already loaded MPI 4 into the configuration process. Attempting
  to discover MPI 5 in that process crashed during `dlclose`. Setup now clears
  the task-local MPI selection using TOML before importing MPIPreferences in a
  fresh process; it never loads two MPI installations together.
- Full local deferred-GC pilot completed at 144 steps / 6,658 particles on
  eight threads per rank. The serial versus two-rank comparison passed every
  declared state/load/history threshold; peak process RSS stayed below 5 GB.
- Setup job `7` completed successfully: Julia 1.13.1, Python 3.12.3, system
  Open MPI 5.0.9amzn1 and all pinned FLOW dependencies import together.
  The local pilot's recorded numerical differences were zero in every field and
  history; cluster equality remains to be established by the distributed runs.
- Validation job `8` placed one rank on each of the three distinct compute
  hosts, bound to cores 0-7. Its smoke test then stopped on a test-harness bug:
  `all` was given per-rank bit vectors instead of scalar predicates. The nested
  assertion is corrected. No aerodynamic comparison ran in this job.
- Validation job `9` passed the three-node smoke test: distinct hosts, binding
  to all eight physical cores, 800,000-byte-per-rank Allgather, matching MPI
  versions, and all four loaded MPI libraries from `/opt/amazon/openmpi5`.
  The inter-node transport is explicitly Open MPI `ob1` / TCP on the private
  subnet; these nodes expose no EFA device.
- The comparison now also requires identical hashes of all dependency identities,
  the application manifest and the complete rotor database, plus matching
  Python/NumPy/SciPy versions. An injected dependency-version mismatch was
  rejected in the local smoke test.
