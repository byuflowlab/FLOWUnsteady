# ParallelCluster validation log

The one-, two- and three-node implementations produced matching numerical results
in the DJI hover pilot, with zero recorded differences across every checked field
and history. Three repetitions per layout gave a 1.346x speedup on three nodes
versus the threaded serial baseline (20.563 s versus 15.282 s). All work is
committed on `feat/mpi-single-simulation`; the complete cluster jobs have finished.

Objective: reduce elapsed time for one coupled simulation. Compare a matched
single-node run with two and three compute nodes. Numerical agreement is required
within declared tolerances; bitwise equality is not required.

Case source: [DJI 9443 rotor in hover](https://flow.byu.edu/FLOWUnsteady/examples/rotorhover-aero/).
The pilot keeps the tutorial's mid-low-fidelity model (Euler, inviscid, no SFS,
near-zero advance ratio, published hub/tip correction and initial hub-wake
suppression), with fewer blade elements and particle sheds. This checks the
implementation; it does not establish aerodynamic accuracy or converged hover.

## Case and final measurements

The two-bladed DJI 9443 case uses eight elements per blade, 36 steps/revolution,
two particle sheds/step and four revolutions at 5,400 RPM. Advance ratio is
`1e-4`; density is 1.071778 kg/m^3. The simulation completes 144 steps with 6,658
particles. It uses Float64, rVPM/Euler, inviscid diffusion, no SFS, the tutorial's
hub/tip correction and initial hub-wake suppression released at revolution three.
FMM settings are fixed: `p=12`, `ncrit=min_ncrit=64`, relative tolerance `1e-7`,
absolute tolerance `1e-10`, and no order/leaf-size autotuning.

Job `10` completed all twelve measurements in 21m52s. Each fresh process ran a
complete same-case warmup before the timed simulation; `check_state=false` for
timings. Every candidate, including repeated serial runs, passed nine input
identity checks and 35 numerical checks against the first serial reference.
All recorded normalized L2/max errors were zero. No acceptance tolerance was
loosened and no aerodynamic model was changed to obtain agreement.

| Active nodes | Mode | Ranks | Threads/rank | Runs | Median s | Min-max s | Speedup |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Serial | 1 | 8 | 3 | 20.563 | 20.447-20.755 | 1.000 |
| 1 | MPI | 1 | 8 | 3 | 21.511 | 21.497-21.702 | 0.956 |
| 2 | MPI | 2 | 8 | 3 | 16.605 | 16.562-16.730 | 1.238 |
| 3 | MPI | 3 | 8 | 3 | 15.282 | 15.241-15.389 | 1.346 |

Each active node uses its eight physical cores; Slurm advertises 16 vCPUs because
SMT is enabled. The sequential comparison allocation reserves all three nodes,
but each layout launches only on the listed number of hosts. These are active
compute counts, not an AWS billing estimate. Three nodes reduce measured elapsed
time by 25.7%, with 44.9% efficiency relative to ideal three-node scaling.
Peak process RSS across the repeated MPI runs was 4.69 GiB, including imports,
compilation and warmup.

The bounded runs require the explicit deferred-GC workaround described below.
Cleanup collection is included in elapsed time; artifact export is excluded.
Longer/higher-fidelity production runs need a proper PyCall threading/finalizer
solution before this policy can be generalized.

Evidence: [complete repeated comparisons](assets/mpi-cluster/benchmark-comparison.json),
[median summary](assets/mpi-cluster/benchmark-summary.csv),
[individual measurements](assets/mpi-cluster/benchmark-summary-runs.csv).

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
- Job `9`: distributed regression suite passed on all three nodes (106 checks
  on the output-owning rank, 104 on each worker), including coupled wing/rotor,
  RK3/SFS, callback synchronization and output. The reference hover run completed
  144 steps with 6,658 active particles; warmup 47.045 s, timed run 20.686 s.
- Job `9` completed successfully. Every MPI layout passed all nine input identity
  checks and all 35 numerical checks. Recorded state, circulation and load
  differences were zero; clocks, counts and shedding-mask histories agreed
  exactly. The case exercised suppression through revolution three and subsequent
  release, completing four revolutions.

| Active nodes | Mode | Ranks | Threads/rank | Timed seconds | Speedup vs serial |
| --- | --- | --- | --- | --- | --- |
| 1 | Serial | 1 | 8 | 20.686 | 1.000 |
| 1 | MPI | 1 | 8 | 21.654 | 0.955 |
| 2 | MPI | 2 | 8 | 16.803 | 1.231 |
| 3 | MPI | 3 | 8 | 15.417 | 1.342 |

These initial validation timings use `check_state=true`. Peak process RSS was
3.78 GiB in serial and at most 4.77 GiB in MPI, including imports/compilation and
warmup. One MPI rank adds overhead; distributing interaction targets across
two/three nodes reduces elapsed time for this case.

Recorded evidence: [all comparison checks](assets/mpi-cluster/validation-comparison.json),
[group summary](assets/mpi-cluster/validation-summary.csv) and
[individual runs](assets/mpi-cluster/validation-summary-runs.csv).
The complete particle/blade/history artifacts and job logs are retained outside
Git in `/tmp/flowunsteady-cluster-20261013` locally and the isolated cluster task
directory. The committed reports contain input fingerprints and measured results;
they contain no AWS credentials or MPI machine preferences.

- Job `10` repeats each of the four layouts three times, with `check_state=false`
  and the same accuracy, inputs and GC policy. Each process performs a complete
  same-case warmup, then constructs a fresh simulation for measurement. The
  distributed order alternates between repetitions. Every measured result is
  compared with the first serial reference using the unchanged thresholds.

## Interpretation of the first measurements

The backend accelerates one simulation by distributing targets. It retains the
whole wake, source-tree construction/upward passes, particle integration and
rotor solves on each rank. It does not distribute particle storage or increase
the particle capacity of a node.

In job `9`, accumulated maximum-rank interaction evaluation fell from 15.778 s
on one MPI rank to 10.083 s on two and 8.195 s on three. Surface-query time changed
little (2.144, 2.044, 2.029 s), and callback/synchronization time was 0.889, 1.246,
1.236 s. Overall evaluation totals across ranks differed by only a few percent
in the first benchmark repetition; severe overall imbalance is not the leading
observed limit for this pilot.

Target-result Allgatherv took 0.021, 0.090, 0.160 s, respectively. This timer does
not include control collectives, and callback timing includes broadcast/waiting.
Independent phase maxima can come from different ranks and must not be summed
into a critical-path breakdown. Coupling, integration, allocation and deferred
GC cleanup have not been profiled separately.

Before increasing case size/duration, address PyCall finalizer safety. Then
profile FMM tree construction, upward/translation/near-field passes and allocation
costs, and investigate buffer/cache reuse before choosing a distributed-source
implementation. These measurements establish numerical equivalence and modest
speedup for this four-revolution configuration, not larger-case scaling or
aerodynamic convergence.

## Repeating the cluster checks

The scripts expect the prepared shared task directory with `code`, `runtime`
(Julia 1.13.1 and `uv`) and a private `depot`. Setup was validated using the
preseeded pinned package downloads/artifacts; it rebuilds machine-specific Python
metadata and configures all MPI libraries consistently. On the head node:

```bash
task_root=/home/ubuntu/flowunsteady-mpi-20261013
/opt/slurm/bin/sbatch --output="$task_root/logs/setup-%j.log" \
  "$task_root/code/scripts/cluster/setup.sbatch" "$task_root"
# Wait for successful setup before launching validation.
/opt/slurm/bin/sbatch --output="$task_root/logs/validate-%j.log" \
  "$task_root/code/scripts/cluster/validate.sbatch" "$task_root"
# After validation passes, repeat each layout three times.
/opt/slurm/bin/sbatch --output="$task_root/logs/benchmark-%j.log" \
  "$task_root/code/scripts/cluster/benchmark.sbatch" "$task_root" 3
```

The application writes full particle, blade and load histories plus input hashes,
rank placement, timings and RSS. Compare/summarize downloaded results with:

```bash
python3 scripts/compare_mpi_rotorhover.py SERIAL_DIR MPI_DIR [MPI_DIR ...] \
  --output comparison.json
python3 scripts/summarize_mpi_rotorhover.py comparison.json \
  --results-root RESULTS_DIR --csv summary.csv --markdown summary.md
```

The pinned manifest, source/database hashes and Python versions must match.
MPI machine preferences and Python environments are ignored by Git. Credential
patterns were checked on each staged block and the complete branch diff; AWS
credentials, SSH keys and machine preferences were not committed. No branch was
pushed and no pull request was opened.
