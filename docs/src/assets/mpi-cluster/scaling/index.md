# Cluster scaling evidence

Completed batch jobs are screening runs with reused MPI processes. Fresh verification uses a separate process launch for each layout. Each layout performs a complete warmup and constructs a fresh measured case. Compilation, warmup and artifact export are outside the reported runtime; deferred garbage collection at the end of the measured case is inside it. Screening minima and fresh verification stay in separate summaries and timing cohorts.

| Frozen version | Completed Slurm jobs | Fixed blade resolution |
| --- | --- | --- |
| v3 | 20, 21, 22 | n8 |
| v4 | 23, 24, 25; 26 | n8; n16 |
| v5 | 28; 29 | n8; n16 |
| v6 | 32, 33, 34; 36; fresh37, 38, 39 | n8; n16; n32 |

Each version directory contains the grouped summary CSV, individual run CSV, actual scalar `case-metadata.csv`, per-rank phase/RSS ledgers and untouched original comparison JSON files. The metadata and comparison reports retain their original remote artifact paths. v6 has separate `v6-fresh-summary` CSVs for completed fresh verification. Full particle histories and sampling snapshots are not included here.

All archived MPI candidates pass the same 35 numerical checks and nine software/data identity checks. The normalized L2 tolerance is `1e-5`, maximum tolerance is `5e-5`, and scale floor is `1e-12`; the FMM remains p12, ncrit64, relative tolerance `1e-7`, absolute tolerance `1e-10`. These checks cover the complete 144-step pilot and its recorded loading history within each fixed n8, n16 or n32 experiment.

Summaries segregate case and software identities, process provenance, benchmark harness hash, replica-check settings and layouts. The CSV `speedup` field is the serial-reference ratio; `strong_speedup` uses the fastest validated matching one-node implementation using all eight physical cores, including alternate MPI layouts. Both ratios require a matching process cohort and harness. Serial runs using two or four threads are consistency references; their slower times do not establish eight-core strong scaling. Compare whole-simulation speedup separately from evaluation-kernel speedup.

The phase ledger records each rank's own accumulated timers. Its remainder is elapsed time minus that rank's counters, an illustrative diagnostic containing untimed work and synchronization. Per-rank maxima and different phase maxima do not form an additive critical path.

The node RSS ledger sums rank process-lifetime peaks on each host, including imports, compilation and warmup. This is not a simultaneous measurement of total node RSS.

v6 also includes a worker-binding CSV extracted and checked against every rank's original TOML metadata. Every active Julia default-pool worker has a singleton CPU affinity, its recorded current CPU matches that affinity, and its physical package/core identifier is distinct from all other active workers on the same host. MPI processes use disjoint sets of physical cores and collectively cover eight physical cores per node. Earlier versions recorded process masks, which alone do not prove actual worker placement. The worker records are the post-pinning observation before warmup. Full original TOML metadata remains in the downloaded raw result sets; this compact archive contains its worker records, scalar case metadata and original comparison reports.

No runtime minimum from different frozen software identities or physical resolutions is pooled into one timing group. New completed evidence is added without changing the original comparison reports.

Known-fresh n32 reports lack an explicit process-reuse marker in the frozen exporter. Their fresh summary therefore uses the explicit `--process-provenance fresh` override, justified by separate process launches in the archived frozen wrappers. Wrapper bodies and SHA256 provenance are under `v6/wrappers/`. Missing provenance otherwise defaults to `unknown`; reused metadata cannot be overridden as fresh. Reused reference/candidate artifacts are deduplicated by their real local path rather than counted again as timing samples.
