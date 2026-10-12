# Local MPI implementation validation

13 October 2026. Linux x86-64, Ryzen 9 9950X3D2, 16 physical cores, Julia 1.13.1,
Python 3.12, MPI.jl 0.20.27 with its bundled MPICH. Dependencies are locked in
`environments/mpi/Manifest.toml`. No AWS cluster jobs were run.

## Correctness checks

The MPI regression suite passed with 1, 2 and 4 ranks, and with two Julia threads
per rank. It covers velocity and all gradient components, uneven/empty target
partitions, static sources, reset/accumulation behavior, RK3, relaxation,
constant/dynamic SFS, preservation of the full-field FMM SFS neighborhoods,
target-only queries, callback mutation/stopping, replica checks, output ownership,
and coupled wing and rotor simulations. Local native airfoil objects and vehicle
aliases are preserved during callback synchronization.

An intentional root-only output failure terminated the two-rank MPI job rather
than leaving workers blocked. Unsupported options fail explicitly. Git whitespace
checks passed. The GitHub workflow has been added but has not run remotely.

## Pilot timings

These are local synthetic measurements, not multi-node or aircraft speedups.
The elongated frozen field contains 5,000 Float64 particles. Each timing is the
minimum of three warmed samples, with fixed FMM settings and no SFS or output.

| Interaction evaluation | Time |
|---|---:|
| Serial process, 1 Julia thread | 0.152114 s |
| Serial process, 4 Julia threads | 0.042739 s |
| 4 MPI processes, 1 thread each, same machine | 0.057903 s |

MPI was faster than the one-thread baseline, but about 35% slower than four Julia
threads at the same total core count. This is consistent with replicated source
work and communication overhead. It does not predict the crossover on multiple
nodes or on larger cases.

For 128 probe positions on the same 5,000-particle field, using four threads:

| Query path | Time |
|---|---:|
| Original whole-field evaluation | 0.045085 s |
| Separate velocity-only targets | 0.011174 s |

The query was about 4.0 times faster. The maximum velocity-component difference
between the two FMM approximations was `1.223196e-6`. This is a query-level result,
not a fourfold improvement in complete simulation time or an engineering
acceptance tolerance. Direct-reference and coupled-equivalence checks are separate.

## Existing serial validation

`Pkg.test("FLOWUnsteady"; allow_reresolve=false)` completed with three passing
cases, one failure and one error. The coupled wing, kinematic wing and regularized
VLM cases passed their existing lift/drag limits.

The two remaining issues were reproduced using the original source at commit
`66d986673b5fce034d594127fa3b41c4dbd58e9b` with the same pinned dependencies:

- The uncoupled wing returns CL `0.23296` and CD `0.00436`. Its drag error against
  the experimental CD `0.005` is 12.75%, above the existing 11% limit.
- The quasi-steady solve rejects `apply_correction3D` and `apply_extrapolate`
  keywords passed by the existing driver.

No experimental values, acceptance limits or aerodynamic models were changed to
make those checks pass. MPI is currently restricted to `UVLMVehicle`. The new CI
workflow runs the MPI/serial-equivalence checks; these pre-existing validation
issues remain for separate engineering review.
