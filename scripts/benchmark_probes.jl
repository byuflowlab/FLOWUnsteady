#!/usr/bin/env julia
# Measure the target-only query optimization independently of MPI.
using Random, Printf, LinearAlgebra
import FLOWUnsteady as uns
const vpm = uns.vpm
BLAS.set_num_threads(1)
n = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 5000
probes = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 128
samples = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 5
all(>(0), (n, probes, samples)) || error("Counts must be positive")
legacy_uj(p) = vpm.UJ_fmm(p)
fmm = vpm.FMM(; p=8, ncrit=64, min_ncrit=64, relative_tolerance=1e-3,
    absolute_tolerance=1e-6, autotune_p=false, autotune_ncrit=false)
legacy = vpm.ParticleField(n + probes, Float64; UJ=legacy_uj, fmm)
targeted = vpm.ParticleField(n + probes, Float64; UJ=vpm.UJ_fmm, fmm)
rng = MersenneTwister(20261012)
for i in 1:n
    X = [20.0 * i/n, 0.2 * randn(rng), 0.2 * randn(rng)]
    G = 0.001 .* randn(rng, 3)
    vpm.add_particle(legacy, X, G, 0.03)
    vpm.add_particle(targeted, X, G, 0.03)
end
Xs = [[20.0 * i/probes, 0.2 * randn(rng), 0.2 * randn(rng)] for i in 1:probes]
old_values, new_values = uns.Vvpm_on_Xs(legacy, Xs), uns.Vvpm_on_Xs(targeted, Xs)
error = maximum(abs(old_values[i][j] - new_values[i][j]) for i in 1:probes, j in 1:3)
println("mode,threads,particles,probes,samples,min_seconds,mean_seconds,max_velocity_difference")
for (name, pf) in (("whole_field", legacy), ("targets_only", targeted))
    uns.Vvpm_on_Xs(pf, Xs) # warmup
    times = Float64[]
    for _ in 1:samples
        GC.gc()
        push!(times, @elapsed uns.Vvpm_on_Xs(pf, Xs))
    end
    @printf("%s,%d,%d,%d,%d,%.6f,%.6f,%.6e\n", name, Threads.nthreads(), n,
        probes, samples, minimum(times), sum(times)/samples, error)
end
