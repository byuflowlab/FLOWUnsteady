using Test, Random
import FLOWUnsteady as uns
const vpm, MPI = uns.vpm, uns.MPI
MPI.Init()
const comm = MPI.COMM_WORLD
const rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)

function cyclic_field(uj; n=65, farfield=false, R=Float64)
    settings = vpm.FMM(; p=12, ncrit=farfield ? 16 : 256,
        min_ncrit=farfield ? 16 : 256, theta=0.4,
        relative_tolerance=1e-7, absolute_tolerance=1e-10,
        autotune_p=false, autotune_ncrit=false)
    pf = vpm.ParticleField(n + 32, R; UJ=uj, SFS=vpm.SFS_none, fmm=settings)
    rng = MersenneTwister(20261014)
    for i in 1:n
        center = farfield ? [i <= div(n, 2) ? -5.0 : 5.0, 0.0, 0.0] : zeros(3)
        position = center + (farfield ? 0.15 : 1.0) .* randn(rng, 3)
        vpm.add_particle(pf, R.(position), R.(0.001 .* randn(rng, 3)),
            R(farfield ? 0.002 : 0.1); static=i == n)
    end
    return pf
end

try
    @testset "Cyclic source ownership ($nranks ranks)" begin
        @test_throws ArgumentError uns.ReplicatedMPI(; source_order=:invalid)
        @test uns.ReplicatedMPI().source_order === :contiguous
        @test_throws ArgumentError uns._owned_sources(3, 0, nranks, :invalid)
        @testset "Every source has exactly one owner" begin
            for n in unique([0, 1, max(0, nranks-1), nranks+1, 3nranks+1]),
                    order in (:contiguous, :cyclic)
                blocks = [uns._owned_sources(n, r, nranks, order) for r in 0:nranks-1]
                @test sort(vcat(collect.(blocks)...)) == collect(1:n)
                @test sum(length, blocks) == n
                @test maximum(length.(blocks)) - minimum(length.(blocks)) <= 1
                order === :contiguous && @test blocks ==
                    [uns._owned_targets(n, r, nranks) for r in 0:nranks-1]
            end
        end
        @testset "Strided source buffers retain original data" begin
            for R in (Float32, Float64)
                pf = cyclic_field(vpm.UJ_direct; R)
                indices = uns._owned_sources(pf.np, rank, nranks, :cyclic)
                sources = uns._particle_sources(pf, indices)
                @test sources.indices === indices
                before = copy(pf.particles)
                expected, actual = zeros(R, 8, length(indices)), zeros(R, 8, length(indices))
                for i in eachindex(indices)
                    uns._particle_fmm.source_system_to_buffer!(expected, i, pf, indices[i])
                    uns._particle_fmm.source_system_to_buffer!(actual, i, sources, i)
                    @test uns._particle_fmm.get_position(sources, i) ==
                          uns._particle_fmm.get_position(pf, indices[i])
                end
                @test isequal(actual, expected)
                @test isequal(pf.particles, before)
            end
        end
        layouts = vcat([(:sources, 1)], [(:hybrid, t) for t in 1:nranks if nranks % t == 0])
        for (partition, groups) in layouts
            @testset "$partition target groups $groups" begin
                backend = uns.ReplicatedMPI(; partition, target_groups=groups,
                    source_order=:cyclic, nearfield=:dynamic, check_state=true)
                try
                    for serial in (vpm.UJ_direct, vpm.UJ_fmm), n in (0, 1, 3, 65)
                        uj = uns._parallel_interaction(backend, serial, vpm.Inviscid())
                        reference, distributed = cyclic_field(serial; n), cyclic_field(uj; n)
                        for reset in (true, false)
                            serial(reference; reset)
                            uj(distributed; reset)
                            @test distributed.particles ≈ reference.particles rtol=2e-6 atol=1e-7
                        end
                        before = copy(distributed.particles)
                        points = [[0.2, 0.4, -0.3], [-0.1, 0.3, 0.5], [0.9, 0.1, -0.5]]
                        got, expected = uns.Vvpm_on_Xs(distributed, points), uns.Vvpm_on_Xs(reference, points)
                        @test all(i -> isapprox(got[i], expected[i]; rtol=1e-9, atol=1e-12), eachindex(got))
                        @test isequal(distributed.particles, before)
                        @test isempty(uns.Vvpm_on_Xs(distributed, Vector{Float64}[]))
                        @test_throws ArgumentError uj(distributed; sfs=true)
                    end
                    uj = uns._parallel_interaction(backend, vpm.UJ_fmm, vpm.Inviscid())
                    # Keep enough sources in each separated cluster after
                    # striding that the pinned FMM accepts genuine translations.
                    farfield_n = max(256, 128nranks)
                    reference, direct, distributed = cyclic_field(vpm.UJ_fmm; n=farfield_n, farfield=true),
                        cyclic_field(vpm.UJ_direct; n=farfield_n, farfield=true),
                        cyclic_field(uj; n=farfield_n, farfield=true)
                    owned, sources = partition === :hybrid ?
                        uns._hybrid_domains(backend, distributed.np, distributed.np) :
                        (1:distributed.np, uns._owned_sources(distributed.np, rank, nranks, :cyclic))
                    before = copy(distributed.particles)
                    full_result = uns._evaluate_targets!(uns._interaction_targets(distributed, 1:distributed.np),
                        distributed, vpm.UJ_fmm)
                    @test !isempty(full_result[5])
                    result = uns._evaluate_targets!(uns._interaction_targets(distributed, owned),
                        distributed, vpm.UJ_fmm; source_indices=sources, nearfield=:dynamic)
                    @test MPI.Allreduce(!isnothing(result) && !isempty(result[5]), |, comm)
                    @test isequal(distributed.particles, before)
                    vpm.UJ_fmm(reference)
                    vpm.UJ_direct(direct)
                    uj(distributed)
                    for rows in (vpm.U_INDEX, vpm.J_INDEX)
                        @test distributed.particles[rows, :] ≈ reference.particles[rows, :] rtol=1e-5 atol=1e-10
                        @test distributed.particles[rows, :] ≈ direct.particles[rows, :] rtol=1e-5 atol=1e-10
                    end
                    before = copy(distributed.particles)
                    points = [[0.1i, 2.0, -1.0] for i in 1:17]
                    got, expected = uns.Vvpm_on_Xs(distributed, points), uns.Vvpm_on_Xs(reference, points)
                    @test all(i -> isapprox(got[i], expected[i]; rtol=1e-5, atol=1e-10), eachindex(got))
                    @test isequal(distributed.particles, before)
                finally
                    close(backend)
                end
            end
        end
        @testset "Target partition retains source sequence" begin
            plain = uns._parallel_interaction(uns.ReplicatedMPI(), vpm.UJ_fmm, vpm.Inviscid())
            cyclic = uns._parallel_interaction(uns.ReplicatedMPI(; source_order=:cyclic), vpm.UJ_fmm, vpm.Inviscid())
            a, b = cyclic_field(plain), cyclic_field(cyclic)
            plain(a)
            cyclic(b)
            @test isequal(a.particles, b.particles)
        end
        if nranks > 1
            @testset "Mixed source orders fail before evaluation" begin
                backend = uns.ReplicatedMPI(; partition=:sources,
                    source_order=rank == 0 ? :cyclic : :contiguous)
                uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
                pf = cyclic_field(uj; n=3)
                @test_throws ArgumentError uj(pf)
                @test_throws ArgumentError uns.Vvpm_on_Xs(pf, [[0.2, 0.4, -0.3]])
                close(backend)
            end
        end
    end
    rank == 0 && println("Cyclic source regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
