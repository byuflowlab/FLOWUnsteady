using Test, Random
import FLOWUnsteady as uns
const vpm, MPI = uns.vpm, uns.MPI
MPI.Init()
const comm = MPI.COMM_WORLD
const rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)

function field(backend; n=129, fmm=vpm.FMM(; p=12, ncrit=8, min_ncrit=8,
        relative_tolerance=1e-7, absolute_tolerance=1e-10,
        autotune_p=false, autotune_ncrit=false))
    pf = vpm.ParticleField(n + 32, Float64;
        UJ=uns._parallel_interaction(backend, vpm.UJ_fmm, vpm.Inviscid()), fmm)
    rng = MersenneTwister(20261013)
    for _ in 1:n
        vpm.add_particle(pf, randn(rng, 3), 0.01 .* randn(rng, 3), 0.1)
    end
    return pf
end

try
    @testset "Near-field scheduler ($nranks ranks)" begin
        @test_throws ArgumentError uns.ReplicatedMPI(; nearfield=:invalid)
        for partition in (:targets, :sources)
            static = field(uns.ReplicatedMPI(; partition, nearfield=:static, check_state=true))
            dynamic = field(uns.ReplicatedMPI(; partition, nearfield=:dynamic, check_state=true))
            for _ in 1:3
                static.UJ(static)
                dynamic.UJ(dynamic)
                @test isequal(dynamic.particles, static.particles)
            end
            # A single target leaf must split across bodies while each body's
            # sources retain their original order. Exercise uneven/empty ranks.
            for n in (0, 1, 3, 16, 63)
                points = [[0.01i, 0.02i, -0.01i] for i in 1:n]
                @test isequal(uns.Vvpm_on_Xs(dynamic, points), uns.Vvpm_on_Xs(static, points))
            end
            @test isequal(dynamic.particles, static.particles)
        end
        static = field(uns.ReplicatedMPI(; nearfield=:static))
        dynamic = field(uns.ReplicatedMPI(; nearfield=:dynamic))
        for _ in 1:3
            static.UJ(static; sfs=true)
            dynamic.UJ(dynamic; sfs=true)
            @test isequal(dynamic.particles, static.particles)
        end
    end
    rank == 0 && println("Near-field scheduler regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
