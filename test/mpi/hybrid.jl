using Test, Random
import FLOWUnsteady as uns
const vpm, MPI = uns.vpm, uns.MPI
MPI.Init()
const comm = MPI.COMM_WORLD
const rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)

function field(uj; n=65, R=Float64, fmm=vpm.FMM(; p=12, ncrit=256, min_ncrit=256,
        relative_tolerance=1e-7, absolute_tolerance=1e-10,
        autotune_p=false, autotune_ncrit=false))
    pf = vpm.ParticleField(n + 64, R; UJ=uj, fmm)
    rng = MersenneTwister(20261013)
    for _ in 1:n
        vpm.add_particle(pf, R.(randn(rng, 3)), R.(0.01 .* randn(rng, 3)), R(0.1))
    end
    return pf
end

function wing_simulation()
    wing = uns.vlm.simpleWing(2.0, 6.0, 1.0, 0.0, 20.0, 0.0; n=4)
    system = uns.vlm.WingSystem()
    uns.vlm.addwing(system, "Wing", wing)
    vehicle = uns.UVLMVehicle(system; vlm_system=system, wake_system=system)
    maneuver = uns.KinematicManeuver((), (), t -> zeros(3), t -> zeros(3))
    return uns.Simulation(vehicle, maneuver, 0.0, 0.0, 0.04)
end

function clustered_field(uj)
    settings = vpm.FMM(; p=12, ncrit=16, min_ncrit=16, theta=0.4,
        relative_tolerance=1e-7, absolute_tolerance=1e-10,
        autotune_p=false, autotune_ncrit=false)
    pf = vpm.ParticleField(288, Float64; UJ=uj, fmm=settings)
    rng = MersenneTwister(20261014)
    for cluster in (-1, 1), _ in 1:128
        position = [5.0cluster, 0.0, 0.0] + 0.15 .* randn(rng, 3)
        vpm.add_particle(pf, position, 0.001 .* randn(rng, 3), 0.002)
    end
    return pf
end

try
    @testset "Hybrid interactions ($nranks ranks)" begin
        @test_throws ArgumentError uns.ReplicatedMPI(; partition=:hybrid, target_groups=0)
        @test_throws ArgumentError uns.ReplicatedMPI(; partition=:hybrid, target_groups=nranks+1)
        for groups in filter(t -> nranks % t == 0, 1:nranks)
            @testset "Target groups $groups" begin
                for serial in (vpm.UJ_direct, vpm.UJ_fmm), R in (Float32, Float64)
                    serial === vpm.UJ_fmm && R === Float32 && continue
                    backend = uns.ReplicatedMPI(; partition=:hybrid, target_groups=groups,
                        root=nranks-1, nearfield=:dynamic, check_state=true)
                    try
                        uj = uns._parallel_interaction(backend, serial, vpm.Inviscid())
                        reference, distributed = field(serial; R), field(uj; R)
                        for pf in (reference, distributed)
                            vpm.add_particle(pf, R[0.3, -0.2, 0.1], R[0, 0.01, 0], R(0.1); static=true)
                            pf.particles[vpm.U_INDEX, 1:pf.np] .= R(0.25)
                        end
                        for reset in (true, false)
                            serial(reference; reset)
                            uj(distributed; reset)
                            @test distributed.particles ≈ reference.particles rtol=2e-6 atol=1e-7
                        end
                        for n in (0, 1, 3, 16)
                            points = [R[0.01i, 0.02i, -0.01i] for i in 1:n]
                            got, expected = uns.Vvpm_on_Xs(distributed, points),
                                            uns.Vvpm_on_Xs(reference, points)
                            @test length(got) == length(expected) == n
                            @test all(i -> isapprox(got[i], expected[i]; rtol=2e-6, atol=1e-7),
                                      eachindex(got, expected))
                        end
                        @test_throws ArgumentError uj(distributed; sfs=true)
                        close(backend)
                        close(backend)
                        @test backend.source_column == MPI.COMM_NULL
                        @test backend.target_row == MPI.COMM_NULL
                        # Reusing a returned field after cleanup recreates its grid.
                        uj(distributed)
                        serial(reference)
                        @test distributed.particles ≈ reference.particles rtol=2e-6 atol=1e-7
                    finally
                        close(backend)
                    end
                end
                backend = uns.ReplicatedMPI(; partition=:hybrid, target_groups=groups)
                try
                    uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
                    for n in (0, 1, 3)
                        pf = field(uj; n)
                        uj(pf)
                        @test all(isfinite, pf.particles)
                    end
                    callback_field = field(uj; n=3)
                    @test_throws ErrorException uns._parallel_callback(backend,
                        (args...; kwargs...) -> (close(backend); false), Ref(0), callback_field, 0.0, 0.1)
                    @test backend.active
                    # One rank fails before subgroup entry; every rank must throw.
                    owned, _ = uns._hybrid_domains(backend, 3, 3)
                    values = zeros(3, length(owned) + (rank == 0 ? 1 : 0))
                    @test_throws ErrorException uns._hybrid_values!(backend, values, 3)
                finally
                    close(backend)
                end
            end
        end
        @testset "Genuine far-field translations" begin
            for groups in filter(t -> nranks % t == 0, 1:nranks)
                backend = uns.ReplicatedMPI(; partition=:hybrid, target_groups=groups,
                    nearfield=:dynamic, check_state=true)
                try
                    uj = uns._parallel_interaction(backend, vpm.UJ_fmm, vpm.Inviscid())
                    reference, direct, distributed = clustered_field(vpm.UJ_fmm),
                        clustered_field(vpm.UJ_direct), clustered_field(uj)
                    before = copy(distributed.particles)
                    full = uns._interaction_targets(distributed, 1:distributed.np)
                    full_result = uns._evaluate_targets!(full, distributed, vpm.UJ_fmm)
                    @test !isempty(full_result[5])
                    owned, sources = uns._hybrid_domains(backend, distributed.np, distributed.np)
                    local_targets = uns._interaction_targets(distributed, owned)
                    result = uns._evaluate_targets!(local_targets, distributed, vpm.UJ_fmm;
                        source_indices=sources, nearfield=:dynamic)
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
        if nranks > 1
            @testset "Mixed layouts fail before subgroup collectives" begin
                backend = uns.ReplicatedMPI(; partition=rank == 0 ? :hybrid : :sources)
                uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
                @test_throws ArgumentError uj(field(uj; n=3))
                close(backend)
                backend = uns.ReplicatedMPI(; partition=:hybrid,
                    target_groups=rank == 0 ? 1 : nranks)
                uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
                @test_throws ArgumentError uj(field(uj; n=3))
                close(backend)
            end
        end
        @testset "Coupled wing and root ownership" begin
            groups = nranks % 2 == 0 ? 2 : 1
            backend = uns.ReplicatedMPI(; partition=:hybrid, target_groups=groups,
                root=nranks-1, nearfield=:dynamic, check_state=true)
            try
                options = (; Vinf=(X, t) -> [10.0, 0.0, 0.5], sigma_vlm_surf=0.05,
                    sigma_rotor_surf=0.05, sigma_vpm_overwrite=0.2, vlm_init=true,
                    max_particles=512, prompt=false, verbose=false, raisewarnings=false,
                    vpm_fmm=vpm.FMM(; p=12, ncrit=256, min_ncrit=256,
                        relative_tolerance=1e-7, absolute_tolerance=1e-10,
                        autotune_p=false, autotune_ncrit=false))
                calls = Ref(0)
                callback(args...; kwargs...) = (calls[] += 1; false)
                simulation, reference_sim = wing_simulation(), wing_simulation()
                distributed = uns.run_simulation(simulation, 4; options..., parallel=backend,
                    extra_runtime_function=callback)
                reference = uns.run_simulation(reference_sim, 4; options...)
                @test distributed.particles ≈ reference.particles rtol=1e-8 atol=1e-11
                @test simulation.vehicle.vlm_system.sol["Gamma"] ≈
                      reference_sim.vehicle.vlm_system.sol["Gamma"] rtol=1e-8
                @test simulation.vehicle.system === simulation.vehicle.wake_system
                @test calls[] == (rank == backend.root ? 5 : 0)
            finally
                close(backend)
            end
        end
    end
    rank == 0 && println("Hybrid regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
