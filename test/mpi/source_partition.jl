using Test, Random
import FLOWUnsteady as uns
const vpm, MPI = uns.vpm, uns.MPI

MPI.Init()
const comm = MPI.COMM_WORLD
const rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)

function fixed_fmm(; ncrit=256, p=8, relative_tolerance=1e-8,
        absolute_tolerance=1e-10, autotune_reg_error=false)
    return vpm.FMM(; p, ncrit, min_ncrit=ncrit, relative_tolerance,
        absolute_tolerance, autotune_p=false, autotune_ncrit=false,
        autotune_reg_error)
end

function field(uj; n=65, R=Float64, fmm=fixed_fmm())
    rng = MersenneTwister(20261012)
    pf = vpm.ParticleField(n + 64, R; UJ=uj, SFS=vpm.SFS_none, fmm)
    for _ in 1:n
        vpm.add_particle(pf, R.(randn(rng, 3)), R.(0.01 .* randn(rng, 3)),
            R(0.08 + 0.02 * rand(rng)))
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

function original_target_fmm!(target, source; gradient=true, source_indices=1:source.np)
    f = source.fmm
    return uns._particle_fmm.fmm!((target,), (uns._particle_sources(source, source_indices),);
        expansion_order=f.p - 1, leaf_size_source=max(f.ncrit, f.min_ncrit),
        multipole_acceptance=f.theta,
        error_tolerance=uns._particle_fmm.PowerRelativeGradient{f.relative_tolerance,f.absolute_tolerance,true}(),
        shrink=f.shrink_recenter, recenter=f.shrink_recenter,
        scalar_potential=false, hessian=gradient, nearfield_device=false,
        tune=false, silence_warnings=true)
end

try
    @testset "Source partition ($nranks ranks)" begin
        @test_throws ArgumentError uns.ReplicatedMPI(; partition=:invalid)

        @testset "Unused far-field passes preserve fixed FMM results" begin
            for farfield in (false, true), gradient in (false, true)
                pf = field(vpm.UJ_fmm; n=farfield ? 257 : 65,
                    fmm=fixed_fmm(; ncrit=farfield ? 8 : 256, p=12))
                target_indices = gradient ? (1:pf.np) : (1:3)
                for source_indices in (1:pf.np, uns._owned_targets(pf.np, rank, nranks))
                    original = uns._interaction_targets(pf, target_indices; outputs=gradient ? 12 : 3)
                    optimized = uns._interaction_targets(pf, target_indices; outputs=gradient ? 12 : 3)
                    expected = original_target_fmm!(original, pf; gradient, source_indices)
                    actual = uns._evaluate_targets!(optimized, pf, vpm.UJ_fmm; gradient, source_indices)
                    @test isequal(optimized.values, original.values)
                    @test actual[3].sort_index_list == expected[3].sort_index_list
                    @test actual[4].sort_index_list == expected[4].sort_index_list
                    @test actual[5] == expected[5]
                    @test actual[6] == expected[6]
                    if !farfield
                        @test isempty(actual[5])
                        @test all(iszero, actual[4].expansions)
                    elseif gradient && source_indices == 1:pf.np
                        @test !isempty(actual[5])
                    end
                end
            end
        end

        @testset "Cached source buffers preserve cutoff radii exactly" begin
            for R in (Float32, Float64), autotune in (false, true),
                    relative in (0.0, 1e-7), absolute in (0.0, 1e-10)
                pf = field(vpm.UJ_fmm; R, n=3, fmm=fixed_fmm(;
                    relative_tolerance=relative, absolute_tolerance=absolute,
                    autotune_reg_error=autotune))
                pf.particles[vpm.GAMMA_INDEX, 1] .= 0
                pf.particles[vpm.GAMMA_INDEX, 2] .*= R(1e-8)
                sources = uns._particle_sources(pf, 1:pf.np)
                original, cached = zeros(R, 8, pf.np), zeros(R, 8, pf.np)
                for i in 1:pf.np
                    uns._particle_fmm.source_system_to_buffer!(original, i, pf, i)
                    uns._particle_fmm.source_system_to_buffer!(cached, i, sources, i)
                end
                @test cached == original
            end
        end

        @testset "Absolute cutoff shortcut preserves original radii" begin
            rng = MersenneTwister(20261013)
            comparisons = 0
            for R in (Float32, Float64), relative in (0.0, 1e-14, 1e-10, 1e-7, 1e-3),
                    absolute in (0.0, 1e-14, 1e-10, 1e-6, 1e-2)
                pf = field(vpm.UJ_fmm; R, n=0, fmm=fixed_fmm(;
                    relative_tolerance=relative, absolute_tolerance=absolute,
                    autotune_reg_error=true))
                sources = uns._particle_sources(pf)
                # Cover weak/strong particles and narrow/broad smoothing over
                # many decades, including the original negligible-strength gate.
                inputs = [(R(10.0^(12rand(rng) - 8)), R(10.0^(21rand(rng) - 18)))
                    for _ in 1:256]
                append!(inputs, [(R(0.01), strength) for strength in
                    R[0, prevfloat(R(10eps())), R(10eps()), nextfloat(R(10eps()))]])
                if relative > eps(10.0) && absolute > eps(10.0)
                    cutoff = sources.relative_cutoff
                    denominator = 1 - vpm.erf(cutoff / vpm.const5) +
                        vpm.const2 * cutoff * exp(-cutoff * cutoff * 0.5)
                    for sigma in R[1e-4, 0.01, 1.0]
                        boundary = R(4pi * sigma * sigma * absolute * cutoff * cutoff / denominator)
                        for multiplier in (1 - 1e-6, 1 - 128eps(), 1.0,
                                1 + 128eps(), 1 + 1e-6)
                            strength = R(boundary * multiplier)
                            append!(inputs, [(sigma, prevfloat(strength)), (sigma, strength),
                                (sigma, nextfloat(strength))])
                        end
                    end
                end
                for (sigma, strength) in inputs
                    original = vpm.solve_ρ_over_σ(sigma, strength, relative, absolute, true, 1.0)
                    optimized = uns._regularization_cutoff(sources, sigma, strength)
                    @test isequal(optimized * sigma, original * sigma)
                    comparisons += 1
                end
            end
            @test comparisons > 14000
        end

        @testset "Velocity, gradients, static sources and accumulation" begin
            for serial in (vpm.UJ_direct, vpm.UJ_fmm), R in (Float32, Float64)
                serial === vpm.UJ_fmm && R === Float32 && continue
                uj = uns._parallel_interaction(uns.ReplicatedMPI(;
                    partition=:sources, check_state=true), serial, vpm.Inviscid())
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
                @test_throws ArgumentError uj(distributed; sfs=true)
            end
        end

        @testset "Far-field velocity and gradient" begin
            uj = uns._parallel_interaction(uns.ReplicatedMPI(; partition=:sources),
                vpm.UJ_fmm, vpm.Inviscid())
            settings = fixed_fmm(; ncrit=8, p=12)
            reference, distributed = field(vpm.UJ_direct; n=257, fmm=settings),
                field(uj; n=257, fmm=settings)
            vpm.UJ_direct(reference)
            uj(distributed)
            @test distributed.particles[vpm.U_INDEX, :] ≈ reference.particles[vpm.U_INDEX, :] rtol=3e-3 atol=1e-6
            @test distributed.particles[vpm.J_INDEX, :] ≈ reference.particles[vpm.J_INDEX, :] rtol=3e-3 atol=1e-6
        end

        @testset "Empty and uneven source partitions" begin
            for serial in (vpm.UJ_direct, vpm.UJ_fmm), n in (0, 1, 3)
                uj = uns._parallel_interaction(uns.ReplicatedMPI(; partition=:sources),
                    serial, vpm.Inviscid())
                reference, distributed = field(serial; n), field(uj; n)
                serial(reference)
                uj(distributed)
                @test distributed.particles ≈ reference.particles rtol=1e-10 atol=1e-12
            end
        end

        @testset "Surface queries and temporary actuator sources" begin
            for serial in (vpm.UJ_direct, vpm.UJ_fmm)
                uj = uns._parallel_interaction(uns.ReplicatedMPI(; partition=:sources),
                    serial, vpm.Inviscid())
                reference, distributed = field(serial), field(uj)
                Xs = [[0.2, 0.4, -0.3], [-0.1, 0.3, 0.5], [0.9, 0.1, -0.5]]
                function static_source(pf, args...)
                    vpm.add_particle(pf, [0.3, -0.2, 0.1], [0.0, 0.01, 0.0], 0.1; static=true)
                end
                original = copy(distributed.particles)
                expected = uns.Vvpm_on_Xs(reference, Xs; static_particles_fun=static_source)
                @test uns.Vvpm_on_Xs(distributed, Xs; static_particles_fun=static_source) ≈ expected rtol=1e-9 atol=1e-12
                @test distributed.particles[:, 1:distributed.np] == original[:, 1:distributed.np]
                @test distributed.np == reference.np
                @test isempty(uns.Vvpm_on_Xs(distributed, Vector{Float64}[]))
            end
        end

        @testset "Coupled Euler and RK3 without SFS" begin
            for integration in (vpm.euler, vpm.rungekutta3)
                options = (; Vinf=(X, t) -> [10.0, 0.0, 0.5], sigma_vlm_surf=0.05,
                    sigma_rotor_surf=0.05, sigma_vpm_overwrite=0.2, vlm_init=true,
                    vpm_fmm=fixed_fmm(), vpm_integration=integration,
                    vpm_SFS=vpm.SFS_none, max_particles=512, prompt=false,
                    verbose=false, raisewarnings=false)
                simulation, reference_sim = wing_simulation(), wing_simulation()
                backend = uns.ReplicatedMPI(; partition=:sources, check_state=true)
                distributed = uns.run_simulation(simulation, 4; options..., parallel=backend)
                reference = uns.run_simulation(reference_sim, 4; options...)
                @test distributed.particles ≈ reference.particles rtol=1e-8 atol=1e-11
                @test distributed.t == reference.t
                @test distributed.nt == reference.nt
                @test simulation.vehicle.vlm_system.sol["Gamma"] ≈ reference_sim.vehicle.vlm_system.sol["Gamma"] rtol=1e-8
            end
        end
    end
    rank == 0 && println("Source partition regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
