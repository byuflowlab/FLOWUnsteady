using Test, Random
import FLOWUnsteady as uns
const vpm = uns.vpm
const MPI = uns.MPI

MPI.Init()
const comm = MPI.COMM_WORLD
const rank = MPI.Comm_rank(comm)
const nranks = MPI.Comm_size(comm)

fixed_fmm(; ncrit=256, p=8) = vpm.FMM(; p, ncrit, min_ncrit=ncrit,
    relative_tolerance=1e-8, absolute_tolerance=1e-10,
    autotune_p=false, autotune_ncrit=false, autotune_reg_error=false)

function field(uj; n=65, R=Float64, SFS=vpm.SFS_none, fmm=fixed_fmm())
    rng = MersenneTwister(20261012)
    pf = vpm.ParticleField(n + 64, R; UJ=uj, SFS, fmm)
    for i in 1:n
        X = R.(randn(rng, 3))
        G = R.(0.01 .* randn(rng, 3))
        vpm.add_particle(pf, X, G, R(0.08 + 0.02 * rand(rng)))
    end
    return pf
end

mutable struct CallbackState
    time::Float64
    calls::Int
end

function wing_simulation()
    wing = uns.vlm.simpleWing(2.0, 6.0, 1.0, 0.0, 20.0, 0.0; n=4)
    system = uns.vlm.WingSystem()
    uns.vlm.addwing(system, "Wing", wing)
    vehicle = uns.UVLMVehicle(system; vlm_system=system, wake_system=system)
    maneuver = uns.KinematicManeuver((), (), t -> zeros(3), t -> zeros(3))
    return uns.Simulation(vehicle, maneuver, 0.0, 0.0, 0.04)
end

function rotor_simulation()
    rotor = uns.generate_rotor("DJI9443.csv"; n=3, CW=false, xfoil=false,
        read_polar=uns.vlm.ap.read_polar2, verbose=false, plot_disc=false)
    system, wake = uns.vlm.WingSystem(), uns.vlm.WingSystem()
    uns.vlm.addwing(system, "Rotor", rotor)
    uns.vlm.addwing(wake, "Rotor", rotor)
    vehicle = uns.UVLMVehicle(system; rotor_systems=([rotor],), wake_system=wake)
    maneuver = uns.KinematicManeuver((), (t -> 1.0,), t -> zeros(3), t -> zeros(3))
    return uns.Simulation(vehicle, maneuver, 0.0, 5400.0, 3/(36*90))
end

try
    @testset "MPI interactions ($nranks ranks)" begin
        for serial in (vpm.UJ_direct, vpm.UJ_fmm), R in (Float32, Float64)
            serial === vpm.UJ_fmm && R === Float32 && continue # pinned FMM's Float32 tree constructor is broken
            backend = uns.ReplicatedMPI(; check_state=true)
            uj = uns._parallel_interaction(backend, serial, vpm.Inviscid())
            reference, distributed = field(serial; R), field(uj; R)
            # Include a static actuator source and nonzero accumulated outputs.
            for pf in (reference, distributed)
                vpm.add_particle(pf, R[0.3, -0.2, 0.1], R[0, 0.01, 0], R(0.1); static=true)
                pf.particles[vpm.U_INDEX, 1:pf.np] .= R(0.25)
            end
            for reset in (true, false)
                serial(reference; reset)
                uj(distributed; reset)
                @test distributed.particles ≈ reference.particles rtol=2e-6 atol=1e-7
            end
        end

        @testset "Far-field velocity and gradient" begin
            backend = uns.ReplicatedMPI()
            uj = uns._parallel_interaction(backend, vpm.UJ_fmm, vpm.Inviscid())
            reference = field(vpm.UJ_direct; n=257, fmm=fixed_fmm(; ncrit=8, p=12))
            distributed = field(uj; n=257, fmm=fixed_fmm(; ncrit=8, p=12))
            vpm.UJ_direct(reference)
            uj(distributed)
            @test distributed.particles[vpm.U_INDEX, :] ≈ reference.particles[vpm.U_INDEX, :] rtol=3e-3 atol=1e-6
            @test distributed.particles[vpm.J_INDEX, :] ≈ reference.particles[vpm.J_INDEX, :] rtol=3e-3 atol=1e-6
        end

        @testset "Empty and uneven target partitions" begin
            uj = uns._parallel_interaction(uns.ReplicatedMPI(), vpm.UJ_direct, vpm.Inviscid())
            for n in (0, 1, 3)
                pf = field(uj; n)
                uj(pf)
                @test all(isfinite, pf.particles)
            end
        end

        @testset "SFS and RK3 stages" begin
            for serial in (vpm.UJ_direct, vpm.UJ_fmm),
                    SFS in (vpm.SFS_none, vpm.SFS_Cs_nobackscatter, vpm.SFS_Cd_twolevel_nobackscatter)
                uj = uns._parallel_interaction(uns.ReplicatedMPI(; check_state=true), serial, vpm.Inviscid())
                reference, distributed = field(serial; n=17, SFS), field(uj; n=17, SFS)
                for _ in 1:3
                    vpm.nextstep(reference, 1e-4; relax=true)
                    vpm.nextstep(distributed, 1e-4; relax=true)
                    @test distributed.particles ≈ reference.particles rtol=1e-8 atol=1e-11
                    @test distributed.t == reference.t
                    @test distributed.nt == reference.nt
                end
            end
        end

        @testset "Partitioning preserves FMM SFS neighborhoods" begin
            backend = uns.ReplicatedMPI()
            source = field(vpm.UJ_fmm; n=129, fmm=fixed_fmm(; ncrit=8, p=12))
            vpm.UJ_fmm(source; sfs=true)
            owned = uns._owned_targets(source.np, rank, nranks)
            target = uns._interaction_targets(source, owned)
            result = uns._evaluate_targets!(target, source, vpm.UJ_fmm)
            full = uns._interaction_targets(source, 1:source.np; outputs=0)
            local_sfs = uns._sfs_targets(source, owned, vpm.UJ_fmm, result, full)
            gathered = uns._gather_targets(backend, local_sfs, source.np)
            @test gathered ≈ source.particles[vpm.SFS_INDEX, 1:source.np] rtol=1e-10 atol=1e-12
        end

        @testset "Target-only surface queries" begin
            for serial in (vpm.UJ_direct, vpm.UJ_fmm)
                uj = uns._parallel_interaction(uns.ReplicatedMPI(), serial, vpm.Inviscid())
                pf = field(uj)
                Xs = [[0.2, 0.4, -0.3], [-0.1, 0.3, 0.5], [0.9, 0.1, -0.5]]
                reference = deepcopy(pf)
                np = reference.np
                for X in Xs
                    uns.add_probe(reference, X)
                end
                serial(reference)
                expected = [copy(vpm.get_U(reference, i)) for i in np+1:reference.np]
                original = copy(pf.particles)
                @test uns.Vvpm_on_Xs(pf, Xs) ≈ expected rtol=1e-9 atol=1e-12
                @test pf.particles == original
                @test pf.np == np
                root_failure(p, args...) = rank == 0 ? (vpm.add_particle(p, zeros(3), ones(3), 0.1); error("root static failure")) : nothing
                @test_throws ErrorException uns.Vvpm_on_Xs(pf, Xs; static_particles_fun=root_failure)
                @test pf.np == np
                @test isempty(uns.Vvpm_on_Xs(pf, Vector{Float64}[]))
                throwing_static(p, args...) = (vpm.add_particle(p, zeros(3), ones(3), 0.1); error("static failure"))
                @test_throws ErrorException uns.Vvpm_on_Xs(pf, Xs; static_particles_fun=throwing_static)
                @test pf.np == np
            end
        end

        @testset "Root callback mutation and stopping" begin
            backend = uns.ReplicatedMPI(; root=nranks-1, check_state=true)
            uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
            pf = field(uj; n=3)
            state = CallbackState(0.0, 0)
            callback_calls = Ref(0)
            omitted = Int[]
            function callback(sim, particles, t, dt; kwargs...)
                callback_calls[] += 1
                sim.time += dt
                sim.calls += 1
                append!(omitted, [2, 3])
                vpm.remove_particle(particles, 1)
                particles.particles[vpm.GAMMA_INDEX, 1] .*= 2
                vpm.add_particle(particles, [1.0, 2.0, 3.0], [0.01, 0.0, 0.0], 0.1)
                # A callback's extra query must execute serially on the root.
                uns.Vvpm_on_Xs(particles, [[0.5, 0.3, 0.1]])
                return true
            end
            @test uns._parallel_callback(backend, callback, state, pf, 0.0, 0.1;
                                        sync_omit_shedding=omitted)
            @test omitted == [2, 3]
            @test state.time == 0.1
            @test state.calls == 1
            @test callback_calls[] == (rank == nranks-1 ? 1 : 0)
            @test backend.callback_seconds > 0
            uns._check_replicas(backend, pf)
            @test_throws ErrorException uns._parallel_callback(backend, (args...; kwargs...) -> error("callback failure"), state, pf, 0.0, 0.1)
            @test backend.active
            # Releasing suppression edits the same captured vector on root.
            @test !uns._parallel_callback(backend,
                (args...; kwargs...) -> (empty!(omitted); false), state, pf, 0.0, 0.1;
                sync_omit_shedding=omitted)
            @test isempty(omitted)
            if nranks == 1
                @test_throws ErrorException uns._parallel_callback(backend,
                    (args...; kwargs...) -> 1, state, pf, 0.0, 0.1)
                backend.active = false
                @test !uns._parallel_callback(backend, uns._runtime_noop,
                                              state, pf, 0.0, 0.1)
                @test !backend.active
                backend.active = true
            end
        end

        @testset "Callback snapshots retain shared state" begin
            values = [1.0, 2.0]
            model = CallbackState(0.1, 2)
            original = (values, Dict("values" => values), model, model)
            snapshot = uns._callback_state(original)
            @test snapshot[1] === snapshot[2]["values"]
            @test snapshot[3] === snapshot[4]
            values[1] = -1.0
            @test snapshot[1] == [1.0, 2.0]
            local_values = zeros(2)
            local_model = CallbackState(0.0, 0)
            restored = uns._restore_callback_state(
                (local_values, Dict("values" => local_values), local_model, local_model), snapshot)
            @test restored[1] === restored[2]["values"]
            @test restored[3] === restored[4]
            @test restored[1] == [1.0, 2.0]
            @test restored[3].time == 0.1 && restored[3].calls == 2
        end

        if nranks > 1
            @testset "Replica divergence is detected collectively" begin
                backend = uns.ReplicatedMPI(; check_state=true)
                uj = uns._parallel_interaction(backend, vpm.UJ_direct, vpm.Inviscid())
                pf = field(uj; n=3)
                rank == 0 && (pf.particles[1, 1] += 1)
                @test_throws ArgumentError uj(pf)
            end
        end

        @testset "Coupled wing and output ownership" begin
            backend = uns.ReplicatedMPI(; root=nranks-1, check_state=true)
            output = MPI.bcast(rank == backend.root ? mktempdir() : nothing, comm; root=backend.root)
            simulation = wing_simulation()
            callback_count = Ref(0)
            callback(sim, pf, t, dt; kwargs...) = (callback_count[] += 1; false)
            options = (Vinf=(X, t) -> [10.0, 0.0, 0.5], sigma_vlm_surf=0.05,
                sigma_rotor_surf=0.05, sigma_vpm_overwrite=0.2, vlm_init=true,
                vpm_fmm=fixed_fmm(), max_particles=512, prompt=false, verbose=false,
                raisewarnings=false)
            pf = uns.run_simulation(simulation, 4; options..., parallel=backend,
                extra_runtime_function=callback, save_path=output)
            reference_sim = wing_simulation()
            reference = uns.run_simulation(reference_sim, 4; options...)
            @test pf.particles ≈ reference.particles rtol=1e-8 atol=1e-11
            @test simulation.t == reference_sim.t
            @test simulation.vehicle.vlm_system.sol["Gamma"] ≈ reference_sim.vehicle.vlm_system.sol["Gamma"] rtol=1e-8
            @test simulation.vehicle.system === simulation.vehicle.vlm_system
            @test simulation.vehicle.system === simulation.vehicle.wake_system
            @test callback_count[] == (rank == backend.root ? 5 : 0)
            @test uns.parallel_statistics(backend).evaluations > 0
            early = uns.run_simulation(wing_simulation(), 4; options...,
                parallel=uns.ReplicatedMPI(), tquit=0.015)
            @test early.nt == 2
            MPI.Barrier(comm)
            if rank == backend.root
                @test isfile(joinpath(output, "flowunsteadysim_pfield.4.h5"))
                @test length(filter(endswith(".log"), readdir(output))) == 1
                rm(output; recursive=true)
            end
        end

        @testset "Coupled rotor with local airfoil handles" begin
            backend = uns.ReplicatedMPI(; check_state=true)
            simulation, reference_sim = rotor_simulation(), rotor_simulation()
            options = (Vinf=(X, t) -> [0.001, 0.0, 0.0], sigma_vlm_surf=0.01,
                sigma_rotor_surf=0.01, sigma_vpm_overwrite=0.04,
                vpm_fmm=fixed_fmm(), max_particles=512, prompt=false,
                verbose=false, raisewarnings=false)
            distributed = uns.run_simulation(simulation, 3; options..., parallel=backend)
            reference = uns.run_simulation(reference_sim, 3; options...)
            @test distributed.particles ≈ reference.particles rtol=1e-8 atol=1e-11
            @test simulation.vehicle.rotor_systems[1][1]._wingsystem.sol["Gamma"] ≈ reference_sim.vehicle.rotor_systems[1][1]._wingsystem.sol["Gamma"] rtol=1e-8
            @test simulation.vehicle.system.wings[1] === simulation.vehicle.rotor_systems[1][1]
            @test simulation.vehicle.wake_system.wings[1] === simulation.vehicle.rotor_systems[1][1]
        end
    end
    rank == 0 && println("MPI regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
