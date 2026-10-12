using Test
include(joinpath(@__DIR__, "..", "..", "examples", "mpi", "rotorhover.jl"))
MPI.Init()
const comm = MPI.COMM_WORLD
const rank, nranks = MPI.Comm_rank(comm), MPI.Comm_size(comm)

function test_rotor(; n=8, CW=false)
    return uns.generate_rotor("DJI9443.csv"; n, CW, xfoil=false,
        read_polar=vlm.ap.read_polar2, verbose=false, plot_disc=false)
end

function polar_state(rotor)
    return [(; polar, python=polar.pyPolar,
        python_fields=deepcopy(Dict(polar.pyPolar.__dict__)),
        alpha=copy(vlm.ap.get_cl(polar)[1]), cl=copy(vlm.ap.get_cl(polar)[2]),
        cd=copy(vlm.ap.get_cd(polar)[2]), cm=copy(vlm.ap.get_cm(polar)[2]))
        for polar in rotor._polars]
end

function check_polar_state(rotor, before)
    for (polar, state) in zip(rotor._polars, before)
        @test polar === state.polar
        @test polar.pyPolar === state.python
        @test isequal(Dict(polar.pyPolar.__dict__), state.python_fields)
        @test vlm.ap.get_cl(polar)[1] == state.alpha
        @test vlm.ap.get_cl(polar)[2] == state.cl
        @test vlm.ap.get_cd(polar)[2] == state.cd
        @test vlm.ap.get_cm(polar)[2] == state.cm
    end
end

try
    @testset "Distributed rotor polars ($nranks ranks)" begin
        backend = uns.ReplicatedMPI(; root=nranks-1, rotor_parallel=true, check_state=true)
        @testset "Original converter coefficients and geometry" begin
            for CW in (false, true)
                rotor = test_rotor(; CW)
                before = polar_state(rotor)
                for (RPM, axial, sound, correction, extrapolate, turbine, extrapolate_AR) in
                        ((5400.0, 0.001, 342.35, true, true, false, true),
                         (3200.0, 3.0, 342.35, false, true, true, false),
                         (6000.0, -2.0, 342.35, true, false, false, true),
                         (4300.0, 2.0, nothing, true, true, false, true),
                         (4300.0, 2.0, nothing, false, false, false, true))
                    rotor.turbine_flag = turbine
                    Vinf = (X, t) -> [axial, 0.4, -0.1]
                    vlm.calc_inflow(rotor, Vinf, RPM)
                    opts = (; sound_spd=sound, AR_to_360extrap=extrapolate_AR, apply_correction3D=correction,
                        apply_extrapolate=extrapolate)
                    distributed = uns._parallel_rotor_polars(backend, rotor, RPM; opts...)
                    for blade in 1:rotor.B
                        original = vlm.FLOWVLM2OCCBlade(rotor, RPM, blade, turbine; opts...)
                        @test distributed[blade].r == original.r
                        @test distributed[blade].chord == original.chord
                        @test distributed[blade].theta == original.theta
                        for (af, reference) in zip(distributed[blade].af, original.af)
                            for field in (:cl, :cd)
                                spline, expected = getproperty(af, field), getproperty(reference, field)
                                @test isequal(spline.t, expected.t)
                                @test isequal(spline.c, expected.c)
                                @test spline.k == expected.k
                                @test spline.bc == expected.bc
                                @test isequal(spline.fp, expected.fp)
                                @test isequal(spline([-2.5, -0.4, 0.0, 0.8, 2.6]),
                                    expected([-2.5, -0.4, 0.0, 0.8, 2.6]))
                            end
                        end
                    end
                end
                check_polar_state(rotor, before)
            end
        end

        @testset "Small rotor and empty owned partitions" begin
            rotor = test_rotor(; n=2)
            vlm.calc_inflow(rotor, (X, t) -> [0.01, 0.0, 0.0], 5400.0)
            distributed = uns._parallel_rotor_polars(backend, rotor, 5400.0; sound_spd=342.35)
            for blade in 1:rotor.B
                original = vlm.FLOWVLM2OCCBlade(rotor, 5400.0, blade, false; sound_spd=342.35)
                @test all(isequal(af.cl.c, expected.cl.c) && isequal(af.cd.c, expected.cd.c)
                    for (af, expected) in zip(distributed[blade].af, original.af))
            end
        end

        @testset "Full lookup solve retains fields and aliases" begin
            for (CW, turbine) in ((false, false), (true, false), (false, true))
                reference, distributed = test_rotor(; CW), test_rotor(; CW)
                reference.turbine_flag = turbine
                distributed.turbine_flag = turbine
                before = polar_state(distributed)
                system = distributed._wingsystem
                blades = copy(system.wings)
                pf = vpm.ParticleField(1; UJ=uns._parallel_interaction(backend,
                    vpm.UJ_direct, vpm.Inviscid()))
                for step in 1:3
                    Vinf = (X, t) -> [0.3step, 0.2, -0.1]
                    Vind = [[Float64[0.1 * blade, 0.03 * element, -0.02 * step]
                        for element in 1:vlm.get_mBlade(reference)] for blade in 1:reference.B]
                    options = (; t=0.001step, sound_spd=342.35, verbosewarn=false,
                        hubtiploss_correction=((0.4, 5, 0.1, 0.05), (2, 1, 0.25, 0.05)))
                    vlm.solvefromV(reference, Vind, Vinf, 5400.0, 1.071778; options...)
                    uns._parallel_solvefromV(pf, distributed, Vind, Vinf, 5400.0, 1.071778; options...)
                    @test isequal(distributed.sol, reference.sol)
                    @test isequal(distributed._wingsystem.sol, reference._wingsystem.sol)
                    @test distributed._wingsystem === system
                    @test all(distributed._wingsystem.wings[i] === blades[i] for i in eachindex(blades))
                end
                check_polar_state(distributed, before)
                # Debug and base-polar mutation cases use the original solver.
                @test isnothing(uns._parallel_solvefromV(pf, distributed,
                    [[zeros(3) for _ in 1:8] for _ in 1:2],
                    (X, t) -> [0.01, 0.0, 0.0], 5400.0, 1.071778;
                    sound_spd=nothing, apply_correction3D=false,
                    apply_extrapolate=true, verbosewarn=false))
                if nranks > 1
                    @test_throws ArgumentError uns._parallel_solvefromV(pf, distributed,
                        [[zeros(3) for _ in 1:8] for _ in 1:2],
                        (X, t) -> [0.01, 0.0, 0.0], 5400.0, 1.071778;
                        sound_spd=342.35, debug=rank == 0, verbosewarn=false)
                end
            end
        end

        @testset "Collective polar failure" begin
            rotor = test_rotor(; n=2)
            vlm.calc_inflow(rotor, (X, t) -> [1000.0, 0.0, 0.0], 5400.0)
            @test_throws ErrorException uns._parallel_rotor_polars(backend, rotor, 5400.0; sound_spd=342.35)
        end

        @testset "Coupled hover and ownership" begin
            config = parse_options(["--n", "4", "--revolutions", "0.1111111111111111",
                "--steps-per-rev", "36", "--p-per-step", "1", "--warmup-steps", "4",
                "--output", "/tmp/unused-rotor-parallel-test"])
            reference_case, distributed_case = make_case(config, 4), make_case(config, 4)
            p = uns.ReplicatedMPI(; partition=:sources, nearfield=:dynamic,
                callback_sync=:readonly, rotor_parallel=true, check_state=true)
            reference = run_case(reference_case, 4, nothing, "deferred")
            distributed = run_case(distributed_case, 4, p, "deferred")
            @test distributed.particles ≈ reference.particles rtol=1e-8 atol=1e-11
            @test distributed_case.rotor._wingsystem.sol["Gamma"] ≈ reference_case.rotor._wingsystem.sol["Gamma"] rtol=1e-8
            if rank == p.root
                @test length(distributed_case.histories) == length(reference_case.histories)
                @test all(isapprox(getproperty(got, name), getproperty(expected, name);
                    rtol=1e-8, atol=1e-11) for (got, expected) in
                    zip(distributed_case.histories, reference_case.histories) for name in keys(got))
            else
                @test isempty(distributed_case.histories)
            end
            @test distributed_case.simulation.vehicle.system.wings[1] === distributed_case.rotor
            @test distributed_case.simulation.vehicle.wake_system.wings[1] === distributed_case.rotor
            @test nranks == 1 || p.rotor_seconds > 0
        end
    end
    rank == 0 && println("Distributed rotor regression checks passed on $nranks ranks")
    MPI.Barrier(comm)
    MPI.Finalize()
catch exception
    showerror(stderr, exception, catch_backtrace())
    println(stderr)
    MPI.Abort(comm, 1)
end
