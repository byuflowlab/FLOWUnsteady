using Test
include(joinpath(@__DIR__, "..", "scripts", "pin_mpi_threads.jl"))

function native_masks()
    masks = Dict{Int,String}()
    for tid in readdir("/proc/self/task")
        status = read("/proc/self/task/$tid/status", String)
        masks[parse(Int, tid)] = strip(match(r"(?m)^Cpus_allowed_list:\s*(.+)$", status).captures[1])
    end
    return masks
end

@testset "Benchmark physical thread placement" begin
    original = _benchmark_default_threads(_ -> _benchmark_thread_record())
    original_masks = native_masks()
    try
        bindings = pin_mpi_threads!()
        @test length(bindings) == Threads.nthreads(:default)
        @test length(unique(record["julia_thread_id"] for record in bindings)) == length(bindings)
        @test length(unique(record["native_tid"] for record in bindings)) == length(bindings)
        @test length(unique((record["physical_package_id"], record["physical_core_id"])
            for record in bindings)) == length(bindings)
        for record in bindings
            @test record["thread_pool"] == "default"
            @test record["allowed_cpus"] == [record["current_cpu"]]
            @test only(record["allowed_cpus"]) in record["original_allowed_cpus"]
            @test _benchmark_physical_core(record["current_cpu"]) ==
                (record["physical_package_id"], record["physical_core_id"])
        end
        # Repeated placement preserves each already distinct physical core.
        again = pin_mpi_threads!()
        @test [record["allowed_cpus"] for record in again] ==
            [record["allowed_cpus"] for record in bindings]
        current_masks = native_masks()
        default_ids = Set(record["native_tid"] for record in bindings)
        @test all(tid in default_ids || current_masks[tid] == mask
            for (tid, mask) in original_masks if haskey(current_masks, tid))
    finally
        _benchmark_restore_affinities!(original)
    end
    restored = _benchmark_default_threads(_ -> _benchmark_thread_record())
    @test [record["allowed_cpus"] for record in restored] ==
        [record["allowed_cpus"] for record in original]

    cpus, physical = _benchmark_physical_assignment(original)
    failed_thread = original[1]["julia_thread_id"]
    inspect() = Threads.threadid() == failed_thread ? error("Injected post-pin verification failure") :
        _benchmark_thread_record()
    @test_throws CompositeException _benchmark_pin_affinities!(original, cpus, physical, inspect)
    restored = _benchmark_default_threads(_ -> _benchmark_thread_record())
    @test [record["allowed_cpus"] for record in restored] ==
        [record["allowed_cpus"] for record in original]

    if Threads.nthreads(:default) > 1
        common = intersect((record["allowed_cpus"] for record in original)...)
        @test !isempty(common)
        cpu = first(common)
        try
            _benchmark_default_threads() do _
                _benchmark_set_affinity!([cpu])
            end
            @test_throws ArgumentError pin_mpi_threads!()
            failed = _benchmark_default_threads(_ -> _benchmark_thread_record())
            @test all(record -> record["allowed_cpus"] == [cpu], failed)
        finally
            _benchmark_restore_affinities!(original)
        end
        restored = _benchmark_default_threads(_ -> _benchmark_thread_record())
        @test [record["allowed_cpus"] for record in restored] ==
            [record["allowed_cpus"] for record in original]
    end
end
