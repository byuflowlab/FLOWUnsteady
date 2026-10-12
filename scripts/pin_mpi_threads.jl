# Optional benchmark placement only. This helper does not change solver settings
# or the affinities of interactive, GC, BLAS, Python, or MPI helper threads.

function _benchmark_allowed_cpus()
    status = read("/proc/thread-self/status", String)
    line = match(r"(?m)^Cpus_allowed_list:\s*(.+)$", status)
    isnothing(line) && error("Cannot read this thread's allowed CPU list")
    cpus = Int[]
    for part in split(strip(line.captures[1]), ',')
        bounds = parse.(Int, split(part, '-'))
        length(bounds) == 1 ? push!(cpus, only(bounds)) :
            append!(cpus, bounds[1]:bounds[2])
    end
    isempty(cpus) && error("This thread has no allowed CPUs")
    return sort!(unique!(cpus))
end

function _benchmark_physical_core(cpu)
    topology = "/sys/devices/system/cpu/cpu$cpu/topology"
    package = parse(Int, strip(read(joinpath(topology, "physical_package_id"), String)))
    core = parse(Int, strip(read(joinpath(topology, "core_id"), String)))
    return (package, core)
end

function _benchmark_set_affinity!(cpus)
    # Linux cpu_set_t supports 1024 logical CPUs on the benchmark hosts.
    mask = zeros(UInt8, 128)
    isempty(cpus) && throw(ArgumentError("Cannot set an empty CPU affinity"))
    for cpu in cpus
        0 <= cpu < 8 * length(mask) || throw(ArgumentError("CPU $cpu exceeds the supported affinity mask"))
        mask[div(cpu, 8) + 1] |= UInt8(1) << mod(cpu, 8)
    end
    status = ccall(:sched_setaffinity, Cint, (Cint, Csize_t, Ptr{UInt8}),
        0, length(mask), mask)
    status == 0 || throw(SystemError("sched_setaffinity", Base.Libc.errno()))
    return nothing
end

function _benchmark_thread_record()
    cpu = Int(ccall(:sched_getcpu, Cint, ()))
    cpu >= 0 || throw(SystemError("sched_getcpu", Base.Libc.errno()))
    return Dict{String,Any}(
        "julia_thread_id" => Threads.threadid(),
        "native_tid" => Int(ccall(:gettid, Cint, ())),
        "thread_pool" => string(Threads.threadpool()),
        "allowed_cpus" => _benchmark_allowed_cpus(),
        "current_cpu" => cpu)
end

function _benchmark_default_threads(f)
    Sys.islinux() || throw(ArgumentError("Physical thread pinning requires Linux proc/sysfs and libc"))
    Threads.threadid() == 1 || throw(ArgumentError("Call thread pinning from Julia's main thread"))
    count = Threads.nthreads(:default)
    records, errors = Vector{Any}(undef, count), Any[nothing for _ in 1:count]
    Threads.@threads :static for i in 1:count
        try
            Threads.threadpool() === :default || error("Static placement did not reach a default-pool worker")
            records[i] = f(i)
        catch exception
            errors[i] = exception
        end
    end
    failures = filter(!isnothing, errors)
    isempty(failures) || throw(CompositeException(failures))
    return records
end

function _benchmark_restore_affinities!(original)
    _benchmark_default_threads() do i
        Threads.threadid() == original[i]["julia_thread_id"] || error("Default thread assignment changed")
        _benchmark_set_affinity!(original[i]["allowed_cpus"])
        _benchmark_allowed_cpus() == original[i]["allowed_cpus"] || error("Could not restore worker affinity")
        return nothing
    end
    return nothing
end

function _benchmark_physical_assignment(original)
    choices = [Dict(_benchmark_physical_core(cpu) => cpu
        for cpu in reverse(record["allowed_cpus"])) for record in original]
    cores = union((keys(choice) for choice in choices)...)
    length(cores) >= length(original) || throw(ArgumentError(
        "$(length(original)) default workers require distinct physical cores; only $(length(cores)) are allowed"))
    owner = Dict{Tuple{Int,Int},Int}()
    selected = Vector{Tuple{Int,Int}}(undef, length(original))
    function assign(worker, visited)
        for core in sort!(collect(keys(choices[worker])))
            core in visited && continue
            push!(visited, core)
            if !haskey(owner, core) || assign(owner[core], visited)
                owner[core], selected[worker] = worker, core
                return true
            end
        end
        return false
    end
    for worker in reverse(eachindex(original))
        assign(worker, Set{Tuple{Int,Int}}()) || throw(ArgumentError(
            "Worker CPU masks cannot place every default worker on a distinct physical core"))
    end
    return [choices[i][selected[i]] for i in eachindex(original)], selected
end

function _benchmark_pin_affinities!(original, cpus, physical, inspect=_benchmark_thread_record)
    try
        records = _benchmark_default_threads() do i
            Threads.threadid() == original[i]["julia_thread_id"] || error("Default thread assignment changed")
            _benchmark_set_affinity!([cpus[i]])
            record = inspect()
            record["allowed_cpus"] == [cpus[i]] || error("Worker affinity is not the selected singleton CPU")
            record["current_cpu"] == cpus[i] || error("Worker is not running on its selected CPU")
            record["original_allowed_cpus"] = original[i]["allowed_cpus"]
            record["physical_package_id"], record["physical_core_id"] = physical[i]
            return record
        end
        return Dict{String,Any}[record for record in records]
    catch exception
        try
            _benchmark_restore_affinities!(original)
        catch restore_exception
            throw(CompositeException(Any[exception, restore_exception]))
        end
        rethrow()
    end
end

"""
    pin_mpi_threads!()

Pin each Julia default-pool worker to one allowed logical CPU on a different
physical core. Call from the main thread before benchmark warmup, after MPI's
process binding. This Linux-only opt-in fails when physical cores are insufficient
or placement cannot be verified. Interactive/native/GC helpers retain their masks.
Return serializable per-worker binding records. MPI callers must collectively
agree any error before entering their next collective operation.
"""
function pin_mpi_threads!()
    original = _benchmark_default_threads(_ -> _benchmark_thread_record())
    ids = [record["julia_thread_id"] for record in original]
    length(unique(ids)) == Threads.nthreads(:default) || error("Did not inspect every default worker")
    cpus, physical = _benchmark_physical_assignment(original)
    return _benchmark_pin_affinities!(original, cpus, physical)
end
