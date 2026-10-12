#!/usr/bin/env julia
# Use the launcher matching MPI.jl, rather than assuming the system mpiexec ABI.
import FLOWUnsteady
function launch_mpi(n, script, args=String[]; expected_failure=false, timeout_seconds=600)
    n > 0 || error("Rank counts must be positive")
    environment = normpath(joinpath(@__DIR__, "..", "environments", "mpi"))
    command = `$(FLOWUnsteady.MPI.mpiexec()) -n $n $(Base.julia_cmd()) --startup-file=no --project=$environment $script $args`
    process = run(pipeline(addenv(command, "MPLBACKEND" => "Agg", "OPENBLAS_NUM_THREADS" => "1"),
                           stdout=stdout, stderr=stderr); wait=false)
    timed_out = Ref(false)
    watchdog = Timer(timeout_seconds) do timer
        timed_out[] = true
        kill(process)
    end
    wait(process)
    close(watchdog)
    timed_out[] && error("MPI job timed out after $timeout_seconds seconds")
    success(process) == !expected_failure || error("Unexpected MPI job result on $n ranks (exit $(process.exitcode))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    length(ARGS) >= 2 || error("Usage: launch_mpi.jl RANKS SCRIPT [ARGS...]")
    launch_mpi(parse(Int, ARGS[1]), abspath(ARGS[2]), ARGS[3:end])
end
