#!/usr/bin/env julia
include(joinpath(@__DIR__, "launch_mpi.jl"))
root = normpath(joinpath(@__DIR__, ".."))
ranks = isempty(ARGS) ? [1, 2, 4] : parse.(Int, ARGS)
for n in ranks
    launch_mpi(n, joinpath(root, "test", "mpi", "runtests.jl"))
end
launch_mpi(2, joinpath(root, "test", "mpi", "io_failure.jl");
           expected_failure=true, timeout_seconds=90)
println("Root I/O failure terminated the MPI job without hanging")
