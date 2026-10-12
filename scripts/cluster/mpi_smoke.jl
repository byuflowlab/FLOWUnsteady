using Sockets, SHA, LinearAlgebra, Libdl
import FLOWUnsteady as uns
const MPI = uns.MPI
MPI.Init()
BLAS.set_num_threads(1)
rank, ranks = MPI.Comm_rank(MPI.COMM_WORLD), MPI.Comm_size(MPI.COMM_WORLD)
hosts = MPI.gather(gethostname(), MPI.COMM_WORLD; root=0)
versions = MPI.gather(MPI.Get_library_version(), MPI.COMM_WORLD; root=0)
mpi_libraries = filter(path -> startswith(basename(path), "libmpi"), Libdl.dllist())
@assert !isempty(mpi_libraries)
@assert all(startswith(path, "/opt/amazon/openmpi5/") for path in mpi_libraries)
state = fill(Float64(rank), 100_000)
received = reshape(MPI.Allgather(state, MPI.COMM_WORLD), length(state), ranks)
@assert size(received, 2) == ranks
@assert all(all(received[:, i] .== i - 1) for i in 1:ranks)
if rank == 0
    println("MPI library: ", MPI.Get_library_version())
    @assert all(==(first(versions)), versions) "MPI library versions differ between nodes"
    println("Rank hosts: ", hosts)
    @assert length(unique(hosts)) == ranks "Expected one rank on each distinct compute node"
    println("Allgather across $ranks nodes passed (800000 bytes/rank)")
    println("Loaded MPI libraries: ", mpi_libraries)
end
MPI.Finalize()
