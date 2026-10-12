# Rectangular source x target decomposition. Main communicator rank = s*T + t;
# each source column has the same target block, each target row the same sources.
function _hybrid_domains(p::ReplicatedMPI, ntargets, nsources)
    target_groups = p.target_groups
    source_groups = div(p.nranks, target_groups)
    source, target = divrem(p.rank, target_groups)
    return _owned_targets(ntargets, target, target_groups),
           _owned_sources(nsources, source, source_groups, p.source_order)
end

function _ensure_hybrid_communicators!(p::ReplicatedMPI)
    source, target = divrem(p.rank, p.target_groups)
    if p.source_column == MPI.COMM_NULL
        p.source_column = MPI.Comm_split(p.comm, target, source)
        p.target_row = MPI.Comm_split(p.comm, source, target)
    end
    return nothing
end

function _hybrid_values!(p::ReplicatedMPI, values::Matrix, n)
    target_groups = p.target_groups
    source_groups = div(p.nranks, target_groups)
    target = mod(p.rank, target_groups)
    expected = length(_owned_targets(n, target, target_groups))
    err = size(values, 2) == expected ? nothing :
        ArgumentError("Hybrid result width does not match its target block")
    if isnothing(err) && p.source_column != MPI.COMM_NULL
        if MPI.Comm_size(p.source_column) != source_groups ||
                MPI.Comm_size(p.target_row) != target_groups
            err = ArgumentError("Close the MPI backend before changing hybrid target_groups")
        end
    end
    _collective_error(p, err)
    # Degenerate grids are the existing source-only/target-only algorithms.
    target_groups == 1 && return _sum_sources(p, values)
    source_groups == 1 && return _gather_targets(p, values, n)

    rows = size(values, 1)
    gathered, counts = nothing, nothing
    err = nothing
    try
        gathered = Matrix{eltype(values)}(undef, rows, n)
        counts = [rows * length(_owned_targets(n, t, target_groups)) for t in 0:target_groups-1]
    catch exception
        err = exception
    end
    _collective_error(p, err)
    _ensure_hybrid_communicators!(p)
    started = time_ns()
    err = nothing
    try
        MPI.Allreduce!(MPI.IN_PLACE, values, +, p.source_column)
    catch exception
        err = exception
    end
    p.exchange_seconds += (time_ns() - started) * 1e-9
    # All main-communicator ranks agree before entering the next subgroup.
    _collective_error(p, err)
    started = time_ns()
    err = nothing
    try
        MPI.Allgatherv!(vec(values), MPI.VBuffer(vec(gathered), counts), p.target_row)
    catch exception
        err = exception
    end
    p.exchange_seconds += (time_ns() - started) * 1e-9
    _collective_error(p, err)
    return gathered
end

"""
    close(backend::ReplicatedMPI)

Collectively release the hybrid backend's child communicators on every rank,
outside runtime callbacks and before `MPI.Finalize()`. The caller-owned main
communicator remains open. Repeated calls are harmless, and a later collective
interaction recreates the child communicators. Non-hybrid backends need no cleanup.
"""
function Base.close(p::ReplicatedMPI)
    p.active || throw(ArgumentError("Close the MPI backend outside runtime callbacks"))
    if p.source_column != MPI.COMM_NULL
        MPI.free(p.source_column)
        p.source_column = MPI.COMM_NULL
    end
    if p.target_row != MPI.COMM_NULL
        MPI.free(p.target_row)
        p.target_row = MPI.COMM_NULL
    end
    return nothing
end
