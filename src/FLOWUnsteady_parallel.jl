# Replicated sources, disjoint interaction targets. No MPI operation is performed
# until the caller explicitly initializes MPI and requests this backend.
const _particle_fmm = vpm.fmm

"""
    ReplicatedMPI(comm=MPI.COMM_WORLD; root=0, check_state=false, partition=:targets,
                  nearfield=:static, callback_sync=:full, rotor_parallel=false,
                  target_groups=1, source_order=:contiguous)

MPI backend for accelerating one simulation. Initialize MPI before construction
and call `run_simulation(...; parallel=backend)` on every rank. Every rank retains
the full particle field. Only `root` executes callbacks and writes output.
`check_state=true` additionally checks particle replicas before each evaluation.
`partition=:sources` instead divides sources and sums their contributions; it
currently supports interactions without SFS.
`source_order=:cyclic` distributes every `S`th source to each source group,
spreading chronological wake ages; the default keeps contiguous source blocks.
`partition=:hybrid` divides sources and targets over a rectangular rank grid.
`target_groups` must divide the communicator size. Call `close(backend)` on every
rank after its final interaction to release lazily created child communicators.
`nearfield=:dynamic` schedules disjoint target-body ranges across local threads,
preserving each body's source order.
`callback_sync=:readonly` skips simulation/particle snapshots and synchronization
for callbacks that only read solver state. The stop flag and driver's
`omit_shedding` control still synchronize. This opt-in contract also requires
identical deterministic solver functions on every rank. With `check_state=true`,
serializable callback mutations and independently evolved particle divergence
are checked; changes hidden in native handles or captured closures are not checked.
`rotor_parallel=true` distributes independent polar corrections for the standard
wake-coupled rotor lookup solve. Debug and unsupported options retain the original solver.
"""
mutable struct ReplicatedMPI
    comm::MPI.Comm
    rank::Int
    nranks::Int
    root::Int
    check_state::Bool
    partition::Symbol
    target_groups::Int
    source_order::Symbol
    source_column::MPI.Comm
    target_row::MPI.Comm
    nearfield::Symbol
    callback_sync::Symbol
    rotor_parallel::Bool
    active::Bool
    evaluation_seconds::Float64
    exchange_seconds::Float64
    sfs_seconds::Float64
    probe_seconds::Float64
    callback_seconds::Float64
    rotor_seconds::Float64
    evaluations::Int
end

function ReplicatedMPI(comm=MPI.COMM_WORLD; root=0, check_state=false, partition=:targets,
        nearfield=:static, callback_sync=:full, rotor_parallel::Bool=false,
        target_groups::Int=1, source_order=:contiguous)
    MPI.Initialized() || throw(ArgumentError("Call MPI.Init() before constructing ReplicatedMPI"))
    MPI.Finalized() && throw(ArgumentError("MPI has already been finalized"))
    nranks = MPI.Comm_size(comm)
    0 <= root < nranks || throw(ArgumentError("root must be a rank in the communicator"))
    partition in (:targets, :sources, :hybrid) || throw(ArgumentError("partition must be :targets, :sources or :hybrid"))
    target_groups > 0 || throw(ArgumentError("target_groups must be positive"))
    partition === :hybrid && nranks % target_groups != 0 &&
        throw(ArgumentError("target_groups must divide the communicator size"))
    source_order in (:contiguous, :cyclic) || throw(ArgumentError("source_order must be :contiguous or :cyclic"))
    nearfield in (:static, :dynamic) || throw(ArgumentError("nearfield must be :static or :dynamic"))
    callback_sync in (:full, :readonly) || throw(ArgumentError("callback_sync must be :full or :readonly"))
    return ReplicatedMPI(comm, MPI.Comm_rank(comm), nranks, root, check_state, partition,
                         target_groups, source_order, MPI.COMM_NULL, MPI.COMM_NULL, nearfield,
                         callback_sync, rotor_parallel, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
end

_parallel_root(::Nothing) = true
_parallel_root(p::ReplicatedMPI) = p.rank == p.root
_owned_targets(n, rank, nranks) = (div(n * rank, nranks) + 1):div(n * (rank + 1), nranks)
function _owned_sources(n, rank, nranks, order)
    order === :contiguous && return _owned_targets(n, rank, nranks)
    order === :cyclic && return (rank + 1):nranks:n
    throw(ArgumentError("source_order must be :contiguous or :cyclic"))
end

struct _InteractionTargets{R}
    positions::Matrix{R}
    previous_velocity::Vector{R}
    values::Matrix{R}
end

Base.eltype(::_InteractionTargets{R}) where R = R
_particle_fmm.get_n_bodies(t::_InteractionTargets) = size(t.positions, 2)
_particle_fmm.get_position(t::_InteractionTargets, i) = SVector(t.positions[1, i], t.positions[2, i], t.positions[3, i])

# These metadata methods belong to the pinned FastMultipole API. Keeping the
# definitions conditional allows existing serial installations to keep loading.
const _has_target_api = isdefined(_particle_fmm, :metadata_per_body)
if _has_target_api
    _particle_fmm.metadata_per_body(::_InteractionTargets) = 2
    _particle_fmm.previous_potential_metadata_index(::_InteractionTargets) = 1
    _particle_fmm.previous_gradient_metadata_index(::_InteractionTargets) = 2
    function _particle_fmm.metadata_to_buffer!(buffer, switch, ib, t::_InteractionTargets, i)
        buffer[_particle_fmm.metadata_index(switch, 1), ib] = zero(eltype(t))
        buffer[_particle_fmm.metadata_index(switch, 2), ib] = t.previous_velocity[i]
    end
end

function _particle_fmm.buffer_to_target_system!(t::_InteractionTargets, i,
        switch::_particle_fmm.DerivativesSwitch{PS,VS,GS}, buffer, ib) where {PS,VS,GS}
    if VS
        t.values[1:3, i] .+= _particle_fmm.get_gradient(buffer, switch, ib)
    end
    if GS
        hessian = _particle_fmm.get_hessian(buffer, switch, ib)
        for j in 1:9
            t.values[j + 3, i] += hessian[j]
        end
    end
    return nothing
end

function _interaction_targets(pfield, indices; outputs=12)
    positions = Matrix(pfield.particles[vpm.X_INDEX, indices])
    previous = [sqrt(sum(abs2, vpm.get_U(pfield, i))) for i in indices]
    return _InteractionTargets(positions, previous, zeros(eltype(pfield), outputs, length(indices)))
end

# A source-only adapter retains the pinned ParticleField kernels while allowing
# a rank to buffer only its owned sources. The relative regularization cutoff
# depends only on the fixed FMM tolerance, so solve that identical root once.
struct _ParticleSources{P,R,I<:AbstractVector{Int}}
    pfield::P
    indices::I
    relative_cutoff::R
    nearfield::Symbol
end

Base.eltype(s::_ParticleSources) = eltype(s.pfield)
_particle_fmm.get_n_bodies(s::_ParticleSources) = length(s.indices)
_particle_fmm.get_position(s::_ParticleSources, i) = _particle_fmm.get_position(s.pfield, s.indices[i])
_particle_fmm.data_per_body(::_ParticleSources) = 8
_particle_fmm.strength_dims(::_ParticleSources) = 3
_particle_fmm.has_vector_potential(::_ParticleSources) = true

function _particle_sources(pfield, indices=1:pfield.np; nearfield=:static)
    f = pfield.fmm
    relative_cutoff = f.autotune_reg_error && f.relative_tolerance > eps(10.0) ?
        vpm.solve_ρ_over_σ(1.0, 1.0, f.relative_tolerance, 0.0, true, f.default_rho_over_sigma) :
        f.default_rho_over_sigma
    return _ParticleSources(pfield, indices, relative_cutoff, nearfield)
end

function _regularization_cutoff(s::_ParticleSources, sigma, strength)
    f = s.pfield.fmm
    !f.autotune_reg_error && return f.default_rho_over_sigma
    strength < 10 * eps() && return zero(strength)
    if f.relative_tolerance <= eps(10.0)
        return vpm.solve_ρ_over_σ(sigma, strength, f.relative_tolerance,
            f.absolute_tolerance, true, f.default_rho_over_sigma)
    elseif f.absolute_tolerance <= eps(10.0)
        return s.relative_cutoff
    end
    # For positive inputs the absolute residual increases monotonically with
    # cutoff. A comfortably negative value at the relative root proves that
    # the absolute root is larger, so the existing minimum is already known.
    # Keep Brent for boundary cases, with a conservative cancellation margin.
    relative = s.relative_cutoff
    if sigma > 0 && isfinite(sigma) && isfinite(strength) &&
            isfinite(relative) && relative >= 0 && isfinite(f.absolute_tolerance)
        absolute_term = 4 * pi * sigma * sigma * f.absolute_tolerance * relative * relative / strength
        residual = vpm.residual_abs(relative, sigma, strength, f.absolute_tolerance)
        roundoff_margin = 64 * eps(Float64) * max(1.0, abs(absolute_term))
        residual < -roundoff_margin && return relative
    end
    absolute_cutoff = vpm.solve_ρ_over_σ(sigma, strength, 0.0,
        f.absolute_tolerance, true, f.default_rho_over_sigma)
    return min(s.relative_cutoff, absolute_cutoff)
end

function _particle_fmm.source_system_to_buffer!(buffer, ib, s::_ParticleSources, i)
    pfield, original = s.pfield, s.indices[i]
    sigma = pfield.particles[vpm.SIGMA_INDEX, original]
    gx, gy, gz = view(pfield.particles, vpm.GAMMA_INDEX, original)
    strength = sqrt(gx * gx + gy * gy + gz * gz)
    cutoff = _regularization_cutoff(s, sigma, strength)
    for j in 1:3
        buffer[j, ib] = pfield.particles[vpm.X_INDEX[j], original]
        buffer[j + 4, ib] = pfield.particles[vpm.GAMMA_INDEX[j], original]
    end
    buffer[4, ib] = cutoff * sigma
    buffer[8, ib] = sigma
    return nothing
end

_particle_fmm.body_to_multipole!(s::_ParticleSources, args...) =
    _particle_fmm.body_to_multipole!(_particle_fmm.Point{_particle_fmm.Vortex}, s, args...)

function _particle_fmm.direct!(target_buffer, target_index, switch::_particle_fmm.DerivativesSwitch,
        s::_ParticleSources, source_buffer, source_index)
    return _particle_fmm.direct!(target_buffer, target_index, switch,
        s.pfield, source_buffer, source_index)
end

function _particle_fmm.nearfield_multithread!(target_buffer, i_target_buffer, target_branches,
        source::_ParticleSources, source_buffer, i_source_system, source_branches,
        derivatives_switch, direct_list, interaction_list_method, n_threads)
    if source.nearfield !== :dynamic ||
            !(interaction_list_method isa _particle_fmm.SelfTuningTargetStop)
        return invoke(_particle_fmm.nearfield_multithread!,
            Tuple{Any,Any,Any,Any,Any,Any,Any,Any,Any,Any,Any},
            target_buffer, i_target_buffer, target_branches, source, source_buffer,
            i_source_system, source_branches, derivatives_switch, direct_list,
            interaction_list_method, n_threads)
    end
    isempty(direct_list) && return nothing
    # The pinned TargetStop list is sorted by target and uses only target leaves.
    # Those leaves have disjoint bodies. Each body chunk keeps its leaf's complete
    # source sequence so changing worker assignment preserves the summation order.
    groups = UnitRange{Int}[]
    source_counts = Int[]
    start = firstindex(direct_list)
    target = direct_list[start][1]
    sources = 0
    for i in eachindex(direct_list)
        it, is = direct_list[i]
        if it != target
            push!(groups, start:i-1)
            push!(source_counts, sources)
            start, target, sources = i, it, 0
        end
        sources += source_branches[is].n_bodies[i_source_system]
    end
    push!(groups, start:lastindex(direct_list))
    push!(source_counts, sources)
    total = sum(target_branches[direct_list[first(group)][1]].n_bodies[i_target_buffer] * count
                for (group, count) in zip(groups, source_counts))
    grain = max(1, div(total, 4 * n_threads))
    jobs = Tuple{UnitRange{Int},UnitRange{Int}}[]
    work = Int[]
    for (group, count) in zip(groups, source_counts)
        indices = target_branches[direct_list[first(group)][1]].bodies_index[i_target_buffer]
        chunk = max(1, div(grain, max(1, count)))
        for first_body in first(indices):chunk:last(indices)
            targets = Int(first_body):min(Int(last(indices)), Int(first_body) + chunk - 1)
            push!(jobs, (group, targets))
            push!(work, length(targets) * count)
        end
    end
    # A small probe query can have a single leaf. Subdivide its bodies rather
    # than its source sequence so it can use all workers without changing sums.
    # Start expensive jobs first to reduce the final partially idle wave.
    order = sortperm(work; rev=true)
    next = Threads.Atomic{Int}(0)
    Threads.@threads :static for worker in 1:min(n_threads, length(jobs))
        while true
            job = Threads.atomic_add!(next, 1) + 1
            job > length(jobs) && break
            group, targets = jobs[order[job]]
            for pair in group
                source_branch = source_branches[direct_list[pair][2]]
                _particle_fmm.direct!(target_buffer, targets, derivatives_switch,
                    source, source_buffer, source_branch.bodies_index[i_source_system])
            end
        end
    end
    return nothing
end

function _evaluate_targets!(target, source, uj; gradient=true, source_indices=1:source.np,
                            nearfield=:static)
    (_particle_fmm.get_n_bodies(target) == 0 || isempty(source_indices)) && return nothing
    sources = _particle_sources(source, source_indices; nearfield)
    if uj === vpm.UJ_direct
        _particle_fmm.direct!(target, sources; scalar_potential=false, hessian=gradient)
        return nothing
    elseif uj !== vpm.UJ_fmm
        throw(ArgumentError("Target evaluation supports UJ_fmm and UJ_direct"))
    end
    f = source.fmm
    targets, sources_tuple = (target,), (sources,)
    switches = _particle_fmm.DerivativesSwitch(false, true, gradient, targets)
    cache = _particle_fmm.Cache(targets, sources_tuple, switches)
    _particle_fmm.validate_cache_compatibility(cache, targets, sources_tuple, switches)
    tolerance = _particle_fmm.PowerRelativeGradient{f.relative_tolerance,f.absolute_tolerance,true}()
    _particle_fmm.warn_missing_previous_influence_metadata(targets, tolerance)
    TF = _particle_fmm.get_type(targets, sources_tuple)
    leaf_size = SVector(max(f.ncrit, f.min_ncrit))
    method = _particle_fmm.SelfTuningTargetStop()
    tree_options = (; expansion_order=f.p - 1, leaf_size,
        shrink=f.shrink_recenter, recenter=f.shrink_recenter, interaction_list_method=method)
    target_tree = _particle_fmm.Tree(targets, true, switches, TF;
        buffers=cache.target_buffers, small_buffers=cache.target_small_buffers, tree_options...)
    source_tree = _particle_fmm.Tree(sources_tuple, false, switches, TF;
        buffers=cache.source_buffers, small_buffers=cache.source_small_buffers, tree_options...)
    m2l_list, direct_list = _particle_fmm.build_interaction_lists(target_tree.branches,
        source_tree.branches, leaf_size, f.theta, true, true, true, method)
    m2l_list = _particle_fmm.sort_by_target(m2l_list, target_tree.branches)
    direct_list = _particle_fmm.sort_by_target(direct_list, target_tree.branches)
    # Retain identical trees and near-field neighborhoods. With no translations,
    # source expansions and zero local expansions cannot contribute to targets.
    farfield = !isempty(m2l_list)
    return _particle_fmm.fmm!(targets, target_tree, sources_tuple, source_tree,
        leaf_size, m2l_list, direct_list, switches, method;
        expansion_order=f.p - 1, multipole_acceptance=f.theta, error_tolerance=tolerance,
        upward_pass=farfield, horizontal_pass=farfield, downward_pass=farfield,
        nearfield_device=false,
        tune=false, silence_warnings=true)
end

struct _MPIInteraction{F} <: Function
    backend::ReplicatedMPI
    serial::F
end

function _collective_error(p::ReplicatedMPI, err)
    failed_rank = MPI.Allreduce(isnothing(err) ? p.nranks : p.rank, min, p.comm)
    if failed_rank < p.nranks
        message = MPI.bcast(p.rank == failed_rank ? sprint(showerror, err) : nothing,
                            p.comm; root=failed_rank)
        throw(ErrorException("MPI rank $failed_rank: $message"))
    end
    return nothing
end

function _replica_signature(pfield, check_state)
    signature = UInt64(pfield.np)
    if check_state
        for value in view(pfield.particles, :, 1:pfield.np)
            bits = eltype(pfield) === Float64 ? reinterpret(UInt64, value) : UInt64(reinterpret(UInt32, value))
            signature = (signature ⊻ bits) * UInt64(0x100000001b3)
        end
        for byte in codeunits(repr((pfield.fmm, pfield.transposed, typeof(pfield.kernel))))
            signature = (signature ⊻ UInt64(byte)) * UInt64(0x100000001b3)
        end
    end
    return signature
end

function _check_replicas(p::ReplicatedMPI, pfield)
    partition = p.partition === :targets ? UInt64(0) :
                p.partition === :sources ? UInt64(1) :
                p.partition === :hybrid ? UInt64(2) : typemax(UInt64)
    groups = p.target_groups > 0 ? UInt64(p.target_groups) : typemax(UInt64)
    source_order = p.source_order === :contiguous ? UInt64(0) :
                   p.source_order === :cyclic ? UInt64(1) : typemax(UInt64)
    signatures = reshape(MPI.Allgather(UInt64[
        _replica_signature(pfield, p.check_state), partition, groups, source_order], p.comm), 4, p.nranks)
    all(==(signatures[1, 1]), view(signatures, 1, :)) || throw(ArgumentError(
        "Particle replicas differ between ranks; initialize the same case on every rank"))
    all(rank -> signatures[2, rank] == signatures[2, 1] &&
                signatures[3, rank] == signatures[3, 1] &&
                signatures[4, rank] == signatures[4, 1], 1:p.nranks) ||
        throw(ArgumentError("MPI partition, target_groups or source_order differ between ranks"))
    partition <= 2 && groups != typemax(UInt64) && source_order <= 1 ||
        throw(ArgumentError("Invalid MPI partition, target_groups or source_order"))
    p.partition === :hybrid && p.nranks % p.target_groups != 0 &&
        throw(ArgumentError("target_groups must divide the communicator size"))
end

function _gather_targets(p::ReplicatedMPI, values::Matrix, n)
    rows = size(values, 1)
    counts = [rows * length(_owned_targets(n, r, p.nranks)) for r in 0:p.nranks-1]
    gathered = Matrix{eltype(values)}(undef, rows, n)
    started = time_ns()
    MPI.Allgatherv!(vec(values), MPI.VBuffer(vec(gathered), counts), p.comm)
    p.exchange_seconds += (time_ns() - started) * 1e-9
    return gathered
end

function _sum_sources(p::ReplicatedMPI, values::Matrix)
    started = time_ns()
    MPI.Allreduce!(MPI.IN_PLACE, values, +, p.comm)
    p.exchange_seconds += (time_ns() - started) * 1e-9
    return values
end

# Preserve the serial full-field SFS neighborhood. A tree built only from one
# rank's targets can otherwise change the modeled near-field SFS contributions.
function _sfs_targets(pfield, owned, serial, result, full_targets)
    local_particles = Matrix(pfield.particles[:, owned])
    local_particles[vpm.SFS_INDEX, :] .= zero(eltype(pfield))
    if serial === vpm.UJ_direct
        Threads.@threads for i in 1:length(owned)
            target = view(local_particles, :, i)
            vpm.is_static(target) && continue
            for source in vpm.iterator(pfield)
                r = sqrt(sum(abs2, vpm.get_X(source) - vpm.get_X(target)))
                vpm.Estr_direct(target, source, r, pfield.kernel.zeta, pfield.transposed)
            end
        end
    elseif !isempty(owned)
        f = pfield.fmm
        source_tree = result[4]
        switches = _particle_fmm.DerivativesSwitch(false, true, true, (full_targets,))
        method = _particle_fmm.SelfTuningTargetStop()
        target_tree = _particle_fmm.Tree((full_targets,), true, switches, eltype(pfield);
            expansion_order=f.p - 1, leaf_size=SVector(max(f.ncrit, f.min_ncrit)),
            shrink=f.shrink_recenter, recenter=f.shrink_recenter,
            interaction_list_method=method)
        _, pairs = _particle_fmm.build_interaction_lists(target_tree.branches,
            source_tree.branches, SVector(max(f.ncrit, f.min_ncrit)), f.theta,
            true, true, true, method)
        neighbors = [UnitRange{Int}[] for _ in owned]
        for (it, is) in pairs
            for sorted_target in target_tree.branches[it].bodies_index[1]
                original = target_tree.sort_index_list[1][sorted_target]
                if original in owned
                    push!(neighbors[original - first(owned) + 1], source_tree.branches[is].bodies_index[1])
                end
            end
        end
        Threads.@threads for i in 1:length(owned)
            target = view(local_particles, :, i)
            for range in neighbors[i], sorted_source in range
                source = vpm.get_particle(pfield, source_tree.sort_index_list[1][sorted_source])
                r = sqrt(sum(abs2, vpm.get_X(source) - vpm.get_X(target)))
                vpm.Estr_direct(target, source, r, pfield.kernel.zeta, pfield.transposed)
            end
        end
    end
    return Matrix(local_particles[vpm.SFS_INDEX, :])
end

function (uj::_MPIInteraction)(pfield; reset=true, reset_sfs=false, sfs=false,
        rbf=false, autotune=false, verbose=false, sfs_type=-1, transposed_sfs=true)
    p = uj.backend
    !p.active && return uj.serial(pfield; reset, reset_sfs, sfs, rbf)
    _check_replicas(p, pfield)
    pfield.useGPU == 0 || throw(ArgumentError("ReplicatedMPI is a CPU backend; useGPU must be zero"))
    uj.serial === vpm.UJ_fmm && eltype(pfield) !== Float64 && throw(ArgumentError(
        "The pinned FMM backend requires Float64; use UJ_direct for Float32"))
    rbf && throw(ArgumentError("MPI RBF/core-spreading evaluation is not implemented"))
    autotune && throw(ArgumentError("MPI uses fixed FMM settings; tune the serial baseline first"))
    sfs && p.partition in (:sources, :hybrid) && throw(ArgumentError("Source/hybrid partitioning does not support SFS; use partition=:targets"))
    reset && vpm._reset_particles(pfield)
    (reset_sfs || (sfs && uj.serial === vpm.UJ_fmm)) && vpm._reset_particles_sfs(pfield)
    source_partition = p.partition === :sources
    owned, source_indices = p.partition === :hybrid ? _hybrid_domains(p, pfield.np, pfield.np) :
        (source_partition ? (1:pfield.np) : _owned_targets(pfield.np, p.rank, p.nranks),
         source_partition ? _owned_sources(pfield.np, p.rank, p.nranks, p.source_order) : (1:pfield.np))
    target = _interaction_targets(pfield, owned)
    full_targets = sfs && uj.serial === vpm.UJ_fmm ?
        _interaction_targets(pfield, 1:pfield.np; outputs=0) : nothing
    err = nothing
    result = nothing
    started = time_ns()
    try
        result = _evaluate_targets!(target, pfield, uj.serial; source_indices, nearfield=p.nearfield)
    catch exception
        err = exception
    end
    p.evaluation_seconds += (time_ns() - started) * 1e-9
    _collective_error(p, err)
    values = p.partition === :hybrid ? _hybrid_values!(p, target.values, pfield.np) :
        source_partition ? _sum_sources(p, target.values) : _gather_targets(p, target.values, pfield.np)
    pfield.particles[vpm.U_INDEX, 1:pfield.np] .+= view(values, 1:3, :)
    pfield.particles[vpm.J_INDEX, 1:pfield.np] .+= view(values, 4:12, :)
    if sfs
        started = time_ns()
        local_sfs = zeros(eltype(pfield), 3, length(owned))
        err = nothing
        try
            local_sfs = _sfs_targets(pfield, owned, uj.serial, result, full_targets)
        catch exception
            err = exception
        end
        p.sfs_seconds += (time_ns() - started) * 1e-9
        _collective_error(p, err)
        pfield.particles[vpm.SFS_INDEX, 1:pfield.np] .+= _gather_targets(p, local_sfs, pfield.np)
    end
    p.evaluations += 1
    return nothing
end

function _parallel_interaction(::Nothing, uj, viscous)
    return uj
end

function _parallel_interaction(p::ReplicatedMPI, uj, viscous)
    _has_target_api || throw(ArgumentError("Use the pinned environment from scripts/setup_mpi.jl"))
    uj in (vpm.UJ_fmm, vpm.UJ_direct) || throw(ArgumentError("MPI supports UJ_fmm and UJ_direct"))
    vpm.isinviscid(viscous) || throw(ArgumentError("MPI currently supports Inviscid(); core spreading is not implemented"))
    return _MPIInteraction(p, uj)
end

function _target_velocities(pfield, Xs, uj)
    backend = uj isa _MPIInteraction ? uj.backend : nothing
    serial = uj isa _MPIInteraction ? uj.serial : uj
    collective = !isnothing(backend) && backend.active
    collective && _check_replicas(backend, pfield)
    if collective
        counts = MPI.Allgather(length(Xs), backend.comm)
        all(==(length(Xs)), counts) || throw(ArgumentError("Surface target counts differ between ranks"))
    end
    positions = Matrix{eltype(pfield)}(undef, 3, length(Xs))
    err = nothing
    try
        for i in eachindex(Xs)
            positions[:, i] .= Xs[i]
        end
    catch exception
        err = exception
    end
    if collective
        _collective_error(backend, err)
        if backend.check_state
            signature = UInt64(length(Xs))
            for value in positions
                bits = eltype(pfield) === Float64 ? reinterpret(UInt64, value) : UInt64(reinterpret(UInt32, value))
                signature = (signature ⊻ bits) * UInt64(0x100000001b3)
            end
            signatures = MPI.Allgather(signature, backend.comm)
            all(==(signature), signatures) || throw(ArgumentError("Surface target positions differ between ranks"))
        end
    elseif !isnothing(err)
        throw(err)
    end
    source_partition = collective && backend.partition === :sources
    hybrid_partition = collective && backend.partition === :hybrid
    owned, source_indices = hybrid_partition ? _hybrid_domains(backend, length(Xs), pfield.np) :
        (collective && !source_partition ? _owned_targets(length(Xs), backend.rank, backend.nranks) : 1:length(Xs),
         source_partition ? _owned_sources(pfield.np, backend.rank, backend.nranks, backend.source_order) : (1:pfield.np))
    target = _InteractionTargets(positions[:, owned], zeros(eltype(pfield), length(owned)),
                                  zeros(eltype(pfield), 3, length(owned)))
    err = nothing
    started = time_ns()
    try
        _evaluate_targets!(target, pfield, serial; gradient=false, source_indices,
                           nearfield=isnothing(backend) ? :static : backend.nearfield)
    catch exception
        err = exception
    end
    collective && (backend.probe_seconds += (time_ns() - started) * 1e-9)
    if collective
        _collective_error(backend, err)
    elseif !isnothing(err)
        throw(err)
    end
    values = hybrid_partition ? _hybrid_values!(backend, target.values, length(Xs)) :
        source_partition ? _sum_sources(backend, target.values) :
        collective ? _gather_targets(backend, target.values, length(Xs)) : target.values
    return [values[:, i] for i in eachindex(Xs)]
end

_runtime_noop(args...; kwargs...) = false
_parallel_callback(::Nothing, callback, sim, pfield, t, dt;
    sync_from=pfield.np+1, sync_omit_shedding=nothing, kwargs...) = callback(sim, pfield, t, dt; kwargs...)

# Broadcast data, not executable closures or process-local Python/native handles.
# Every rank already constructed the same models; restore into that local graph
# so vehicle aliases and captured monitor references remain intact.
struct _KeepLocal end
struct _ObjectState
    fields::Dict{Symbol,Any}
end

_callback_state(value) = _callback_state(value, IdDict{Any,Any}())

function _callback_state(value, seen::IdDict)
    if value isa Function || value isa Ptr || value isa Type ||
            nameof(parentmodule(typeof(value))) in (:PyCall, :Dierckx, :AirfoilPrep, :MPI)
        return _KeepLocal()
    elseif value isa AbstractArray
        return get!(seen, value) do
            eltype(value) <: Number ? copy(value) : map(item -> _callback_state(item, seen), value)
        end
    elseif value isa AbstractDict
        return get!(seen, value) do
            Dict(key => _callback_state(item, seen) for (key, item) in value)
        end
    elseif value isa Tuple
        return map(item -> _callback_state(item, seen), value)
    elseif isnothing(value) || value isa Number || value isa Symbol || value isa AbstractString || isbitstype(typeof(value))
        return value
    end
    # Vehicle systems share rotors, solution arrays and model data. Snapshot each
    # object once so serializing a callback does not duplicate those shared graphs.
    return get!(seen, value) do
        _ObjectState(Dict(name => _callback_state(getfield(value, name), seen)
                         for name in fieldnames(typeof(value))))
    end
end

_new_callback_state(value::_KeepLocal) = throw(ArgumentError("Callbacks cannot introduce new functions or native handles on workers"))
_new_callback_state(value::_ObjectState) = throw(ArgumentError("Callbacks cannot change vehicle/model topology"))
_new_callback_state(value::AbstractArray) = map(_new_callback_state, value)
_new_callback_state(value::AbstractDict) = Dict(k => _new_callback_state(v) for (k, v) in value)
_new_callback_state(value::Tuple) = map(_new_callback_state, value)
_new_callback_state(value) = value

_same_callback_state(::_KeepLocal, ::_KeepLocal) = true
_same_callback_state(a::_ObjectState, b::_ObjectState) = _same_callback_state(a.fields, b.fields)
function _same_callback_state(a::AbstractArray, b::AbstractArray)
    axes(a) == axes(b) || return false
    return all(_same_callback_state(a[i], b[i]) for i in eachindex(a, b))
end
function _same_callback_state(a::AbstractDict, b::AbstractDict)
    length(a) == length(b) || return false
    return all(haskey(b, key) && _same_callback_state(value, b[key]) for (key, value) in a)
end
function _same_callback_state(a::Tuple, b::Tuple)
    length(a) == length(b) || return false
    return all(_same_callback_state(a[i], b[i]) for i in eachindex(a))
end
_same_callback_state(a, b) = isequal(a, b)

function _readonly_callback_state(sim, pfield)
    schemes = (pfield.formulation, pfield.viscous, pfield.kernel, pfield.SFS,
               pfield.integration, pfield.transposed, pfield.relaxation, pfield.fmm,
               pfield.maxparticles, pfield.useGPU)
    return _callback_state((sim, (pfield.np, pfield.t, pfield.nt),
                            view(pfield.particles, :, 1:pfield.np), schemes))
end

function _restore_callback_state(local_value, state)
    if state isa _KeepLocal
        return local_value
    elseif state isa _ObjectState
        names = fieldnames(typeof(local_value))
        values = map(names) do name
            _restore_callback_state(getfield(local_value, name), state.fields[name])
        end
        if ismutabletype(typeof(local_value))
            for (name, value) in zip(names, values)
                setfield!(local_value, name, value)
            end
            return local_value
        elseif all(isequal(getfield(local_value, name), value) for (name, value) in zip(names, values))
            return local_value
        elseif applicable(typeof(local_value), values...)
            return typeof(local_value)(values...)
        end
        throw(ArgumentError("Callback reconfiguration of $(typeof(local_value)) is not supported"))
    elseif state isa AbstractDict
        for key in collect(keys(local_value))
            haskey(state, key) || delete!(local_value, key)
        end
        for (key, value) in state
            local_value[key] = haskey(local_value, key) ?
                _restore_callback_state(local_value[key], value) : _new_callback_state(value)
        end
        return local_value
    elseif state isa AbstractArray
        if size(local_value) != size(state)
            return _new_callback_state(state)
        end
        if eltype(local_value) <: Number && eltype(state) <: Number
            copyto!(local_value, state)
            return local_value
        end
        for i in eachindex(local_value, state)
            local_value[i] = _restore_callback_state(local_value[i], state[i])
        end
        return local_value
    elseif state isa Tuple
        return map(_restore_callback_state, local_value, state)
    end
    return state
end

function _parallel_callback_readonly(p::ReplicatedMPI, callback, sim, pfield, t, dt;
                                    sync_omit_shedding=nothing, kwargs...)
    started = time_ns()
    try
        payload, err = nothing, nothing
        if _parallel_root(p)
            previous_active = p.active
            p.active = false # additional callback queries remain serial
            try
                noop = callback === _runtime_noop
                before = p.check_state && !noop ? _readonly_callback_state(sim, pfield) : nothing
                stop = noop ? false : callback(sim, pfield, t, dt; kwargs...)
                stop isa Bool || throw(ArgumentError("Runtime callback must return a Bool"))
                if p.check_state && !noop &&
                        !_same_callback_state(before, _readonly_callback_state(sim, pfield))
                    throw(ArgumentError("callback_sync=:readonly callback modified simulation, particles or schemes; use :full for mutations"))
                end
                payload = (stop, isnothing(sync_omit_shedding) ? nothing : copy(sync_omit_shedding))
            catch exception
                err = exception
            finally
                p.active = previous_active
            end
        end
        _collective_error(p, err)
        stop, new_omit_shedding = MPI.bcast(payload, p.comm; root=p.root)
        err = nothing
        try
            if !_parallel_root(p) && !isnothing(new_omit_shedding)
                resize!(sync_omit_shedding, length(new_omit_shedding))
                copyto!(sync_omit_shedding, new_omit_shedding)
            end
        catch exception
            err = exception
        end
        _collective_error(p, err)
        # The fast mode relies on each rank solving and shedding independently.
        # Validate that assumption after the root's external shedding controls
        # have synchronized, including when the callback requests termination.
        p.check_state && _check_replicas(p, pfield)
        return stop
    finally
        p.callback_seconds += (time_ns() - started) * 1e-9
    end
end

function _parallel_callback(p::ReplicatedMPI, callback, sim, pfield, t, dt;
                            sync_from=pfield.np+1, sync_omit_shedding=nothing, kwargs...)
    p.callback_sync === :readonly && return _parallel_callback_readonly(p, callback, sim, pfield, t, dt;
        sync_omit_shedding, kwargs...)
    started = time_ns()
    noop = callback === _runtime_noop
    if p.nranks == 1
        previous_active = p.active
        p.active = false
        try
            stop = noop ? false : callback(sim, pfield, t, dt; kwargs...)
            stop isa Bool || throw(ArgumentError("Runtime callback must return a Bool"))
            return stop
        catch exception
            throw(ErrorException("MPI rank $(p.rank): $(sprint(showerror, exception))"))
        finally
            p.active = previous_active
            p.callback_seconds += (time_ns() - started) * 1e-9
        end
    end
    before = _parallel_root(p) && !noop ? copy(view(pfield.particles, :, 1:pfield.np)) : nothing
    payload = nothing
    err = nothing
    if _parallel_root(p)
        p.active = false # callbacks may make additional serial interaction queries
        try
            stop = noop ? false : callback(sim, pfield, t, dt; kwargs...)
            stop isa Bool || throw(ArgumentError("Runtime callback must return a Bool"))
            # Root is authoritative for new wake shedding and coupled state,
            # including when no monitor/runtime callback was requested.
            changed = collect(sync_from:pfield.np)
            if !noop
                for i in 1:max(pfield.np, size(before, 2))
                    if i > size(before, 2) || !isequal(view(before, :, i), view(pfield.particles, :, i))
                        push!(changed, i)
                    end
                end
            end
            unique!(changed)
            payload = (stop, pfield.np, pfield.t, pfield.nt, changed,
                       pfield.particles[:, changed], _callback_state(sim),
                       _callback_state((pfield.formulation, pfield.viscous, pfield.kernel, pfield.SFS,
                        pfield.integration, pfield.transposed, pfield.relaxation,
                        pfield.fmm)), isnothing(sync_omit_shedding) ? nothing : copy(sync_omit_shedding))
        catch exception
            err = exception
        finally
            p.active = true
        end
    end
    _collective_error(p, err)
    stop, np, time, nt, changed, particles, new_sim, schemes, new_omit_shedding = MPI.bcast(payload, p.comm; root=p.root)
    err = nothing
    try
        if !_parallel_root(p)
            pfield.particles[:, changed] .= particles
            pfield.np, pfield.t, pfield.nt = np, time, nt
            names = (:formulation, :viscous, :kernel, :SFS, :integration, :transposed,
                     :relaxation, :fmm)
            for (name, state) in zip(names, schemes)
                setfield!(pfield, name, _restore_callback_state(getfield(pfield, name), state))
            end
            _restore_callback_state(sim, new_sim)
            # The standard hover callback captures this driver option outside
            # `sim`. Preserve its identity while propagating mask edits.
            if !isnothing(new_omit_shedding)
                resize!(sync_omit_shedding, length(new_omit_shedding))
                copyto!(sync_omit_shedding, new_omit_shedding)
            end
        end
    catch exception
        err = exception
    end
    _collective_error(p, err)
    p.callback_seconds += (time_ns() - started) * 1e-9
    return stop
end

"""
    parallel_statistics(backend)

Collectively report maximum-rank accumulated interaction, exchange and SFS times.
Call on every rank after the simulation. Timings exclude compilation warmup only
if the caller constructs a fresh backend after warming up.
"""
function parallel_statistics(p::ReplicatedMPI)
    p.active || throw(ArgumentError("Call parallel_statistics on every rank outside runtime callbacks"))
    times = MPI.Allreduce([p.evaluation_seconds, p.exchange_seconds, p.sfs_seconds,
                           p.probe_seconds, p.callback_seconds, p.rotor_seconds], max, p.comm)
    return (ranks=p.nranks, evaluations=p.evaluations,
            evaluation_seconds=times[1], exchange_seconds=times[2], sfs_seconds=times[3],
            probe_seconds=times[4], callback_seconds=times[5], rotor_seconds=times[6])
end
