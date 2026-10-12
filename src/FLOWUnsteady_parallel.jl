# Replicated sources, disjoint interaction targets. No MPI operation is performed
# until the caller explicitly initializes MPI and requests this backend.
const _particle_fmm = vpm.fmm

"""
    ReplicatedMPI(comm=MPI.COMM_WORLD; root=0, check_state=false)

MPI backend for accelerating one simulation. Initialize MPI before construction
and call `run_simulation(...; parallel=backend)` on every rank. Every rank retains
the full particle field. Only `root` executes callbacks and writes output.
`check_state=true` additionally checks particle replicas before each evaluation.
"""
mutable struct ReplicatedMPI
    comm::MPI.Comm
    rank::Int
    nranks::Int
    root::Int
    check_state::Bool
    active::Bool
    evaluation_seconds::Float64
    exchange_seconds::Float64
    sfs_seconds::Float64
    probe_seconds::Float64
    callback_seconds::Float64
    evaluations::Int
end

function ReplicatedMPI(comm=MPI.COMM_WORLD; root=0, check_state=false)
    MPI.Initialized() || throw(ArgumentError("Call MPI.Init() before constructing ReplicatedMPI"))
    MPI.Finalized() && throw(ArgumentError("MPI has already been finalized"))
    nranks = MPI.Comm_size(comm)
    0 <= root < nranks || throw(ArgumentError("root must be a rank in the communicator"))
    return ReplicatedMPI(comm, MPI.Comm_rank(comm), nranks, root, check_state,
                         true, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
end

_parallel_root(::Nothing) = true
_parallel_root(p::ReplicatedMPI) = p.rank == p.root
_owned_targets(n, rank, nranks) = (div(n * rank, nranks) + 1):div(n * (rank + 1), nranks)

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

function _evaluate_targets!(target, source, uj; gradient=true)
    (_particle_fmm.get_n_bodies(target) == 0 || source.np == 0) && return nothing
    if uj === vpm.UJ_direct
        _particle_fmm.direct!(target, source; scalar_potential=false, hessian=gradient)
        return nothing
    elseif uj !== vpm.UJ_fmm
        throw(ArgumentError("Target evaluation supports UJ_fmm and UJ_direct"))
    end
    f = source.fmm
    return _particle_fmm.fmm!((target,), (source,);
        expansion_order=f.p - 1, leaf_size_source=max(f.ncrit, f.min_ncrit),
        multipole_acceptance=f.theta,
        error_tolerance=_particle_fmm.PowerRelativeGradient{f.relative_tolerance,f.absolute_tolerance,true}(),
        shrink=f.shrink_recenter, recenter=f.shrink_recenter,
        scalar_potential=false, hessian=gradient, nearfield_device=false,
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
    signatures = MPI.Allgather(_replica_signature(pfield, p.check_state), p.comm)
    all(==(first(signatures)), signatures) || throw(ArgumentError(
        "Particle replicas differ between ranks; initialize the same case on every rank"))
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
    reset && vpm._reset_particles(pfield)
    (reset_sfs || (sfs && uj.serial === vpm.UJ_fmm)) && vpm._reset_particles_sfs(pfield)
    owned = _owned_targets(pfield.np, p.rank, p.nranks)
    target = _interaction_targets(pfield, owned)
    full_targets = sfs && uj.serial === vpm.UJ_fmm ?
        _interaction_targets(pfield, 1:pfield.np; outputs=0) : nothing
    err = nothing
    result = nothing
    started = time_ns()
    try
        result = _evaluate_targets!(target, pfield, uj.serial)
    catch exception
        err = exception
    end
    p.evaluation_seconds += (time_ns() - started) * 1e-9
    _collective_error(p, err)
    values = _gather_targets(p, target.values, pfield.np)
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
    owned = collective ? _owned_targets(length(Xs), backend.rank, backend.nranks) : 1:length(Xs)
    target = _InteractionTargets(positions[:, owned], zeros(eltype(pfield), length(owned)),
                                  zeros(eltype(pfield), 3, length(owned)))
    err = nothing
    started = time_ns()
    try
        _evaluate_targets!(target, pfield, serial; gradient=false)
    catch exception
        err = exception
    end
    collective && (backend.probe_seconds += (time_ns() - started) * 1e-9)
    if collective
        _collective_error(backend, err)
    elseif !isnothing(err)
        throw(err)
    end
    values = collective ? _gather_targets(backend, target.values, length(Xs)) : target.values
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

function _callback_state(value)
    if value isa Function || value isa Ptr || value isa Type ||
            nameof(parentmodule(typeof(value))) in (:PyCall, :Dierckx, :AirfoilPrep, :MPI)
        return _KeepLocal()
    elseif value isa AbstractArray
        return eltype(value) <: Number ? copy(value) : map(_callback_state, value)
    elseif value isa AbstractDict
        return Dict(key => _callback_state(item) for (key, item) in value)
    elseif value isa Tuple
        return map(_callback_state, value)
    elseif isnothing(value) || value isa Number || value isa Symbol || value isa AbstractString || isbitstype(typeof(value))
        return value
    end
    return _ObjectState(Dict(name => _callback_state(getfield(value, name))
                             for name in fieldnames(typeof(value))))
end

_new_callback_state(value::_KeepLocal) = throw(ArgumentError("Callbacks cannot introduce new functions or native handles on workers"))
_new_callback_state(value::_ObjectState) = throw(ArgumentError("Callbacks cannot change vehicle/model topology"))
_new_callback_state(value::AbstractArray) = map(_new_callback_state, value)
_new_callback_state(value::AbstractDict) = Dict(k => _new_callback_state(v) for (k, v) in value)
_new_callback_state(value::Tuple) = map(_new_callback_state, value)
_new_callback_state(value) = value

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
        for i in eachindex(local_value, state)
            local_value[i] = _restore_callback_state(local_value[i], state[i])
        end
        return local_value
    elseif state isa Tuple
        return map(_restore_callback_state, local_value, state)
    end
    return state
end

function _parallel_callback(p::ReplicatedMPI, callback, sim, pfield, t, dt;
                            sync_from=pfield.np+1, sync_omit_shedding=nothing, kwargs...)
    started = time_ns()
    noop = callback === _runtime_noop
    before = _parallel_root(p) && !noop ? copy(pfield.particles[:, 1:pfield.np]) : nothing
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
                           p.probe_seconds, p.callback_seconds], max, p.comm)
    return (ranks=p.nranks, evaluations=p.evaluations,
            evaluation_seconds=times[1], exchange_seconds=times[2], sfs_seconds=times[3],
            probe_seconds=times[4], callback_seconds=times[5])
end
