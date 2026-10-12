# Use the pinned converter's unchanged polar transformations on selected original
# element indices. Geometry remains complete, including the AR extrapolation spline.
struct _IndexedRotorPolars{P}
    polars::P
    indices::UnitRange{Int}
end
Base.size(p::_IndexedRotorPolars) = (length(p.indices),)
Base.enumerate(p::_IndexedRotorPolars) = ((i, p.polars[i]) for i in p.indices)

struct _PolarRotorProxy{R,P}
    rotor::R
    polars::P
end
function Base.getproperty(p::_PolarRotorProxy, name::Symbol)
    name === :_polars && return getfield(p, :polars)
    return getproperty(getfield(p, :rotor), name)
end

_rotor_spline_record(spline) = (; t=copy(spline.t), c=copy(spline.c),
    k=spline.k, bc=spline.bc, fp=spline.fp)
_rotor_spline_from_record(record) = vlm.Dierckx.Spline1D(copy(record.t),
    copy(record.c), record.k, record.bc, record.fp)

function _parallel_rotor_polars(p::ReplicatedMPI, rotor, RPM;
        sound_spd=nothing, AR_to_360extrap=true,
        apply_correction3D=true, apply_extrapolate=true)
    n = length(rotor._polars)
    owned = _owned_targets(rotor.B * n, p.rank, p.nranks)
    local_records = NamedTuple[]
    err = nothing
    try
        for blade in 1:rotor.B
            offset = (blade - 1) * n
            first_element = max(first(owned) - offset, 1)
            last_element = min(last(owned) - offset, n)
            first_element > last_element && continue
            indices = first_element:last_element
            proxy = _PolarRotorProxy(rotor, _IndexedRotorPolars(rotor._polars, indices))
            converted = vlm.FLOWVLM2OCCBlade(proxy, RPM, blade, rotor.turbine_flag;
                sound_spd, AR_to_360extrap, apply_correction3D, apply_extrapolate)
            for (element, af) in zip(indices, converted.af)
                push!(local_records, (; blade, element,
                    cl=_rotor_spline_record(af.cl), cd=_rotor_spline_record(af.cd)))
            end
        end
    catch exception
        err = exception
    end
    _collective_error(p, err)
    parts = MPI.gather(local_records, p.comm; root=p.root)
    records = MPI.bcast(_parallel_root(p) ? reduce(vcat, parts) : nothing, p.comm; root=p.root)
    airfoils = [Vector{vlm.OCCBAirfoilData}(undef, n) for _ in 1:rotor.B]
    err = nothing
    try
        length(records) == rotor.B * n || error("Distributed rotor polar count differs")
        assigned = falses(n, rotor.B)
        for record in records
            assigned[record.element, record.blade] && error("Duplicate distributed rotor polar")
            airfoils[record.blade][record.element] = vlm.OCCBAirfoilData(
                _rotor_spline_from_record(record.cl), _rotor_spline_from_record(record.cd))
            assigned[record.element, record.blade] = true
        end
        all(assigned) || error("Missing distributed rotor polar")
    catch exception
        err = exception
    end
    _collective_error(p, err)
    return [vlm.OCCBRotor(rotor._r, rotor._chord,
        (-1)^rotor.turbine_flag * (-1)^rotor.CW * rotor._theta * pi / 180,
        airfoils[blade], rotor.hubR, rotor.rotorR, rotor.B, 0.0) for blade in 1:rotor.B]
end

_parallel_rotor_backend(pfield) = pfield.UJ isa _MPIInteraction ? pfield.UJ.backend : nothing

function _parallel_solvefromV(pfield, rotor, Vind, args...; debug=false, kwargs...)
    p = _parallel_rotor_backend(pfield)
    if isnothing(p) || !p.active
        return vlm.solvefromV(rotor, Vind, args...; debug, kwargs...)
    end
    supported = (:t, :sound_spd, :hubtiploss_correction, :AR_to_360extrap,
        :apply_correction3D, :apply_extrapolate, :verbosewarn)
    # A supplied sound speed makes the original Mach step construct a transient
    # polar before further Python methods. Keep the original solver otherwise,
    # preserving every stored base-polar handle and its scratch attributes.
    no_transient_polar = isnothing(get(kwargs, :sound_spd, nothing))
    started = time_ns()
    try
        distributed = p.rotor_parallel && p.nranks > 1 && !debug &&
            !no_transient_polar && all(key -> key in supported, keys(kwargs))
        if p.rotor_parallel && p.nranks > 1
            choices = MPI.Allgather(distributed, p.comm)
            all(==(first(choices)), choices) || throw(ArgumentError(
                "Rotor solver options select different execution paths between ranks"))
        end
        if !distributed
            return vlm.solvefromV(rotor, Vind, args...; debug, kwargs...)
        end
        return _parallel_solvefromV_impl(p, rotor, Vind, args...; kwargs...)
    finally
        p.rotor_seconds += (time_ns() - started) * 1e-9
    end
end

# This is the standard non-debug lookup orchestration from pinned FLOWVLM
# f7c43725. Keep force, circulation, correction and coordinate math in its
# original functions; only the independent polar preparation is distributed.
function _parallel_solvefromV_impl(p::ReplicatedMPI, rotor::vlm.Rotor{TF}, Vind,
        Vinf, RPM, rho; t=0.0, sound_spd=nothing,
        hubtiploss_correction=vlm.hubtiploss_nocorrection, AR_to_360extrap=true,
        apply_correction3D=true, apply_extrapolate=true, verbosewarn=true) where TF
    err = nothing
    try
        size(Vind, 1) == rotor.B || error("Expected $(rotor.B) Vind entries; got $(size(Vind, 1)).")
        for blade in 1:rotor.B
            size(Vind[blade], 1) == vlm.get_mBlade(rotor) || error("Unexpected rotor blade inflow count")
        end
        vlm.setVinf(rotor, Vinf)
        vlm.setRPM(rotor, RPM)
        vlm.getHorseshoe(rotor, 1)
        if isnothing(sound_spd) && verbosewarn
            @warn "No sound speed has been provided. No Mach corrections will be applied."
        end
        vlm.calc_inflow(rotor, Vinf, RPM; t, Vinds=Vind)
    catch exception
        err = exception
    end
    _collective_error(p, err)
    converted = _parallel_rotor_polars(p, rotor, RPM;
        sound_spd, AR_to_360extrap, apply_correction3D, apply_extrapolate)
    TF_promoted = promote_type(TF, eltype(Vinf), typeof(RPM), typeof(rho))
    data = Array{Vector{TF_promoted}}[]
    data_Np, data_Tp, data_cd, data_roR = (Vector{TF_promoted}[] for _ in 1:4)
    gammas, mus_drag = [], []
    err = nothing
    try
        for blade in 1:rotor.B
            inflow = rotor.sol["CCBInflow"]["field_data"][blade]
            inflow_x, inflow_y = [V[1] for V in inflow], [V[2] for V in inflow]
            inflow_x = (-1)^(!rotor.turbine_flag) * inflow_x
            occbinflow = vlm.OCCBInflow(inflow_x, inflow_y, rho)
            Np, Tp, _, _, gamma, _, _, _, cd, _, mu_drag =
                vlm._calc_distributedloads_lookuptable(converted[blade], occbinflow,
                    rotor.turbine_flag; hubtiploss_correction)
            ccb_Fs = [[Np[i], Tp[i], 0.0] for i in 1:vlm.get_mBlade(rotor)]
            Fs = [vlm._ccblade2global(vlm.get_blade(rotor, blade), force, rotor.CW;
                translate=false) for force in ccb_Fs]
            push!(data, Fs)
            push!(data_Np, Np)
            push!(data_Tp, Tp)
            push!(data_cd, cd)
            push!(data_roR, rotor._r / rotor.rotorR)
            push!(gammas, gamma)
            push!(mus_drag, mu_drag)
        end
        for (name, kind, values) in (("DistributedLoad", "vector", data),
                ("Np", "scalar", data_Np), ("Tp", "scalar", data_Tp),
                ("cd", "scalar", data_cd), ("roR", "scalar", data_roR))
            rotor.sol[name] = Dict("field_name" => name, "field_type" => kind, "field_data" => values)
        end
        gamma, mu_drag = vlm.calc_aerodynamicforces(rotor, rho;
            overwritegammas=gammas, overwritemus=mus_drag)
        TF_solution = promote_type(eltype(Vinf), typeof(RPM), typeof(rho), typeof(t))
        new_gamma, new_mu = TF_solution[], TF_solution[]
        new_Ftot, new_L, new_D, new_S = (Vector{TF_solution}[] for _ in 1:4)
        for blade in 1:rotor.B, element in 1:vlm.get_mBlade(rotor)
            push!(new_gamma, gamma[blade][element])
            push!(new_mu, mu_drag[blade][element])
            push!(new_Ftot, rotor.sol["DistributedLoad"]["field_data"][blade][element])
            push!(new_L, rotor.sol["Lift"]["field_data"][blade][element])
            push!(new_D, rotor.sol["Drag"]["field_data"][blade][element])
            push!(new_S, rotor.sol["RadialForce"]["field_data"][blade][element])
        end
        for (name, values) in (("Gamma", new_gamma), ("mu", new_mu),
                ("Ftot", new_Ftot), ("L", new_L), ("D", new_D), ("S", new_S))
            vlm._addsolution(rotor._wingsystem, name, values; t)
        end
    catch exception
        err = exception
    end
    _collective_error(p, err)
    return nothing
end
