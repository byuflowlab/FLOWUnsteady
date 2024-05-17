#------- abstract types -------#

"""
    <: AbstractModel{TF,Quasisteady::Bool}

Model type definition for float type `TF`. If `Quasisteady==true`, the solution of the model is independent of its time history. For every `<:AbstractModel` type, the following functions should be overloaded:

* `solve!(model::AbstractModel, dt)`: solves the lower-level model used in the simulation

"""
abstract type AbstractModel{TF,Quasisteady} end

function kinematic_velocity!(model::AbstractModel, map, velocity, angular_velocity, center_of_rotation)
    @error "kinematic_velocity! not defined for $(typeof(model))"
end

function apply!(model::AbstractModel, freestream)
    @error "apply! not defined for $(typeof(model))"
end

"""
    force!(model, map, center)

Calculates the force and moment about `center` applied to the portion of `model` designated by `map`. Also updates `model` with the calculated force if applicable.

# Arguments

* `model::AbstractModel`: the model from which the force is computed
* `map`: the portion of `model` on which the force is applied
* `center::SVector{3,Float64}`: the point about which moments are taken

# Returns

* `force::SVector{3,Float64}`: the force applied
* `moment::SVector{3,Float64}`: the moment applied about `center`

"""
function force!(model::AbstractModel, map, center)
    @error "force! not defined for $(typeof(model))"
end

function transform!(model::AbstractModel, map, translation, rotation, center_of_rotation)
    @error "transform! not defined for $(typeof(model))"
end

function map_all(model::AbstractModel)
    @error "map_all not defined for $(typeof(model))"
end

"""
    solve!(model, dt)

Solves the `model` in-place for use across a timestep of `dt`.

# Arguments

* `model::AbstractModel`: the model to be solved
* `dt::Number`: the timestep duration of this solve (useful for determining wake shedding locations)

"""
function solve!(model::AbstractModel, dt)
    @error "solve! not yet implemented for models of type $(typeof(model))"
end

function initialize_history(model::AbstractModel, save_steps)
    @error "initialize_history not defined for $(typeof(model))"
end

"""
    <: AbstractBoundaryElementModel{TF,Quasisteady}

Subtype of `<:AbstractModel` for float type `TF`. Provides a convenient framework for defining the `solve!` function for any specific instance of `AbstractBoundaryElementModel`. Instead of writing the entire `solve!` function, the user need only define the following smaller functions:

* `all, wake, fmm_options = prepare_wake_on_all!(model::AbstractModel, dt)`: `all` and `wake` are target and source, respectively, in a call to `FastMultipole.fmm!(all, wake; fmm_options...)`
* `solve_boundary!(model::AbstractModel, dt)`: in-place function solves all boundary elements in the `model`
* `all, surface, fmm_options = prepare_surface_on_all!(model::AbstractModel, dt)`: `all` and `surface` are target and source, respectively, in a call to `FastMultipole.fmm!(all, surface; fmm_options...)`
* `shed_wake!(model::AbstractModel)`: in-place function sheds wake elements
* `forces!(model::AbstractModel)`: in-place function calculates forces based on the solved `model`

"""
abstract type AbstractBoundaryElementModel{TF,Quasisteady} <: AbstractModel{TF,Quasisteady} end

"""
    n_wake_elements(model)

Returns the number of active wake elements in `model`. This is used to avoid calling the FMM for wake-on-all influence if there are no active wake elements.

# Arguments

* `model::AbstractBoundaryElementModel`: the model whose wake elements are to be counted

# Returns

* `n::Int`: the number of active wake elements in `model`

"""
function n_wake_elements(model::AbstractBoundaryElementModel)
    @error "n_wake_elements not defined for $(typeof(model))"
end

#--- fmm setup ---#

requires_scalar_potential(system) = false
requires_vector_potential(system) = false
requires_velocity(system) = true
requires_velocity_gradient(system) = false
requires_velocity_gradient(::vpm.ParticleField) = true
requires_velocity_gradient(::fmm.ProbeSystem{<:Any,<:Any,<:Any,<:Any,true}) = true

requires_potential(system) = requires_scalar_potential(system), requires_vector_potential(system), requires_velocity(system), requires_velocity_gradient(system)
requires_potential(systems::Tuple) =
    [requires_scalar_potential(system) for system in systems],
    [requires_vector_potential(system) for system in systems],
    [requires_velocity(system) for system in systems],
    [requires_velocity_gradient(system) for system in systems]

#--- solve! ---#

"""
    solve!(model::AbstractBoundaryElementModel, dt)

Solves an `<:AbstractBoundaryElementModel` object in-place over a timestep of `dt`.

# Arguments

* `model::AbstractBoundaryElementModel`: the model to be solved
* `dt::Number`: the timestep over which the model is to be solved (primarily used to determine wake shedding locations)

"""
function solve!(model::AbstractBoundaryElementModel{<:Any,false}, dt)

    #--- wake-on-all induced velocity ---#
    DEBUG[] && println("pre-prepare-wake-on-all")
    all, wake, fmm_options = prepare_wake_on_all!(model, dt)


    scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)
    DEBUG[] && println("pre-wake-on-all")
    n_wake_elements(model) > 0 && fmm.fmm!(all, wake; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    #--- solve boundary elements ---#
    DEBUG[] && println("pre-solve-boundary")
    solve_boundary!(model, dt)

    #--- surface-on-all induced velocity ---#
    DEBUG[] && println("pre-prep-surface-on-all")
    all, surface, fmm_options = prepare_surface_on_all!(model)

    scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)
    DEBUG[] && println("pre-surface-on-all")
    fmm.fmm!(all, surface; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    #--- shed wake for this timestep ---#
    DEBUG[] && println("pre-shedwake!")
    shed_wake!(model, dt)

    #--- compute forces ---#
    DEBUG[] && println("pre forces!")
    forces!(model)
    DEBUG[] && println("post forces!")

    return nothing
end

#------- Vortex Lattice Model -------#

struct VortexLatticeModel{TF,TW,Quasisteady} <: AbstractBoundaryElementModel{TF,Quasisteady}
    # vlm
    vlm_system::vlm.System{TF}
    new_particle_probes::fmm.ProbeSystem{TF, Nothing, Nothing, Vector{SVector{3,TF}}, Vector{SMatrix{3,3,TF,9}}}
    surface_forces::Vector{SVector{3,TF}}
    surface_moments::Vector{SVector{3,TF}}

    # wake
    wake::TW
    toggle_wake::Vector{Bool}
    new_particle_overlap_trailing::Vector{TF}
    new_particle_overlap_unsteady::Vector{TF}
    new_particle_p_per_step_trailing::Vector{Int}
    new_particle_p_per_step_unsteady::Vector{Int}
    eta_wake::TF # shed the next wake filament V * dt * eta_wake[i_surface]

    # fmm parameters
    expansion_order::Int
    n_per_branch::Int
    multipole_acceptance_criterion_surface::Float64
    multipole_acceptance_criterion_wake::Float64
end

#--- constructor ---#

function VortexLatticeModel(system::vlm.System{TF}, quasisteady=false;
        # dynamics options
        center_of_mass = nothing, # if nothing, use system.reference.r

        # wake shedding options
        toggle_wake = [true for _ in 1:length(system.surfaces)], # indicates whether each surface should shed a wake
    	new_particle_overlap_trailing = [0.3 for _ in 1:length(system.surfaces)], # overlap determines newly shed particle smoothing radius
    	new_particle_overlap_unsteady = [0.3 for _ in 1:length(system.surfaces)], # overlap determines newly shed particle smoothing radius
    	new_particle_p_per_step_trailing = [1 for _ in 1:length(system.surfaces)], # how many particles to shed in the freestream direction each timestep
    	new_particle_p_per_step_unsteady = [1 for _ in 1:length(system.surfaces)], # how many particles to shed in the spanwise direction each timestep
    	eta_wake = 0.3, # distance from the trailing edge to place new particles in terms of the local velocity induced at the trailing edge during the last timestep

        # wake object
        wake = nothing, # if nothing, a vpm.ParticleField is generated using the following options
        max_timesteps = 0, # used to determine how many particles to preallocate

        # vpm options
        vpm_options = (), # keyword arguments supplied to the `vpm.ParticleField` constructor
    	fmm_expansion_order = 4, # expansion order of multipole expansions
    	fmm_n_per_branch = 50, # number of particles per multipole expansion
    	fmm_multipole_acceptance_criterion_surface=0.4, # number between 0-1 serves as the accuracy-performance tradeoff dial for evaluating expansions on surfaces
    	fmm_multipole_acceptance_criterion_wake=0.4, # number between 0-1 serves as the accuracy-performance tradeoff dial for evaluating expansions on wakes
    ) where TF

    # assertions
    for trailing_vortex in system.trailing_vortices
        @assert trailing_vortex == quasisteady "semi-infinite vortex horseshoes " * (quasisteady ? "required" : "not supported") * " for `VortexLatticeModel{<:Any,<:Any,$(quasisteady)}`; set system.trailing_vortices .= $(quasisteady)"
    end

    # update fmm_probes
    surfaces = system.surfaces
    fmm_velocity_probes = system.fmm_velocity_probes
    n_control_points, n_surface_filaments, n_new_particles, n_trailing_edge_corners = count_probes(surfaces)
    vlm.update_n_probes!(fmm_velocity_probes, n_control_points + n_surface_filaments + n_trailing_edge_corners)
    vlm.update_probes!(fmm_velocity_probes, surfaces, 0) # control points and filament centers
    update_probes_te!(fmm_velocity_probes, surfaces, n_control_points + n_surface_filaments) # trailing edge corners

    # new particle probes
    n_new_particles = 0
    for surface in system.surfaces
        ns = size(surface,2)
        n_new_particles += ns # unsteady loading
        n_new_particles += ns + 1 # trailing vortex
    end
    new_particle_probes = fmm.ProbeSystem(n_new_particles; velocity=true, velocity_gradient=true)

    # surface forces/moments
    surface_forces = Vector{SVector{3,TF}}(undef,length(system.surfaces))
    surface_moments = Vector{SVector{3,TF}}(undef,length(system.surfaces))

    # wake
    if isnothing(wake)
        max_particles = n_new_particles * max_timesteps
        wake = vpm.ParticleField(max_particles, TF; vpm_options...)
    end

    # zero fields
    reset!(system)
    fmm.reset!(system.fmm_velocity_probes)

    # dimensionalize and set center of mass
    center_of_mass = isnothing(center_of_mass) ? system.reference[].r : center_of_mass
    ref = system.reference[]
    system.reference[] = eltype(system.reference)(1.0, 1.0, 1.0, center_of_mass, 1.0)

    return VortexLatticeModel{TF,typeof(wake),quasisteady}(system, new_particle_probes, surface_forces, surface_moments, wake, toggle_wake, new_particle_overlap_trailing, new_particle_overlap_unsteady, new_particle_p_per_step_trailing, new_particle_p_per_step_unsteady, eta_wake, fmm_expansion_order, fmm_n_per_branch, fmm_multipole_acceptance_criterion_surface, fmm_multipole_acceptance_criterion_wake)
end

function reset!(system::vlm.System{TF}) where TF

    # set all applicable fields to zero
    system.AIC .= zero(TF)
    system.w .= zero(TF)
    system.Γ .= zero(TF)
    system.dΓdt .= zero(TF)
    for v in system.V
        for i in eachindex(v)
            v[i] = zero(SVector{3,TF})
        end
    end
    for dw in system.dw
        dw .= zero(TF)
    end
    for dΓ in system.dΓ
        dΓ .= zero(TF)
    end
    for Vcp in system.Vcp
        for i in eachindex(Vcp)
            Vcp[i] = zero(SVector{3,TF})
        end
    end
    for Vh in system.Vh
        for i in eachindex(Vh)
            Vh[i] = zero(SVector{3,TF})
        end
    end
    for Vv in system.Vv
        for i in eachindex(Vv)
            Vv[i] = zero(SVector{3,TF})
        end
    end
    for Vte in system.Vte
        for i in eachindex(Vte)
            Vte[i] = zero(SVector{3,TF})
        end
    end

end

@inline function n_wake_elements(model::VortexLatticeModel)
    return vpm.get_np(model.wake)
end

@inline function count_probes(surfaces)

    #--- count probes at control points and the center of each bound vortex (including vertical bound vortices) ---#

    n_control_points = 0
    n_surface_filaments = 0
    for surface in surfaces
        nc, ns = size(surface)
        n_control_points += nc * ns # control points
        n_surface_filaments += nc * ns + nc * ns + nc # top (nc*ns) + left (nc*ns) + right (nc)
    end

    #--- count probes at new wake particle locations ---#

    n_new_particles = 0
    for surface in surfaces
        ns = size(surface,2)
        n_new_particles += ns # unsteady loading
        n_new_particles += ns + 1 # trailing vortex
    end

    #--- count probes at trailing edge corners (to inform the wake shedding locations for next timestep) ---#

    n_trailing_edge_corners = 0
    for surface in surfaces
        ns = size(surface,2)
        n_trailing_edge_corners += ns+1
    end

    return n_control_points, n_surface_filaments, n_new_particles, n_trailing_edge_corners
end

#--- solve! subfunctions ---#

function prepare_wake_on_all!(model::VortexLatticeModel, dt)

    #--- unpack model ---#

    fmm_velocity_probes = model.vlm_system.fmm_velocity_probes
    new_particle_probes = model.new_particle_probes
    surfaces = model.vlm_system.surfaces
    wakes = model.vlm_system.wakes
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    ref = model.vlm_system.reference[]
    fs = model.vlm_system.freestream[]
    Vte = model.vlm_system.Vte
    nwake = model.vlm_system.nwake
    eta = model.eta_wake

    #--- count probes at control points and the center of each bound vortex (including vertical bound vortices) ---#

    n_control_points, n_surface_filaments, n_new_particles, n_trailing_edge_corners = count_probes(surfaces)

    #--- update Vte to include the wake and surface influence from the last timestep (should already contain freestream influence for this timestep) ---#

    update_Vte!(Vte, fmm_velocity_probes, n_control_points + n_surface_filaments)

    #--- update wake shedding locations ---#

    # (based on freestream/kinematic/other velocity as well as wake influence from the previous timestep)
    # become the top corners of the next generation of wake
    additional_velocity = nothing # don't use this for now
    vlm.update_wake_shedding_locations!(wakes, wake_shedding_locations,
            surfaces, ref, fs, dt, additional_velocity, Vte,
            nwake, eta
        )

    #--- update probes ---#
    vlm.update_n_probes!(fmm_velocity_probes, n_control_points + n_surface_filaments + n_trailing_edge_corners)
    vlm.update_probes!(fmm_velocity_probes, surfaces, 0) # control points and filament centers
    update_probes_te!(fmm_velocity_probes, surfaces, n_control_points + n_surface_filaments) # trailing edge corners
    vlm.update_probes!(new_particle_probes, surfaces, wake_shedding_locations, model.new_particle_p_per_step_trailing, model.new_particle_p_per_step_unsteady, 0) # new particle locations

    #--- get wake ---#
    wake = model.wake

    #--- reset vlm velocity ---#

    reset!(model.vlm_system)
    fmm.reset!(fmm_velocity_probes)
    fmm.reset!(new_particle_probes)

    #--- reset wake velocity ---#
    vpm._reset_particles(wake)
    vpm.isSFSenabled(wake.SFS) && vpm._reset_particles_sfs(wake)

    #--- all elements ---#
    all = (fmm_velocity_probes, new_particle_probes, wake)

    #--- fmm options ---#
    fmm_options = (expansion_order=model.expansion_order,
                   n_per_branch_source=model.n_per_branch,
                   n_per_branch_target=model.n_per_branch,
                   multipole_acceptance_criterion=model.multipole_acceptance_criterion_wake
                  )

    return all, wake, fmm_options
end

function solve_boundary!(model::VortexLatticeModel, dt)
    #--- unpack system ---#

    vlm_system = model.vlm_system
    eta = model.eta_wake # shed the next wake filament V * dt * eta
    surfaces = vlm_system.surfaces
    wakes = vlm_system.wakes
    nwake = vlm_system.nwake
    wake_shedding_locations = vlm_system.wake_shedding_locations
    wake_finite_core = vlm_system.wake_finite_core
    AIC = vlm_system.AIC
    w = vlm_system.w
    Γ = vlm_system.Γ
    dΓdt = vlm_system.dΓdt
    ref = vlm_system.reference[]
    fs = vlm_system.freestream[]
    Vcp = vlm_system.Vcp
    trailing_vortices = vlm_system.trailing_vortices
    xhat = vlm_system.xhat
    surface_id = vlm_system.surface_id
    symmetric = vlm_system.symmetric
    fmm_velocity_probes = vlm_system.fmm_velocity_probes

    #--- assertions ---#

    @assert !(true in symmetric) "the FMM doesn't support symmetric kernels; set `model.vlm_system.symmetric[] = false`"
    if true in trailing_vortices
        @warn "semiinfinite horseshoes are used for surfaces $(([i for i in 1:length(trailing_vortices) if trailing_vortices[i]])); undesirable behavior may occur"
    end

    #--- update influence matrix ---#
    vlm.influence_coefficients!(AIC, surfaces;
            symmetric = symmetric,                               # the FMM doesn't support symmetric kernels, so this should always be false
            wake_shedding_locations = wake_shedding_locations,   # tip of the vortex filaments that will be converted into particles
            surface_id = surface_id,                             # in vanilla VortexLattice, determines when to use the finite core model (we always use the finite core model in FLOWUnsteady so this parameter is irrelevant)
            trailing_vortices = trailing_vortices,               # ::Vector{Bool} tells which surfaces should use semiinfinite horseshoes
            xhat = xhat                                          # only used if semi-infinite horseshoes are used
        )

    #--- update the influence matrix to use the new wake shedding locations ---#

    vlm.update_trailing_edge_coefficients!(AIC, surfaces;
            symmetric = symmetric,                               # the FMM doesn't support symmetric kernels, so this should always be false
            wake_shedding_locations = wake_shedding_locations,   # tip of the vortex filaments that will be converted into particles
            trailing_vortices = trailing_vortices,               # ::Vector{Bool} telling which systems should use semiinfinite horseshoes
            xhat = xhat                                          # only used if semi-infinite horseshoes are used
        )

    #--- set up boundary condition ---#

    additional_velocity = nothing # not using additional_velocity
    vlm.normal_velocity!(w, surfaces, wakes, ref, fs, fmm_velocity_probes;
            additional_velocity, Vcp, symmetric, nwake, surface_id,
            wake_finite_core, trailing_vortices, xhat
        )

    #--- save (negative) previous circulation in dΓdt ---#

    dΓdt .= .-Γ

    #--- solve for the new circulation ---#

    vlm.circulation!(Γ, AIC, w)

    #--- solve for dΓdt using finite difference `dΓdt = (Γ - Γp)/dt` ---#

    dΓdt .+= Γ # add newly computed circulation
    dΓdt ./= dt # divide by corresponding time step

    return nothing
end

function prepare_surface_on_all!(model::VortexLatticeModel)

    #--- unpack model---#

    fmm_panels = model.vlm_system.fmm_panels
    surfaces = model.vlm_system.surfaces
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    Γ = model.vlm_system.Γ
    new_particle_probes = model.new_particle_probes
    fmm_velocity_probes = model.vlm_system.fmm_velocity_probes
    wake = model.wake

    #--- update fmm_panels ---#

    vlm.update_fmm_panels!(fmm_panels, surfaces, wake_shedding_locations, Γ)

    #--- all elements ---#

    all = (fmm_velocity_probes, new_particle_probes, wake)

    #--- surface ---#

    surface = model.vlm_system

    #--- fmm options ---#

    fmm_options = (expansion_order=model.expansion_order,
                   n_per_branch_source=model.n_per_branch,
                   n_per_branch_target=model.n_per_branch,
                   multipole_acceptance_criterion=model.multipole_acceptance_criterion_surface
                  )

    return all, surface, fmm_options
end


function shed_wake!(model::VortexLatticeModel{TF,<:Any,<:Any}, dt) where TF

    #--- unpack model ---#

    toggle_wake = model.toggle_wake
    new_particle_probes = model.new_particle_probes
    new_particle_overlap_trailing = model.new_particle_overlap_trailing
    new_particle_overlap_unsteady = model.new_particle_overlap_unsteady
    new_particle_p_per_step_trailing = model.new_particle_p_per_step_trailing
    new_particle_p_per_step_unsteady = model.new_particle_p_per_step_unsteady
    surfaces = model.vlm_system.surfaces
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    Γ = model.vlm_system.Γ
    dΓdt = model.vlm_system.dΓdt
    wake = model.wake

    #--- shed particles ---#

    i_probe = 0
    iΓ = 0
    for (surface, wsl, trailing_pps, unsteady_pps, trailing_overlap, unsteady_overlap, toggle) in zip(surfaces, wake_shedding_locations, new_particle_p_per_step_trailing, new_particle_p_per_step_unsteady, new_particle_overlap_trailing, new_particle_overlap_unsteady, toggle_wake)
        if toggle
            nc, ns = size(surface)
            Γ_last = zero(TF)
            for i_te in 1:ns
                # trailing edge panel
                panel = surface[nc,i_te]

                # circulation magnitude of this wake panel
                iΓ += nc
                Γ_magnitude = Γ[iΓ]
                dΓdt_magnitude = dΓdt[iΓ]

                # left trailing vortex
                i_probe = add_line!(wake, new_particle_probes, wsl[i_te], panel.rbl, i_probe, trailing_pps, Γ_magnitude - Γ_last, trailing_overlap)

                # unsteady loading
                i_probe = add_line!(wake, new_particle_probes, wsl[i_te+1], wsl[i_te], i_probe, unsteady_pps, dΓdt_magnitude * dt, unsteady_overlap)

                # recurse
                Γ_last = Γ_magnitude
            end

            # rightmost trailing vortex
            panel = surface[nc,end]
            i_probe = add_line!(wake, new_particle_probes, panel.rbr, wsl[end], i_probe, trailing_pps, Γ_last, trailing_overlap)

        end
    end
end

function forces!(model::VortexLatticeModel)

    #--- unpack model ---#

    vlm_system = model.vlm_system
    surfaces = vlm_system.surfaces
    props = vlm_system.properties
    wakes = vlm_system.wakes
    ref = vlm_system.reference[]

    fs = vlm_system.freestream[]
    @assert iszero(fs.Omega) "VLM freestream object rotational velocity should be zero: got $(fs.Omega)"

    Γ = vlm_system.Γ
    fmm_velocity_probes = vlm_system.fmm_velocity_probes
    dΓdt = vlm_system.dΓdt
    Vh = vlm_system.Vh
    Vv = vlm_system.Vv
    symmetric = vlm_system.symmetric
    nwake = vlm_system.nwake
    surface_id = vlm_system.surface_id
    wake_finite_core = vlm_system.wake_finite_core
    wake_shedding_locations = vlm_system.wake_shedding_locations
    trailing_vortices = vlm_system.trailing_vortices
    xhat = vlm_system.xhat
    vertical_segments = true

    #--- calculate forces ---#

    additional_velocity = nothing # ignore this for now
    vlm.near_field_forces!(props, surfaces, wakes, ref, fs, Γ, fmm_velocity_probes;
            dΓdt, additional_velocity, Vh, Vv, symmetric, nwake, surface_id,
            wake_finite_core, wake_shedding_locations, trailing_vortices, xhat, vertical_segments
        )

end

#--- auxiliary functions for VortexLatticeModel ---#

"""
    add_line!(wake, probes, x1, x2, i_last, n_particles, Γ_magnitude, overlap)

Replace the vortex filament defined by `x1` and `x2` with vortex particles at coincident with the next `n_particles` probes in `probes`, inheriting the position, velocity, and velocity gradient of the probes.

# Arguments

* `wake::FLOWVPM.ParticleField`: particle field to receive the new particles
* `probes::FastMultipole.ProbeSystem`: contains colinear probes to be converted into particles
* `x1::SVector{3,Float64}`: position of the first point defining the vortex filament
* `x2::SVector{3,Float64}`: position of the second point defining the vortex filament
* `i_last::Int`: index of the last probe before the current probes
* `n_particles::Int`: number of particles desired
* `Γ_magnitude::Float64`: circulation strength of the vortex filament
* `overlap::Float64`: determines the core radius of particles as ''\\sigma = \\frac{\\Delta x}{2n} \\times ''`overlap` where ''\\Delta x'' is the distance partitioned for a single particle; e.g. `overlap=0` means particle radii barely touch

# Output

* `i_last::Int`: the index of the last probe used

"""
@inline function add_line!(wake::vpm.ParticleField, probes::fmm.ProbeSystem, x1, x2, i_last::Int, n_particles::Int, Γ_magnitude, overlap)

    # strength and radius
    Γ = x2 - x1 # strength is aligned with the line
    Δx = norm(Γ) / n_particles # distance spanned by a single particle
    σ = Δx / 2 * (1 + overlap) # divide by 2 to get a radius rather than a diameter
    Γ *= Γ_magnitude / n_particles # divide strength by the number of particles in the line

    # shed particles
    i_particle = 0
    for _ in 1:n_particles
        i_particle += 1
        X = probes.position[i_particle + i_last]

        # ensure probes are colinear
        @assert isapprox(abs(dot(Γ, X-x1)), norm(Γ) * norm(X-x1); atol=1e-12) "probes used for particle shedding are not colinear"

        # add particle
        vpm.add_particle(wake, X, Γ, σ; vol=0, circulation=1, C=0)
    end

    # return the index of the last probe used
    return i_particle + i_last
end

"""
    update_probes!(probes::ProbeSystem, surfaces::Vector{Matrix{<:SurfacePanel}}, wake_shedding_locations::Vector{Vector{<:Vector}}, i_start::Int)

Places probes at the new shed locations, which are evenly distributed over the vertical and bottom vortices of the transition panels.

# Arguments

* `probes::FastMultipole.ProbeSystem`: system of probes to be updated using the shed locations
* `surfaces::Vector{Matrix{<:VortexLattice.SurfacePanel}}`: vector of surfaces from which particles are to be shed
* `wake_shedding_locations::Vector{Vector{<:Vector}}`: positions of the corners of the transition panels
* `p_per_step_trailing::Vector{Int}`: number of particles to shed per step per trailing vortex for each surface
* `p_per_step_unsteady::Vector{Int}`: number of particles to shed per step per unsteady vortex for each surface
* `i_last`: the index of the last-added probe

# Output

* `i_last::Int`: updated index of the last-added probe

"""
function vlm.update_probes!(probes::fmm.ProbeSystem{<:Any,<:Any,<:Any,<:Any,<:Any}, surfaces::Vector{Matrix{vlm.SurfacePanel{TF}}}, wake_shedding_locations::Vector{Vector{SVector{3,TF}}}, p_per_step_trailing, p_per_step_unsteady, i_last) where TF
    # loop over surfaces
    for (surface, wsl, pps_t, pps_u) in zip(surfaces, wake_shedding_locations, p_per_step_trailing, p_per_step_unsteady)
        nc, ns = size(surface)
        # loop over panels
        for j_panel in 1:ns
            panel = surface[nc, j_panel]

            # left trailing vortex
            i_last = fmm.add_line!(probes, panel.rbl, wsl[j_panel], pps_t, i_last)

            # unsteady loading
            i_last = fmm.add_line!(probes, wsl[j_panel], wsl[j_panel+1], pps_u, i_last)
        end

        # right-most trailing vortex
        panel = surface[nc, end]
        i_last = fmm.add_line!(probes, panel.rbr, wsl[end], pps_t, i_last)
    end

    # return updated i_last
    return i_last
end

"""
    update_probes_te!(probes::ProbeSystem, surfaces::Vector{Matrix{<:SurfacePanel}}, i_last::Int)

Place probes at the trailing edge corners.

# Arguments

* `probes::FastMultipole.ProbeSystem`: probe system to be updated
* `surfaces::Vector{Matrix{<:VortexLattice.SurfacePanel}}: `VortexLattice` surfaces at whose trailing edge corners probes are to be added
* `i_last::Int`: index of the last added probe in `probes`

# Output

* `i_last::Int`: updated index of the last added probe in `probes`

"""
function update_probes_te!(probes::fmm.ProbeSystem, surfaces, i_last)
    i = 0

    # loop over surfaces
    for surface in surfaces
        nc, ns = size(surface)

        # loop over trailing edge panels
        for j_panel in 1:ns
            panel = surface[nc, j_panel]

            # bottom left corner
            i += 1
            probes.position[i + i_last] = panel.rbl
        end

        # bottom right-most corner
        i += 1
        probes.position[i + i_last] = surface[nc, end].rbr
    end

    # return updated i_last
    return i + i_last
end

"""
    update_Vte!(Vte, probes, i_last)

Add the probed velocity to each trailing edge point.

# Arguments

* `Vte::Vector{Vector{SVector{3,Float64}}}`: vector of trailing edge velocities at each surface; this is a field in `::VortexLattice.System` objects
* `probes::FastMultipole.ProbeSystem`: probes whose velocity is to be added to the trailing edge
* `i_last::Int`: index of the last probe in `probes` before the trailing edge probes

# Output

* `i_last::Int`: index of the last trailing edge probe in `probes`

"""
function update_Vte!(Vte, probes, i_last)
    i = 0
    for vte in Vte # loop over surfaces
        for i_corner in eachindex(vte) # loop over corners
            i += 1
            vte[i_corner] += probes.velocity[i + i_last]
        end
    end

    # return updated i_last
    return i + i_last
end

@inline function vlm.rotate(panel::vlm.SurfacePanel, Q::Quaternion, r = (@SVector zeros(3)))

    rtl = rotate(panel.rtl - r, Q) + r
    rtc = rotate(panel.rtc - r, Q) + r
    rtr = rotate(panel.rtr - r, Q) + r
    rbl = rotate(panel.rbl - r, Q) + r
    rbc = rotate(panel.rbc - r, Q) + r
    rbr = rotate(panel.rbr - r, Q) + r
    rcp = rotate(panel.rcp - r, Q) + r
    ncp = rotate(panel.ncp, Q)
    core_size = panel.core_size
    chord = panel.chord

    return vlm.SurfacePanel(rtl, rtc, rtr, rbl, rbc, rbr, rcp, ncp, core_size, chord)
end

@inline function rotate!(surface::AbstractMatrix{<:vlm.SurfacePanel}, Q, r = (@SVector zeros(3)))
    # rotate
    for i in CartesianIndices(surface)
        surface[i] = vlm.rotate(surface[i], Q, r)
    end
end

@inline function rotational_velocity(point, angular_velocity, center_of_rotation)
    r = center_of_rotation - point
    v = cross(r, angular_velocity)
    return v
end

@inline function kinematic_velocity!(system::vlm.System, map::Int, velocity, angular_velocity, center_of_rotation)

    # unpack surface
    surface = system.surfaces[map]
    Vcp = system.Vcp[map]
    Vh = system.Vh[map]
    Vv = system.Vv[map]
    # Vte = system.Vte[map]

    # apply translational velocity
    for i in eachindex(Vcp)
        Vcp[i] -= velocity
    end

    for i in eachindex(Vh)
        Vh[i] -= velocity
    end

    for i in eachindex(Vv)
        Vv[i] -= velocity
    end

    # for i in eachindex(Vte)
    #     Vte[i] -= velocity
    # end

    #--- apply rotational velocity ---#
    nc, ns = size(surface)

    # at control points
    for j in 1:ns, i in 1:nc
        point = vlm.controlpoint(surface[i,j])
        Vcp[i,j] -= rotational_velocity(point, angular_velocity, center_of_rotation)
    end

    # at horizontal bound vortices
    for j in 1:ns
        for i in 1:nc
            point = vlm.top_center(surface[i,j])
            Vh[i,j] -= rotational_velocity(point, angular_velocity, center_of_rotation)
        end
        point = vlm.bottom_center(surface[end,j])
        Vh[end,j] -= rotational_velocity(point, angular_velocity, center_of_rotation)
    end

    # at vertical bound vortices
    for i in 1:nc
        for j in 1:ns
            point = vlm.left_center(surface[i,j])
            Vv[i,j] -= rotational_velocity(point, angular_velocity, center_of_rotation)
        end
        point = vlm.right_center(surface[i,end])
        Vv[i,end] -= rotational_velocity(point, angular_velocity, center_of_rotation)
    end

    # # at trailing edge vertices (THESE ARE USED TO DETERMINE THE NEXT WAKE SHED LOCATIONS, SO NO NEED TO INCLUDE KINEMATIC VELOCITY)
    # for j in 1:ns
    #     point = vlm.bottom_left(surface[end,j])
    #     Vte[j] += rotational_velocity(point, angular_velocity, center_of_rotation)
    # end
    # point = vlm.bottom_right(surface[end,end])
    # Vte[j] += rotational_velocity(point, angular_velocity, center_of_rotation)

    return nothing
end

function kinematic_velocity!(system::vlm.System, maps, velocity, angular_velocity, center_of_rotation)
    for map in maps
        kinematic_velocity!(system, map, velocity, angular_velocity, center_of_rotation)
    end
end

function kinematic_velocity!(model::VortexLatticeModel, maps, velocity, angular_velocity, center_of_rotation)
    kinematic_velocity!(model.vlm_system, maps, velocity, angular_velocity, center_of_rotation)
end

"""
    apply!(system, freestream)

Apply `freestream` properties to the `system`.

# Arguments

* `system::VortexLattice.System`: the `system` to which the `freestream` is to be applied
* `freestream::SimpleFreestream`: a `FLOWUnsteady.SimpleFreestream` object

"""
function apply!(system::vlm.System, freestream::SimpleFreestream)
    # unpack freestream
    velocity = freestream.velocity
    density = freestream.density

    # apply fluid density to model
    fs = eltype(system.freestream)(0.0, 0.0, 0.0, SVector{3}(0.0,0,0), density)
    system.freestream[] = fs

    # apply freestream velocity to model control points and bound vortex centers
    kinematic_velocity!(system, 1:length(system.surfaces), -velocity, zero(SVector{3,Float64}), zero(SVector{3,Float64}))

    # apply freestream velocity to Vte
    Vte = system.Vte
    for vte in Vte
        for i in eachindex(vte)
            vte[i] += velocity
        end
    end

end

"""
    apply!(particle_field, freestream)

Apply `freestream` properties to the `particle_field`.

# Arguments

* `particle_field::FLOWVPM.ParticleField`: the `particle_field` to which the `freestream` is to be applied
* `freestream::SimpleFreestream`: a `FLOWUnsteady.SimpleFreestream` object

"""
function apply!(particle_field::vpm.ParticleField, freestream::SimpleFreestream)
    for p in vpm.iterator(particle_field)
        vpm.get_U(p) .+= freestream.velocity
    end
end

"""
    apply!(model, freestream)

Apply `freestream` properties to the `model`.

# Arguments

* `model::VortexLatticeModel`: the `model` to which the `freestream` is to be applied
* `freestream::SimpleFreestream`: a `FLOWUnsteady.SimpleFreestream` object

"""
function apply!(model::VortexLatticeModel, freestream::SimpleFreestream)
    # apply to vlm
    apply!(model.vlm_system, freestream)

    # apply to wake
    apply!(model.wake, freestream)

    # apply freestream velocity to new_particle_probes
    for i in eachindex(model.new_particle_probes.velocity)
        model.new_particle_probes.velocity[i] += freestream.velocity
    end
end

function transform!(model::vlm.System, map::Int, translation, rotation::Quaternion, center_of_rotation)
    # rotate
    rotate!(model.surfaces[map], rotation, center_of_rotation)

    # translate
    vlm.translate!(model.surfaces[map], translation)
end

function transform!(model::VortexLatticeModel, map::AbstractVector{Int}, translation, rotation::Quaternion, center_of_rotation)
    for i in map
        transform!(model.vlm_system, i, translation, rotation, center_of_rotation)
    end
end

function transform!(model::VortexLatticeModel, translation, rotation::Quaternion, center_of_rotation)
    # update reference
    ref = model.reference[]
    model.reference[] = vlm.Reference(ref.S, ref.c, ref.b, ref.r + translation, ref.V)
end

"""
    force!(model, map, center)

Computes the dimensional force applied to the portion of the `model` designated by `map` about `center` and stores it to `model.surface_forces` and `model.surface_moments`. Units are defined implicitly by user inputs.

# Arguments

* `model::VortexLatticeModel`: the model from which forces are to be computed
* `map::Int`: the index of `model.vlm_system.surfaces` from which forces are to be computed

# Returns

* `force::SVector{3,Float64}`: the net force experienced by the portion of `model` designated by `map`
* `moment::SVector{3,Float64}`: the net moment experienced by the portion of `model` designated by `map` about `center`

"""
function force!(model::VortexLatticeModel, map::Int, center)
    f, m = force(model.vlm_system, map, center)
    model.surface_forces[map] = f
    model.surface_moments[map] = m
    return f, m
end

function force!(model::VortexLatticeModel{TF,<:Any,<:Any}, maps, center) where TF
    force = zero(SVector{3,TF})
    moment = zero(SVector{3,TF})
    for map in maps
        f, m = force!(model, map, center)
        force += f
        moment += m
    end
    return force, moment
end

"""
    force(model, map, center)

Computes the dimensional net force and net moment obtained by the VLM on `model.surfaces[map]`. Moments are computed about `center`. Units are defined implicitly.

# Arguments

* `model::VortexLattice.System`: the vortex lattice system object used
* `map::Union{Int,Vector{Int}}`: the index(indices) of the surfaces over which forces and moments are accumulated (typically the results of `map_all(model)`)
* `center::SVector{3,Float64}`: the point about which moments are evaluated

# Returns

* `force::SVector{3,Float64}`: the net force evaluated by the VLM
* `moment::SVector{3,Float64}`: the net moment evaluated by the VLM about `center`

"""
function force(model::vlm.System, map, center)
    surfaces = view(model.surfaces, map)
    properties = view(model.properties, map)
    ref = vlm.Reference(1.0,1.0,1.0,center,1.0) # hard-coded to 1 to yield dimensional force
    fs = model.freestream
    symmetric = model.symmetric

    # accumulate forces for these surfaces; note that `convention_change=[1.0,1.0,1.0]` assumes that `model` is defined using flight dynamics convention (north-east-down frame)
    force, moment = vlm.body_forces(surfaces, properties, ref, fs, symmetric, vlm.Body(); convention_change=[1.0,1.0,1.0]) # note that body frame IS the global frame here

    return force, moment
end

function map_all(model::VortexLatticeModel)
    return collect(1:length(model.vlm_system.surfaces))
end

#--- visualization functions ---#

function visualize(system::vlm.System, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default_", path="./")
    name_suffix = isnothing(i) ? "vlm" : "vlm-step$i"
    kwargs = isnothing(t) ? () : ((metadata = Dict{String,Float64}("time" => t)),)
    vlm.write_vtk(joinpath(path, name_prefix*name_suffix), system.surfaces, system.properties; symmetric = fill(false, length(system.surfaces)), trailing_vortices=false, kwargs...)
end

function visualize(wake::vpm.ParticleField, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default_", path="./")
    name_suffix = "particlefield"
    kwargs = isnothing(i) ? () : (add_num=true, num=i)
    !isnothing(t) && (kwargs = (kwargs..., overwrite_time=t))
    vpm.save(wake, name_prefix * name_suffix; path, kwargs...)
end

function visualize(model::VortexLatticeModel, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default_", path="./")
    visualize(model.vlm_system, i, t; name_prefix, path)
    visualize(model.wake, i, t; name_prefix, path)
end

function initialize_history(model::VortexLatticeModel, save_steps)
    n_steps = length(save_steps)

    # save net force, net moment, force for each surface, moment for each surface
    force_history = zeros(3, n_steps)
    moment_history = zeros(3, n_steps)

    n_surfaces = length(model.vlm_system.surfaces)
    surface_force_history = zeros(3, n_surfaces, n_steps)
    surface_moment_history = zeros(3, n_surfaces, n_steps)

    # assemble output
    history = (
        force = force_history,
        moment = moment_history,
        surface_force = surface_force_history,
        surface_moment = surface_moment_history
    )

    fields = (:force, :moment, :surface_force, :surface_moment)

    return history, fields
end

function update_history!(history, model::VortexLatticeModel, i_step)
    # surface forces
    n_surfaces = length(model.vlm_system.surfaces)
    for i in 1:n_surfaces
        history[:surface_force][:,i,i_step] .= model.surface_forces[i]
        history[:surface_moment][:,i,i_step] .= model.surface_moments[i]
    end

    # top level forces
    for i in 1:3
        history[:force][i,i_step] = sum(view(history[:surface_force],i,:,i_step))
        history[:moment][i,i_step] = sum(view(history[:surface_moment],i,:,i_step))
    end
end
