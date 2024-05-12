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

"""
    <: AbstractBoundaryElementModel{TF,Quasisteady}

Subtype of `<:AbstractModel` for float type `TF`. Provides a convenient framework for defining the `solve!` function for any specific instance of `AbstractBoundaryElementModel`. Instead of writing the entire `solve!` function, the user need only define the following smaller functions:

* `all, wake, fmm_options = prepare_wake_on_all!(model::AbstractModel)`: `all` and `wake` are target and source, respectively, in a call to `FastMultipole.fmm!(all, wake; fmm_options...)`
* `solve_boundary!(model::AbstractModel, dt)`: in-place function solves all boundary elements in the `model`
* `all, surface, fmm_options = prepare_surface_on_all!(model::AbstractModel, dt)`: `all` and `surface` are target and source, respectively, in a call to `FastMultipole.fmm!(all, surface; fmm_options...)`
* `shed_wake!(model::AbstractModel)`: in-place function sheds wake elements
* `forces!(model::AbstractModel)`: in-place function calculates forces based on the solved `model`

"""
abstract type AbstractBoundaryElementModel{TF,Quasisteady} <: AbstractModel{TF,Quasisteady} end

#--- fmm setup ---#

requires_scalar_potential(system) = false
requires_vector_potential(system) = false
requires_velocity(system) = true
requires_velocity_gradient(system) = false
requires_velocity_gradient(::vpm.ParticleField) = true
requires_velocity_gradient(::ProbeSystem{<:Any,<:Any,<:Any,<:Any,true}) = true

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

    all, wake, fmm_options = prepare_wake_on_all!(model)

    scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)

    fmm!(all, wake; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    #--- solve boundary elements ---#

    solve_boundary!(model, dt)

    #--- surface-on-all induced velocity ---#

    all, surface, fmm_options = prepare_surface_on_all!(model)

    scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)

    fmm!(all, surface; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    #--- shed wake for this timestep ---#

    shed_wake!(model)

    #--- compute forces ---#

    forces!(model)

    return nothing
end

#------- Vortex Lattice Model -------#

struct VortexLatticeModel{TF,TW,Quasisteady} <: AbstractBoundaryElementModel{TF,Quasisteady}
    # vlm
    vlm_system::vlm.System{TF}
    new_particle_probes::ProbeSystem{TF, Nothing, Nothing, Vector{SVector{3,TF}}, Vector{SMatrix{3,3,TF,9}}}

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

    # new particle probes
    n_new_particles = 0
    for surface in system.surfaces
        ns = size(surface,2)
        n_new_particles += ns # unsteady loading
        n_new_particles += ns + 1 # trailing vortex
    end
    new_particle_probes = ProbeSystem(n_new_particles; velocity=true, velocity_gradient=true)

    # wake
    if isnothing(wake)
        max_particles = n_new_particles * max_timesteps
        wake = vpm.ParticleField(max_particles; R=TF, vpm_options...)
    end

    return VortexLatticeModel{TF,typeof(wake),quasisteady}(system, new_particle_probes, wake, toggle_wake, new_particle_overlap_trailing, new_particle_overlap_unsteady, new_particle_p_per_step_trailing, new_particle_p_per_step_unsteady, eta_wake, fmm_expansion_order, fmm_n_per_branch, fmm_multipole_acceptance_criterion_surface, fmm_multipole_acceptance_criterion_wake)
end

#--- solve! subfunctions ---#

function prepare_wake_on_all!(model::VortexLatticeModel)

    #--- unpack model ---#

    fmm_velocity_probes = model.vlm_system.fmm_velocity_probes
    new_particle_probes = model.new_particle_probes
    surfaces = model.vlm_system.surfaces
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    ref = model.vlm_system.reference
    fs = model.vlm_system.freestream
    Vte = model.vlm_system.Vte
    nwake = model.vlm_system.nwake
    eta = model.eta_wake

    #--- place probes at control points and the center of each bound vortex (including vertical bound vortices) ---#

    n_control_points = 0
    n_surface_filaments = 0
    for surface in surfaces
        nc, ns = size(surface)
        n_control_points += nc * ns # control points
        n_surface_filaments += nc * ns + nc * ns + nc # top (nc*ns) + left (nc*ns) + right (nc)
    end

    #--- place probes at new wake particle locations ---#

    n_new_particles = 0
    for surface in surfaces
        ns = size(surface,2)
        n_new_particles += ns # unsteady loading
        n_new_particles += ns + 1 # trailing vortex
    end

    #--- place probes at trailing edge corners (to inform the wake shedding locations for next timestep) ---#

    n_trailing_edge_corners = 0
    for surface in surfaces
        ns = size(surface,2)
        n_trailing_edge_corners += ns+1
    end

    #--- update Vte to include the wake influence from the last timestep ---#

    update_Vte!(Vte, fmm_velocity_probes.velocity, n_control_points + n_surface_filaments)

    #--- update wake shedding locations ---#

    # (based on freestream/kinematic/other velocity as well as wake influence from the previous timestep)
    # become the top corners of the next generation of wake
    additional_velocity = nothing # don't use this for now
    update_wake_shedding_locations!(wakes, wake_shedding_locations,
            surfaces, ref, fs, dt, additional_velocity, Vte,
            nwake, eta
        )

    #--- update probes ---#

    update_n_probes!(fmm_velocity_probes, n_control_points + n_surface_filaments + n_trailing_edge_corners + n_new_particles)
    vlm.update_probes!(fmm_velocity_probes, surfaces, 0) # control points and filament centers
    update_probes_te!(fmm_velocity_probes, surfaces, n_control_points + n_surface_filaments) # trailing edge corners
    vlm.update_probes!(new_particle_probes, surfaces, wake_shedding_locations, 0) # new particle locations

    # reset to zero
    reset!(fmm_velocity_probes)

    #--- get wake ---#
    wake = model.wake

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

    vlm_system = model.surface
    eta = model.eta_wake # shed the next wake filament V * dt * eta
    surfaces = vlm_system.surfaces
    wakes = vlm_system.wakes
    nwake = vlm_system.nwake
    wake_shedding_locations = vlm_system.wake_shedding_locations
    AIC = vlm_system.AIC
    w = vlm_system.w
    Γ = vlm_system.Γ
    dΓdt = vlm_system.dΓdt
    ref = vlm_system.ref
    fs = vlm_system.fs
    Vte = vlm_system.Vte
    trailing_vortices = vlm_system.trailing_vortices
    xhat = vlm_system.xhat
    surface_id = vlm_system.surface_id

    #--- assertions ---#

    @assert symmetric == false "the FMM doesn't support symmetric kernels; set `model.vlm_system.symmetric[] = false`"
    @warn true in trailing_vortices "semiinfinite horseshoes are used for surfaces $([i for i in 1:length(trailing_vortices) if trailing_vortices[i]]); undesirable behavior may occur"

    #--- update influence matrix ---#

    vlm.influence_coefficients!(AIC, surfaces;
            symmetric = symmetric,                               # the FMM doesn't support symmetric kernels, so this should always be false
            wake_shedding_locations = wake_shedding_locations,   # tip of the vortex filaments that will be converted into particles
            surface_id = surface_id,                             # in vanilla VortexLattice, determines when to use the finite core model (we always use the finite core model in FLOWUnsteady so this parameter is irrelevant)
            trailing_vortices = trailing_vortices,               # ::Vector{Bool} tells which surfaces should use semiinfinite horseshoes
            xhat = xhat                                          # only used if semi-infinite horseshoes are used
        )

    #--- update the influence matrix to use the new wake shedding locations ---#

    update_trailing_edge_coefficients!(AIC, surfaces;
            symmetric = symmetric,                               # the FMM doesn't support symmetric kernels, so this should always be false
            wake_shedding_locations = wake_shedding_locations,   # tip of the vortex filaments that will be converted into particles
            trailing_vortices = trailing_vortices,               # ::Vector{Bool} telling which systems should use semiinfinite horseshoes
            xhat = xhat                                          # only used if semi-infinite horseshoes are used
        )

    #--- set up boundary condition ---#

    normal_velocity!(w, surfaces, wakes, ref, fs, fmm_velocity_probes;
            additional_velocity, Vcp, symmetric, nwake, surface_id,
            wake_finite_core, trailing_vortices, xhat
        )

    #--- save (negative) previous circulation in dΓdt ---#

    dΓdt .= .-Γ

    #--- solve for the new circulation ---#

    circulation!(Γ, AIC, w)

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

    #--- update fmm_panels ---#

    update_fmm_panels!(fmm_panels, surfaces, wake_shedding_locations, Γ)

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
    for (surface, wsl, trailing_overlap, unsteady_overlap, toggle) in zip(surfaces, wake_shedding_locations, new_particle_trailing_overlap, new_particle_unsteady_overlap, toggle_wake)
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
                i_probe = add_line!(wake, new_particle_probes, wsl[i_te], panel.rbl, i_probe, new_particle_p_per_step_trailing, Γ_magnitude - Γ_last, new_particle_overlap_trailing)

                # unsteady loading
                i_probe = add_line!(wake, new_particle_probes, wsl[i_te+1], wsl[i_te], i_probe, new_particle_p_per_step_unsteady, dΓdt_magnitude * dt, new_particle_overlap_unsteady)

                # recurse
                Γ_last = Γ_magnitude
            end

            # rightmost trailing vortex
            i_probe = add_line!(wake, new_particle_probes, panel.rbr, wsl[end], i_probe, new_particle_p_per_step_trailing, Γ_last, new_particle_overlap_trailing)
        end
    end
end

function forces!(model::VortexLatticeModel)

    #--- unpack model ---#

    vlm_system = model.vlm_system
    props = vlm_system.properties
    wakes = vlm_system.wakes
    ref = vlm_system.reference
    fs = vlm_system.freestream
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
    near_field_forces!(props, surfaces, wakes, ref, fs, Γ, fmm_velocity_probes;
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

Places probes at the new shed locations, which are at the midpoint of the vertical and bottom vortices of the transition panels.

# Arguments

* `probes::FastMultipole.ProbeSystem`: system of probes to be updated using the shed locations
* `surfaces::Vector{Matrix{<:VortexLattice.SurfacePanel}}`: vector of surfaces from which particles are to be shed
* `wake_shedding_locations::Vector{Vector{<:Vector}}`: positions of the corners of the transition panels
* `i_last`: the index of the last-added probe

# Output

* `i_last::Int`: updated index of the last-added probe

"""
function vlm.update_probes!(probes::FastMultipole.ProbeSystem{<:Any,<:Any,<:Any,<:Any,<:Any}, surfaces::Vector{Matrix{vlm.SurfacePanel{TF}}}, wake_shedding_locations::Vector{Vector{SVector{3,TF}}}, i_last) where TF
    # loop over surfaces
    for (surface, wsl) in zip(surfaces, wake_shedding_locations)
        nc, ns = size(surface)
        # loop over panels
        for j_panel in 1:ns
            panel = surface[nc, j_panel]

            # left trailing vortex
            i_last = add_line!(probes, panel.rbl, wsl[j_panel], p_per_step_trailing, i_last)

            # unsteady loading
            i_last = add_line!(probes, wsl[j_panel], wsl[j_panel+1], p_per_step_unsteady, i_last)
        end

        # right-most trailing vortex
        i_last = add_line!(probes, panel.rbr, wsl[end], p_per_step, trailing, i_last)
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
function update_probes_te!(probes::vlm.FastMultipole.ProbeSystem, surfaces::Vector{Matrix{<:vlm.SurfacePanel}}, i_last)
    i = 0

    # loop over surfaces
    for surface in surfaces
        nc, ns = size(surface, 1)

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
            vte[i_corner] += velocity[i + i_last]
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

    return SurfacePanel(rtl, rtc, rtr, rbl, rbc, rbr, rcp, ncp, core_size, chord)
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

function kinematic_velocity!(system::vlm.System, map::Int, velocity, angular_velocity, center_of_rotation)

    # unpack surface
    surface = system.surfaces[map]
    Vcp = system.Vcp[map]
    Vh = system.Vh[map]
    Vv = system.Vv[map]
    Vte = system.Vte[map]

    # apply translational velocity
    Vcp .-= velocity
    Vh .-= velocity
    Vv .-= velocity
    Vte .-= velocity

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
        point = vlm.right_center(surface[i,j])
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

function kinematic_velocity!(model::VortexLatticeModel, map::Int, velocity, angular_velocity, center_of_rotation)
    kinematic_velocity!(model.vlm_system, map, velocity, angular_velocity, center_of_rotation)
end

function transform!(model::vlm.System, map::Int, translation, rotation::Quaternion, center_of_rotation)
    # rotate
    rotate!(model.surfaces[map], rotation, center_of_rotation)

    # translate
    vlm.translate!(model.surfaces[map], translation)
end

function transform!(model::VortexLatticeModel, map::Int, velocity, angular_velocity, center_of_rotation)
    transform!(model.vlm_system, map, velocity, angular_velocity, center_of_rotation)
end

function map_all(model::VortexLatticeModel)
    return collect(1:length(model.vlm_system.surfaces))
end

