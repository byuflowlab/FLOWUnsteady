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
    @error "transform!(model, map, translation, rotation, center_of_rotation) not defined for $(typeof(model))"
end

function transform!(model::AbstractModel, dt)
    @error "transform!(model, dt) not defined for $(typeof(model))"
end

function map_all(model::AbstractModel)
    @error "map_all not defined for $(typeof(model))"
end

function reset!(model::AbstractModel)
    @error "reset! not defined for $(typeof(model))"
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

abstract type AbstractWake{TF,Quasisteady} <: AbstractModel{TF,Quasisteady} end

"""
    <: AbstractBoundaryElementModel{TF,Quasisteady,FMM}

Subtype of `<:AbstractModel` for float type `TF`. Provides a convenient framework for defining the `solve!` function for any specific instance of `AbstractBoundaryElementModel`. Instead of writing the entire `solve!` function, the user need only define the following smaller functions:

* `all, wake, fmm_options = prepare_wake_on_all!(model::AbstractModel, dt)`: `all` and `wake` are target and source, respectively, in a call to `FastMultipole.fmm!(all, wake; fmm_options...)`
* `solve_boundary!(model::AbstractModel, dt)`: in-place function solves all boundary elements in the `model`
* `all, surface, fmm_options = prepare_surface_on_all!(model::AbstractModel, dt)`: `all` and `surface` are target and source, respectively, in a call to `FastMultipole.fmm!(all, surface; fmm_options...)`
* `shed_wake!(model::AbstractModel)`: in-place function sheds wake elements
* `forces!(model::AbstractModel)`: in-place function calculates forces based on the solved `model`

"""
abstract type AbstractBoundaryElementModel{TF,Quasisteady,FMM} <: AbstractModel{TF,Quasisteady} end

"""
    prepare_wake_on_all!(model)

Prepares the `model` for an upcoming wake-on-all computation.

# Arguments

* `model::AbstractBoundaryElementModel`: the `model` for which the wake-on-all computation will be performed

"""
function prepare_wake_on_all!(model::AbstractBoundaryElementModel)
    @error "prepare_wake_on_all! not defined for $(typeof(model))"
end

"""
    prepare_wake_on_all!(wake, model)

Prepares the `wake` for an upcoming wake-on-all computation using information from the `model`.

# Arguments

* `wake::AbstractWake`: the `wake` model to be conditioned
* `model::AbstractModel`: the `model` corresponding to this `wake`

"""
function prepare_wake_on_all!(wake::AbstractWake, model::AbstractBoundaryElementModel; reset=true)
    @error "prepare_wake_on_all! not defined for wake $(typeof(wake)) and model $(typeof(model))"
end

"""
    wake_on_all!(model)

Computes the influence of wake elements on all aerodynamic entities in `model`.

# Arguments

* `model::AbstractModel`: the model for which the wake-on-all calculation is to be performed.

"""
function wake_on_all!(model)
    @error "wake_on_all! not defined for $(typeof(model))"
end

"""
    solve_boundary!(model, dt)

Solves for boundary element strengths for the `model`.

# Arguments

* `model::AbstractBoundaryElementModel`: the model whose boundary elements are to be solved
* `dt::Float64`: the timestep over which the model solution will be used; helpful for determining where to shed new wake elements
* `dt::Float64`: the timestep over which the model solution will be used; helpful for estimating parameters like ''\\frac{d\\Gamma}{dt}''

"""
function solve_boundary!(model::AbstractBoundaryElementModel, dt)
    @error "solve_boundary! not defined for $(typeof(model))"
end

"""
    prepare_surface_on_all!(model)

Prepare `model` to calculate the influence of the `surfaces` in `model` on all aerodynamic entities.

# Arguments

* `model::AbstractBoundaryElementModel`: the model for whom the surface-on-all computation is to be performed

"""
function prepare_surface_on_all!(model::AbstractBoundaryElementModel)
    @error "prepare_surface_on_all! not defined for $(typeof(model))"
end

"""
    surface_on_all!(model)

Calculate the influence of the `surfaces` in `model` on all aerodynamic entities.

# Arguments

* `model::AbstractBoundaryElementModel`: the model for whom the surface-on-all computation is to be performed

"""
function surface_on_all!(model::AbstractBoundaryElementModel)
    @error "surface_on_all! not defined for $(typeof(model))"
end

"""
    shed_wake!(model, dt)

Sheds new wake elements.

# Arguements

* `model::AbstractBoundaryElementModel`: the `model` to receive new wake elements
* `dt::Float64`: the current timestep duration

"""
function shed_wake!(model::AbstractBoundaryElementModel)
    @error "shed_wake! not defined or $(typeof(model))"
end

"""
    forces!(model)

Calculates the aerodynamic forces acting on `model`.

# Arguments

* `model::AbstractBoundaryElementModel`: the `model` for which forces will be calculated

"""
function forces!(model::AbstractBoundaryElementModel)
    @error "forces! not defined for $(typeof(model))"
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

    #--- prepare wake-on-all calculation velocity ---#
    DEBUG[] && println("pre-prepare-wake-on-all")
    prepare_wake_on_all!(model, dt)

    #--- wake-on-all induced velocity ---#
    DEBUG[] && println("pre-wake-on-all")
    wake_on_all!(model)

    #--- solve boundary elements ---#
    DEBUG[] && println("pre-solve-boundary")
    solve_boundary!(model, dt)

    #--- prepare surface-on-all induced velocity ---#
    DEBUG[] && println("pre-prep-surface-on-all")
    all, surface, fmm_options = prepare_surface_on_all!(model)

    #--- surface-on-all induced velocity ---#
    DEBUG[] && println("pre-surface-on-all")
    surface_on_all!(model)

    #--- shed wake for this timestep ---#
    DEBUG[] && println("pre-shedwake!")
    shed_wake!(model, dt)

    #--- compute forces ---#
    DEBUG[] && println("pre forces!")
    forces!(model)
    DEBUG[] && println("post forces!")

    return nothing
end

#------- Vortex Particle Model -------#

"""
    VortexParticleModel <: AbstractWake <: AbstractModel

Model used to describe vortex particle wakes.

# Fields

* `particle_field::FLOWVPM.ParticleField`: the vortex particles and accompanying parameters describing the wake
* `new_particle_probes::FastMultipole.ProbeSystem`: probes placed at the new particle shed locations to inform their velocity and velocity gradient
* `shed_starting::Bool`: whether or not to shed the starting vortex
* `threshold_trailing_gamma_max::Float64`: particle strength threshold above which new trailing (shedding due to steady circulation) particle candidates are not added to the particle field
* `threshold_trailing_gamma_min::Float64`: particle strength threshold below which new trailing (shedding due to steady circulation) particle candidates are not added to the particle field
* `threshold_unsteady_gamma_max::Float64`: particle strength threshold above which new unsteady (shedding due to time rate of change of circulation) particle candidates are not added to the particle field
* `threshold_unsteady_gamma_min::Float64`: particle strength threshold below which new unsteady (shedding due to time rate of change of circulation) particle candidates are not added to the particle field
* `overlap_trailing::Float64`: used to set the smoothing radius of new trailing particles; 0.0 means radii should touch, and 1.0 means radii should coincide with adjacent particle centers
* `overlap_unsteady::Float64`: used to set the smoothing radius of new unsteady particles; 0.0 means radii should touch, and 1.0 means radii should coincide with adjacent particle centers
* `p_per_step_trailing::Int64`: number of trailing particles shed each timestep
* `p_per_step_unsteady::Int64`: number of unsteady particles shed each timestep

"""
struct VortexParticleModel{TF,F,V,TUinf,S,Tkernel,TUJ,Tintegration,TRelaxation} <: AbstractWake{TF,false}
    # wake particles
    particle_field::vpm.ParticleField{TF, F, V, TUinf, S, Tkernel, TUJ, Tintegration, TRelaxation}

    # where and how to shed new particles
    new_particle_probes::fmm.ProbeSystem{TF, Nothing, Nothing, Vector{SVector{3,TF}}, Vector{SMatrix{3,3,TF,9}}}

    # new particle shedding parameters
    shed_starting::Bool
    threshold_trailing_gamma_max::Float64
    threshold_trailing_gamma_min::Float64
    threshold_unsteady_gamma_max::Float64
    threshold_unsteady_gamma_min::Float64
    overlap_trailing::Float64
    overlap_unsteady::Float64
    p_per_step_trailing::Int64
    p_per_step_unsteady::Int64
end

function VortexParticleModel(particle_field, new_particle_probes;
        shed_starting=true,
        threshold_trailing_gamma_max=Inf,
        threshold_trailing_gamma_min=0.0,
        threshold_unsteady_gamma_max=Inf,
        threshold_unsteady_gamma_min=0.0,
        overlap_trailing=0.3,
        overlap_unsteady=0.3,
        p_per_step_trailing=1,
        p_per_step_unsteady=1
    )
    return VortexParticleModel(particle_field, new_particle_probes,
        shed_starting, threshold_trailing_gamma_max, threshold_trailing_gamma_min,
        threshold_unsteady_gamma_max, threshold_unsteady_gamma_min,
        overlap_trailing, overlap_unsteady,
        p_per_step_trailing, p_per_step_unsteady)
end

function reset!(wake::VortexParticleModel)
    vpm._reset_particles(wake.particle_field)
    vpm.isSFSenabled(wake.particle_field.SFS) && vpm._reset_particles_sfs(wake.particle_field)
    fmm.reset!(wake.new_particle_probes)
end

struct RigidWake{TF} <: AbstractWake{TF,true} end

#------- Vortex Lattice Model -------#

"""
    VortexLatticeModel <: AbstractBoundaryElementModel <: AbstractModel

Model defining vortex lattice surfaces.

# Fields

* `vlm_system::VortexLattice.System`: the vortex lattice system
* `surface_forces::Vector{SVector{3,Float64}}`: the `i` th element is the force experienced by the `i` th surface in `vlm_system`
* `surface_moments::Vector{SVector{3,Float64}}`: the `i` th element is the moment experienced by the `i` th surface in `vlm_system`
* `wake <: AbstractWake`: model used to describe the wake
* `toggle_wake::Vector{Bool}`: determines whether the corresponding surfaces in `vlm_system` should shed wakes
* `eta_wake::Float64`: when using an unsteady wake, new wake filaments are shed with a length of `V * dt * deta_wake[i_surface]`
* `expansion_order::Int64`: the expansion order of the FMM
* `leaf_size::Int`: maximum number of bodies per leaf in the FMM
* `multipole_threshold::Float64`: value between 0 and 1 determines when multipole expansions should be used; 0 means they are never used, and 1 means they are always used

"""
struct VortexLatticeModel{TF,Quasisteady,TW<:AbstractWake{TF,Quasisteady},FMM} <: AbstractBoundaryElementModel{TF,Quasisteady,FMM}
    # vlm
    vlm_system::vlm.System{TF}
    surface_forces::Vector{SVector{3,TF}}
    surface_moments::Vector{SVector{3,TF}}

    # wake
    wake::TW
    toggle_wake::Vector{Bool} # which surfaces should shed wake particles
    eta_wake::TF # shed the next wake filament V * dt * eta_wake[i_surface]

    # fmm parameters
    expansion_order::Int
    leaf_size::Int
    multipole_threshold::Float64
end

#--- constructor ---#

function VortexLatticeModel(system::vlm.System{TF};
        # type parameters
        Quasisteady=false,
        FMM=true,

        # dynamics options
        center_of_mass = nothing, # if nothing, use system.reference.r

        # wake shedding options
        toggle_wake = [true for _ in 1:length(system.surfaces)], # indicates whether each surface should shed a wake
        eta_wake = 0.3, # distance from the trailing edge to place new wake elements in terms of the local velocity induced at the trailing edge during the last timestep (if `wake` is an unsteady wake)

        # wake object
        wake = nothing, # if nothing, a VortexParticleModel is generated using the following options
        max_timesteps = 0, # used to determine how many particles to preallocate

        # vpm options; used if `isnothing(wake)` to construct a `::VortexParticleModel`
        vpm_options = (), # keyword arguments supplied to the `vpm.ParticleField` constructor
        shed_starting=true,
        threshold_trailing_gamma_max=Inf,
        threshold_trailing_gamma_min=0.0,
        threshold_unsteady_gamma_max=Inf,
        threshold_unsteady_gamma_min=0.0,
        overlap_trailing=0.3,
        overlap_unsteady=0.3,
        p_per_step_trailing=1,
        p_per_step_unsteady=1,

        # options for the surface-on-all call to the FMM
    	expansion_order = 4, # expansion order of multipole expansions
    	leaf_size = 50, # number of particles per multipole expansion
    	multipole_threshold = 0.4, # number between 0-1 serves as the accuracy-performance tradeoff dial for evaluating expansions on surfaces

        # additional options
        reset=true
    ) where TF

    # assertions
    for trailing_vortex in system.trailing_vortices
        @assert trailing_vortex == Quasisteady "semi-infinite vortex horseshoes " * (quasisteady ? "required" : "not supported") * " for `VortexLatticeModel{<:Any,<:Any,$(quasisteady)}`; set system.trailing_vortices .= $(quasisteady)"
    end

    # update fmm_probes
    surfaces = system.surfaces
    fmm_velocity_probes = system.fmm_velocity_probes
    n_control_points, n_surface_filaments, n_trailing_edge_corners = count_probes(surfaces)
    vlm.update_n_probes!(fmm_velocity_probes, n_control_points + n_surface_filaments + n_trailing_edge_corners)
    vlm.update_probes!(fmm_velocity_probes, surfaces, 0) # control points and filament centers
    update_probes_te!(fmm_velocity_probes, surfaces, n_control_points + n_surface_filaments) # trailing edge corners


    # surface forces/moments
    surface_forces = Vector{SVector{3,TF}}(undef,length(system.surfaces))
    surface_moments = Vector{SVector{3,TF}}(undef,length(system.surfaces))

    # wake
    if isnothing(wake)

        # new particle probes
        n_new_particles = 0
        for surface in system.surfaces
            ns = size(surface,2)
            n_new_particles += ns * p_per_step_unsteady # unsteady loading
            n_new_particles += (ns + 1) * p_per_step_trailing # trailing vortex
        end
        new_particle_probes = fmm.ProbeSystem(n_new_particles; velocity=true, velocity_gradient=true)

        # max number of particles
        max_particles = n_new_particles * max_timesteps
        particle_field = vpm.ParticleField(max_particles, TF; vpm_options...)

        # build vortex particle wake
        wake = VortexParticleModel(particle_field, new_particle_probes;
            shed_starting,
            threshold_trailing_gamma_max,
            threshold_trailing_gamma_min,
            threshold_unsteady_gamma_max,
            threshold_unsteady_gamma_min,
            overlap_trailing,
            overlap_unsteady,
            p_per_step_trailing,
            p_per_step_unsteady
        )

    end

    # dimensionalize and set center of mass
    center_of_mass = isnothing(center_of_mass) ? system.reference[].r : center_of_mass
    ref = system.reference[]
    system.reference[] = eltype(system.reference)(1.0, 1.0, 1.0, center_of_mass, 1.0)

    # construct model
    model = VortexLatticeModel{TF,Quasisteady,typeof(wake),FMM}(system, surface_forces, surface_moments, wake, toggle_wake, eta_wake, expansion_order, leaf_size, multipole_threshold)

    # zero fields
    reset && reset!(model)

    return model
end

function reset!(model::VortexLatticeModel)
    reset!(model.vlm_system)
    reset!(model.wake)
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

    fmm.reset!(system.fmm_velocity_probes)

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

    #--- count probes at trailing edge corners (to inform the wake shedding locations for next timestep) ---#

    n_trailing_edge_corners = 0
    for surface in surfaces
        ns = size(surface,2)
        n_trailing_edge_corners += ns+1
    end

    return n_control_points, n_surface_filaments, n_trailing_edge_corners
end

#--- solve! subfunctions ---#

function prepare_wake_on_all!(wake::VortexParticleModel, model::VortexLatticeModel; reset=true)

    #--- unpack model ---#

    new_particle_probes = wake.new_particle_probes
    surfaces = model.vlm_system.surfaces
    wake_shedding_locations = model.vlm_system.wake_shedding_locations

    #--- update probes ---#

    vlm.update_probes!(new_particle_probes, surfaces, wake_shedding_locations,
        wake.p_per_step_trailing, wake.p_per_step_unsteady, 0) # new particle locations

    #--- reset wake velocity ---#

    if reset
        fmm.reset!(wake.new_particle_probes)
        vpm._reset_particles(wake.particle_field)
        vpm.isSFSenabled(wake.particle_field.SFS) && vpm._reset_particles_sfs(wake.particle_field)
    end

    #--- prepare SFS contribution to the wake ---#

    wake.particle_field.SFS(wake, vpm.BeforeUJ())

end

function prepare_wake_on_all!(model::VortexLatticeModel, dt; reset=true)

    #--- unpack model ---#

    fmm_velocity_probes = model.vlm_system.fmm_velocity_probes
    surfaces = model.vlm_system.surfaces
    wakes = model.vlm_system.wakes
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    ref = model.vlm_system.reference[]
    fs = model.vlm_system.freestream[]
    Vte = model.vlm_system.Vte
    nwake = model.vlm_system.nwake
    eta = model.eta_wake

    #--- count probes at control points and the center of each bound vortex ---#

    n_control_points, n_surface_filaments, n_trailing_edge_corners = count_probes(surfaces)

    #--- update Vte to include the wake and surface influence from the last timestep ---#

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

    #--- prepare wake ---#

    particle_field = prepare_wake_on_all!(model.wake, model)

    #--- reset vlm velocity ---#

    if reset
        reset!(model.vlm_system)
        fmm.reset!(fmm_velocity_probes)
    end

end

function wake_on_all!(model::VortexLatticeModel{<:Any,<:Any,<:VortexParticleModel,FMM}) where FMM

    #--- all elements ---#

    all = (model.vlm_system.fmm_velocity_probes, model.wake.new_particle_probes, model.wake.particle_field)

    #--- solve N-body problem ---#

    if FMM

        fmm_options = (expansion_order=model.expansion_order,
                       leaf_size_source=model.wake.particle_field.fmm.ncrit,
                       leaf_size_target=model.leaf_size,
                       multipole_threshold=model.wake.particle_field.fmm.theta
                      )

        scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)
        fmm.fmm!(all, model.wake.particle_field; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    else

        fmm.direct!(all, model.wake.particle_field; scalar_potential, vector_potential, velocity, velocity_gradient)

    end

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

    if true in symmetric
        @warn "the FMM doesn't support symmetric kernels;
        set `model.vlm_system.symmetric[] .= false` before using the FMM"
    end
    if true in trailing_vortices
        @warn "semiinfinite horseshoes are being used for surfaces
            $(([i for i in 1:length(trailing_vortices) if trailing_vortices[i]]))"
    end

    #--- update influence matrix ---#
    vlm.influence_coefficients!(AIC, surfaces;
            symmetric = symmetric,
            wake_shedding_locations = wake_shedding_locations,
            surface_id = surface_id, # in vanilla VortexLattice, determines when to use the
                                     # finite core model (we always use the finite core model
                                     # in FLOWUnsteady so this parameter is irrelevant)
            trailing_vortices = trailing_vortices, # ::Vector{Bool} tells which surfaces should use semiinfinite horseshoes
            xhat = xhat                            # only used if semi-infinite horseshoes are used
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

    dΓdt .= -Γ

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
    fmm_velocity_probes = model.vlm_system.fmm_velocity_probes

    #--- update fmm_panels ---#

    vlm.update_fmm_panels!(fmm_panels, surfaces, wake_shedding_locations, Γ)

end

function surface_on_all!(model::VortexLatticeModel{<:Any,<:Any,<:VortexParticleModel,FMM}) where FMM

    #--- all elements ---#

    all = (model.vlm_system.fmm_velocity_probes, model.wake.new_particle_probes, model.wake.particle_field)

    #--- surface ---#

    surface = model.vlm_system

    #--- surface-on-all computation ---#

    if FMM

        fmm_options = (expansion_order=model.expansion_order,
                       leaf_size_source=model.leaf_size,
                       leaf_size_target=model.leaf_size,
                       multipole_threshold=model.multipole_threshold
                      )

        scalar_potential, vector_potential, velocity, velocity_gradient = requires_potential(all)
        fmm.fmm!(all, surface; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    else

        fmm.direct!(all, surface; scalar_potential, vector_potential, velocity, velocity_gradient, fmm_options...)

    end

end

function shed_wake!(model::VortexLatticeModel{TF,<:Any,<:VortexParticleModel,<:Any}, dt) where TF

    #--- unpack model ---#

    toggle_wake = model.toggle_wake
    new_particle_probes = model.wake.new_particle_probes
    shed_starting = model.wake.shed_starting
    threshold_trailing_gamma_max = model.wake.threshold_trailing_gamma_max
    threshold_trailing_gamma_min = model.wake.threshold_trailing_gamma_min
    threshold_unsteady_gamma_max = model.wake.threshold_unsteady_gamma_max
    threshold_unsteady_gamma_min = model.wake.threshold_unsteady_gamma_min
    overlap_trailing = model.wake.overlap_trailing
    overlap_unsteady = model.wake.overlap_unsteady
    p_per_step_trailing = model.wake.p_per_step_trailing
    p_per_step_unsteady = model.wake.p_per_step_unsteady
    surfaces = model.vlm_system.surfaces
    wake_shedding_locations = model.vlm_system.wake_shedding_locations
    Γ = model.vlm_system.Γ
    dΓdt = model.vlm_system.dΓdt
    wake = model.wake

    trailing_args = (overlap_trailing, threshold_trailing_gamma_max, threshold_trailing_gamma_min)
    unsteady_args = (overlap_unsteady, threshold_unsteady_gamma_max, threshold_unsteady_gamma_min)

    #--- shed particles ---#

    i_probe = 0
    iΓ = 0
    for (surface, wsl, toggle) in zip(surfaces, wake_shedding_locations, toggle_wake)
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
                i_probe = add_line!(wake.particle_field, new_particle_probes, wsl[i_te], panel.rbl, i_probe, p_per_step_trailing, Γ_magnitude - Γ_last, trailing_args...)

                # unsteady loading
                i_probe = add_line!(wake.particle_field, new_particle_probes, wsl[i_te+1], wsl[i_te], i_probe, p_per_step_unsteady, dΓdt_magnitude * dt, unsteady_args...)

                # recurse
                Γ_last = Γ_magnitude
            end

            # rightmost trailing vortex
            panel = surface[nc,end]
            i_probe = add_line!(wake.particle_field, new_particle_probes, panel.rbr, wsl[end], i_probe, p_per_step_trailing, Γ_last, trailing_args...)

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
    add_line!(wake, probes, x1, x2, i_last, n_particles, Γ_magnitude, overlap, gamma_max, gamma_min)

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
* `gamma_max::Float64`: if the requested particle has a strength magnitude greater than or equal to `gamma_max`, it is omitted
* `gamma_min::Float64`: if the requested particle has a strength magnitude less than or equal to `gamma_min`, it is omitted

# Output

* `i_last::Int`: the index of the last probe used

"""
@inline function add_line!(wake::vpm.ParticleField, probes::fmm.ProbeSystem, x1, x2, i_last::Int, n_particles::Int, Γ_magnitude, overlap, gamma_max, gamma_min)

    # strength and radius
    Γ = x2 - x1 # strength is aligned with the line
    Δx = norm(Γ) / n_particles # distance spanned by a single particle
    σ = Δx / 2 * (1 + overlap) # divide by 2 to get a radius rather than a diameter
    Γ *= Γ_magnitude / n_particles # divide strength by the number of particles in the line

    # shed particles
    i_particle = 0
    for _ in 1:n_particles
        i_particle += 1
        if gamma_min < Γ_magnitude < gamma_max
            X = probes.position[i_particle + i_last]

            # ensure probes are colinear
            @assert isapprox(abs(dot(Γ, X-x1)), norm(Γ) * norm(X-x1); atol=1e-12) "probes used for particle shedding are not colinear; x1=$x1, x2=$x2, X=$X, Γ_magnitude=$Γ_magnitude, n_particles=$n_particles, overlap=$overlap"

            # add particle
            vpm.add_particle(wake, X, Γ, σ; vol=0, circulation=1, C=0)
        end
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
    apply!(wake, freestream)

Apply `freestream` properties to the `wake`.

# Arguments

* `wake::VortexParticleModel`: the wake to which the freestream is to be applied
* `freestream::SimpleFreestream`: the freestream

"""
function apply!(wake::VortexParticleModel, freestream::SimpleFreestream)
    # apply freestream to wake particles
    apply!(wake.particle_field, freestream)

    # apply freestream velocity to new_particle_probes
    for i in eachindex(wake.new_particle_probes.velocity)
        wake.new_particle_probes.velocity[i] += freestream.velocity
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

function transform!(model::VortexLatticeModel, dt, i_step::Int)
    transform!(model.wake, dt, i_step)
end

function transform!(wake::VortexParticleModel, dt, i_step::Int)
    transform!(wake.particle_field, dt, i_step)
end

function transform!(wake::vpm.ParticleField, dt, i_step::Int)

    # determine whether relaxation should be performed
    relax = wake.relaxation != vpm.relaxation_none &&
        wake.relaxation.nsteps_relax >= 1 &&
        i_step > 0 && (i_step % wake.relaxation.nsteps_relax == 0)

    # convect wake
    vpm._euler(wake, dt; relax=false)

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

function visualize(system::vlm.System, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default", path="./")
    # file name
    name_suffix = "_vlm"
    file_name = name_prefix*name_suffix

    # save paraview collection file
    !isnothing(t) && (pvd = WriteVTK.paraview_collection(joinpath(path,file_name), append=true))

    # add timestep to file name
    !isnothing(i) && (file_name *= "-step$i")
    kwargs = isnothing(t) ? () : ((metadata = Dict{String,Float64}("TimeValue" => t)),)

    # write files
    vlm.write_vtk(joinpath(path,file_name), system.surfaces, system.properties; symmetric = fill(false, length(system.surfaces)), trailing_vortices=false, pvd, time=t, kwargs...)
    !isnothing(i) && WriteVTK.vtk_save(pvd)
end

function visualize(wake::VortexParticleModel, i=nothing, t=nothing; name_prefix="default", path="")
    visualize(wake.particle_field, i, t; name_prefix, path)
end

function visualize(wake::vpm.ParticleField, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default", path="./")
    name_suffix = "_particlefield"
    kwargs = isnothing(i) ? () : (add_num=true, num=i)
    !isnothing(t) && (kwargs = (kwargs..., overwrite_time=t))
    vpm.save(wake, name_prefix * name_suffix; path, kwargs...)
end

function visualize(model::VortexLatticeModel, i::Union{Nothing,Int}=nothing, t::Union{Nothing,Float64}=nothing; name_prefix="default", path="./")
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
