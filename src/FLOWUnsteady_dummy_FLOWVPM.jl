module FLOWVPM

import Dates
import GeometricTools: create_path

const RealFMM = Float64

# Available Kernels
const kernel_singular = (args...)->nothing
const kernel_gaussian = (args...)->nothing
const kernel_gaussianerf = (args...)->nothing
const kernel_winckelmans = (args...)->nothing
const singular = kernel_singular
const gaussian = kernel_gaussian
const gaussianerf = kernel_gaussianerf
const winckelmans = kernel_winckelmans

mutable struct FMM
  # Optional user inputs
  p::Int32                        # Multipole expansion order
  ncrit::Int32                    # Max number of particles per leaf
  theta::RealFMM                  # Neighborhood criterion
  phi::RealFMM                    # Regularizing neighborhood criterion

  FMM(; p=4, ncrit=50, theta=0.4, phi=1/3) = new(p, ncrit, theta, phi)
end


"""
    `ViscousScheme{R}`

Type declaring viscous scheme.

Implementations must have the following properties:
    * `nu::R`                   : Kinematic viscosity.
"""
abstract type ViscousScheme{R} end

"""
Implementation of viscous diffusion scheme that gets called in the inner loop
of the time integration scheme at each time step.
"""
function viscousdiffusion(pfield, scheme::ViscousScheme, dt; optargs...)
    error("Viscous diffusion scheme has not been implemented yet!")
end

viscousdiffusion(pfield, dt; optargs...
                    ) = viscousdiffusion(pfield, pfield.viscous, dt; optargs...)

zeta_direct(args...) = nothing
zeta_fmm(args...) = nothing
##### END OF ABSTRACT VISCOUS SCHEME ###########################################

################################################################################
# INVISCID SCHEME TYPE
################################################################################
struct Inviscid{R} <: ViscousScheme{R}
    nu::R                                 # Kinematic viscosity
    Inviscid{R}(; nu=zero(R)) where {R} = new(nu)
end

Inviscid() = Inviscid{RealFMM}()

"""
    `isinviscid(scheme::ViscousScheme)`

Returns true if viscous scheme is inviscid.
"""
isinviscid(scheme::ViscousScheme) = typeof(scheme).name == Inviscid.body.name

viscousdiffusion(pfield, scheme::Inviscid, dt; optargs...) = nothing
##### END OF INVISCID SCHEME ###################################################


################################################################################
# CORE SPEADING SCHEME TYPE
################################################################################
mutable struct CoreSpreading{R} <: ViscousScheme{R}
    # User inputs
    nu::R                                 # Kinematic viscosity
    sgm0::R                               # Core size after reset
    zeta::Function                        # Basis function evaluation method

    # Optional inputs
    beta::R                               # Maximum core size growth σ/σ_0
    itmax::Int                            # Maximum number of RBF iterations
    tol::R                                # RBF interpolation tolerance
    iterror::Bool                         # Throw error if RBF didn't converge
    verbose::Bool                         # Verbose on RBF interpolation
    v_lvl::Int                            # Verbose printing tab level
    debug::Bool                           # Print verbose for debugging

    # Internal properties
    t_sgm::R                              # Time since last core size reset
    rbf::Function                         # RBF function
    rr0s::Array{R, 1}                     # Initial field residuals
    rrs::Array{R, 1}                      # Current field residuals
    prev_rrs::Array{R, 1}                 # Previous field residuals
    pAps::Array{R, 1}                     # pAp product
    alphas::Array{R, 1}                   # Alpha coefficients
    betas::Array{R, 1}                    # Beta coefficients
    flags::Array{Bool, 1}                 # Convergence flags

    CoreSpreading{R}(
                        nu, sgm0, zeta;
                        beta=R(1.5),
                        itmax=R(15), tol=R(1e-3),
                        iterror=true, verbose=false, v_lvl=2, debug=false,
                        t_sgm=R(0.0),
                        rbf=rbf_conjugategradient,
                        rr0s=zeros(R, 3), rrs=zeros(R, 3), prev_rrs=zeros(R, 3),
                        pAps=zeros(R, 3), alphas=zeros(R, 3), betas=zeros(R, 3),
                        flags=zeros(Bool, 3)
                    ) where {R} = new(
                        nu, sgm0, zeta,
                        beta,
                        itmax, tol,
                        iterror, verbose, v_lvl, debug,
                        t_sgm,
                        rbf,
                        rr0s, rrs, prev_rrs,
                        pAps, alphas, betas,
                        flags
                    )
end

CoreSpreading(nu, sgm0, args...; optargs...
                    ) = CoreSpreading{RealFMM}(RealFMM(nu), RealFMM(sgm0), args...; optargs...)

"""
   `iscorespreading(scheme::ViscousScheme)`

Returns true if viscous scheme is core spreading.
"""
iscorespreading(scheme::ViscousScheme
                            ) = typeof(scheme).name == CoreSpreading.body.name


mutable struct ParticleField{T, V<:ViscousScheme}
    # User inputs
    maxparticles::Int                           # Maximum number of particles
    particles::Array{T, 1}                      # Array of particles
    bodies::Any                                 # ExaFMM array of bodies
    viscous::V                                  # Viscous scheme

    # Internal properties
    np::Int                                     # Number of particles in the field
    nt::Int                                     # Current time step number
    t::Float64                                  # Current time

    # Solver setting
    kernel                                      # Vortex particle kernel
    UJ::Function                                # Particle-to-particle calculation

    # Optional inputs
    Uinf::Function                              # Uniform freestream function Uinf(t)
    transposed::Bool                            # Transposed vortex stretch scheme
    relax::Bool                                 # Activates relaxation scheme
    rlxf::Float64                               # Relaxation factor (fraction of dt)
    integration::Function                       # Time integration scheme
    fmm::FMM                                    # Fast-multipole settings


    ParticleField{T, V}(
                        maxparticles,
                        particles, bodies, viscous;
                        np=0, nt=0, t=0.0,
                        kernel=gaussianerf,
                        UJ=UJ_fmm,
                        Uinf=t->zeros(3),
                        transposed=true,
                        relax=true, rlxf=0.3,
                        integration=rungekutta3,
                        fmm=FMM(),
                 ) where {T, V} = new(
                        maxparticles,
                        particles, bodies, viscous,
                        np, nt, t,
                        kernel,
                        UJ,
                        Uinf,
                        transposed,
                        relax, rlxf,
                        integration,
                        fmm,
                  )
end

function get_np(self::ParticleField)
  return self.np
end
function add_particle(args...; optargs...)
    nothing
end
function remove_particle(args...; optargs...)
    nothing
end
function UJ_direct(args...; optargs...)
    nothing
end
function UJ_fmm(args...; optargs...)
    nothing
end
function get_particleiterator(args...; optargs...)
    nothing
end
iterator(args...; optargs...) = get_particleiterator(args...; optargs...)
iterate(args...; optargs...) = get_particleiterator(args...; optargs...)
get_X(self::ParticleField, i::Int) = nothing
get_Gamma(self::ParticleField, i::Int) = nothing
get_sigma(self::ParticleField, i::Int) = nothing
get_U(self::ParticleField, i::Int) = nothing
get_W(self::ParticleField, i::Int) = nothing
isinviscid(self::ParticleField) = isinviscid(self.viscous)

"""
  `run_vpm!(pfield, dt, nsteps; runtime_function=nothing, save_path=nothing,
run_name="pfield", nsteps_save=1, verbose=true, prompt=true)`

Solves `nsteps` of the particle field with a time step of `dt`.

**Optional Arguments**
* `runtime_function::Function`   : Give it a function of the form
                            `myfun(pfield, t, dt)`. On each time step it
                            will call this function. Use this for adding
                            particles, deleting particles, etc.
* `static_particles_function::Function`   : Give it a function of the form
                            `myfun(pfield, t, dt)` to add static particles
                            representing solid boundaries to the solver. This
                            function is called at every time step right before
                            solving the governing equations, and any new
                            particles added by this function are immediately
                            removed.
* `nsteps_relax::Int`   : Relaxes the particle field every this many time steps.
* `save_path::String`   : Give it a string for saving VTKs of the particle
                            field. Creates the given path.
* `run_name::String`    : Name of output files.
* `nsteps_save::Int64`  : Saves vtks every this many time steps.
* `prompt::Bool`        : If `save_path` already exist, it will prompt the
                            user before overwritting the folder if true; it will
                            directly overwrite it if false.
* `verbose::Bool`       : Prints progress of the run to the terminal.
* `verbose_nsteps::Bool`: Number of time steps between verbose.
"""
function run_vpm!(pfield::ParticleField, dt::Real, nsteps::Int;
                      # RUNTIME OPTIONS
                      runtime_function::Function=(pfield, t, dt)->false,
                      # OUTPUT OPTIONS
                      save_path::Union{Nothing, String}=nothing,
                      create_savepath::Bool=true,
                      run_name::String="pfield",
                      save_code::String="",
                      prompt::Bool=true,
                      verbose::Bool=true, verbose_nsteps::Int=10, v_lvl::Int=0)

    # Creates save path and save code
    if save_path!=nothing && create_savepath
        create_path(save_path, prompt)
    end
    # Save code
    if save_path!=nothing && save_code!=""
        cp(save_code, save_path*"/"; force=true)
    end

    run_id = save_path!=nothing ? joinpath(save_path, run_name) : ""

    if verbose
        time_beg = Dates.DateTime(Dates.now())
        println("\t"^v_lvl*"*"^(73-8*v_lvl)*"\n"*"\t"^v_lvl*"START $run_id\t$time_beg\n"*
                "\t"^v_lvl*"*"^(73-8*v_lvl))
    end

    # RUN
    for i in 0:nsteps

        if verbose && i%verbose_nsteps==0
            println("\t"^(v_lvl+1)*"Time step $i out of $nsteps"*
            "\tParticles: $(get_np(pfield))")
        end

        # Time step
        if i!=0
            pfield.nt +=1
            pfield.t += dt
        end

        # Calls user-defined runtime function
        breakflag = runtime_function(pfield, pfield.t, dt)

        # User-indicated end of simulation
        if breakflag
            break
        end

    end

    if verbose
        time_end = Dates.DateTime(Dates.now())
        hrs,mins,secs = timeformat(time_beg, time_end)
        println("\t"^v_lvl*"*"^(73-8*v_lvl))
        println("\t"^v_lvl*"END $run_id\t$time_end")
        println("\t"^v_lvl*"*"^(73-8*v_lvl))
        println("\t"^v_lvl*"ELAPSED TIME: $hrs hours $mins minutes $secs seconds")
    end

    return nothing
end



function timeformat(time_beg, time_end)
    time_delta = Dates.value(time_end)-Dates.value(time_beg)
    hrs = Int(floor(time_delta/1000/60/60))
    mins = Int(floor((time_delta-hrs*60*60*1000)/1000/60))
    secs = Int(floor((time_delta-hrs*60*60*1000-mins*1000*60)/1000))
    return hrs,mins,secs
end

end # END OF MODULE
