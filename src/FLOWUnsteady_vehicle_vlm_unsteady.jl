#=##############################################################################
# DESCRIPTION
    Vehicle with vortex-lattice method (VLM) and VPM-based propeller models
    shedding VPM wakes.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


################################################################################
# UNSTEADY VLM VEHICLE TYPE
################################################################################
"""
    UVLMVehicle{N, M, R}(system; tilting_systems, rotors_systems,
                                            vlm_system, wake_system, grids)

Type handling all geometries and subsystems that define a vehicle made
out of FLOWVLM components (Wing, WingSystem, Rotor).

# ARGUMENTS
* `system::FLOWVLM.WingSystem`:        System of all FLOWVLM objects. This system
                                    is considered as the entire vehicle. Not all
                                    components in this system will be solved,
                                    but they will all be rotated and translated
                                    according to the maneuver.
# OPTIONAL ARGUMENTS
* `tilting_systems::NTuple{N, FLOWVLM.WingSystem}`:   Tuple of all FLOWVLM
                                    tilting objects, where `tilting_systems[i]`
                                    contains the i-th FLOWVLM system of lifting
                                    surfaces and rotors that tilt together.
* `rotors_systems::NTuple{M, Array{vlm.Rotor}}`:   Tuple of groups of Rotors
                                    that share a common RPM.
* `vlm_system::FLOWVLM.WingSystem`:    System of all FLOWVLM objects to be solved
                                    through the VLM solver.
* `wake_system::FLOWVLM.WingSystem`:   System of all FLOWVLM objects that will
                                    shed a VPM wake.
* `grids::Array{gt.GridTypes}`:         Array of grids that will be translated and
                                    rotated along with `system`.

# State variables
* `V::Vector`                   : Current vehicle velocity
* `W::Vector`                   : Current vehicle angular velocity
* `prev_data::Array{Any}`       : Information about previous step
* `grid_O::Vector{Vector}`       : Origin of every grid
"""
struct UVLMVehicle{N, M, R} <: AbstractVLMVehicle{N, M, R}

    # Required inputs
    system::vlm.WingSystem

    # Optional inputs
    tilting_systems::NTuple{N, vlm.WingSystem}
    rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}
    vlm_system::vlm.WingSystem
    wake_system::vlm.WingSystem
    grids::Array{gt.GridTypes, 1}

    # Internal properties
    V::Array{R, 1}                          # Current vehicle velocity
    W::Array{R, 1}                          # Current vehicle angular velocity
    prev_data::Array{Any, 1}                # Information about previous step
    grid_O::Array{Array{R, 1}, 1}           # Origin of every grid


    UVLMVehicle{N, M, R}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem(),
                    grids=Array{gt.GridTypes, 1}(),
                    V=zeros(3), W=zeros(3),
                    prev_data=[deepcopy(vlm_system), deepcopy(wake_system),
                                                    deepcopy(rotor_systems)],
                    grid_O=Array{Array{Float64, 1}, 1}(),
                ) where {N, M, R} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system,
                    grids,
                    V, W,
                    prev_data,
                    grid_O,
                )
end

"""
    UVLMVehicle(system; optargs...)

Constructor with implicit `N`, `M`, and `R` parameters.
"""
UVLMVehicle(system::vlm.WingSystem;
        V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        grids=Array{gt.GridTypes, 1}(),
        optargs...
        ) where {N, M, R} = UVLMVehicle{N, M, R}( system;
                                V=V, W=W,
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                grids=Array{gt.GridTypes, 1}(grids),
                                grid_O=[zeros(R, 3) for i in 1:length(grids)],
                                optargs...)

"""Alias for [`FLOWUnsteady.UVLMVehicle`](@ref)"""
VLMVehicle = UVLMVehicle

##### FUNCTIONS  ###############################################################
function shed_wake(self::VLMVehicle, Vinf::Function,
                            pfield::vpm.ParticleField, dt::Real, nt::Int; t=0.0,
                            unsteady_shedcrit=-1.0,
                            shed_starting=false,
                            p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing,
                            omit_shedding=[],
                            shed_boundarylayer=false,
                            prescribed_Cd=nothing, dipole_d=0.0,
                            )
    if nt!=0

        if shed_boundarylayer
            if dipole_d == 0
                error("Boundary layer shedding requested but no `d` was provided!")
            end

            # Shed only boundary layer (dragging line) particles
            VLM2VPM_draggingline(self.wake_system, _get_prev_wake_system(self),
                                    pfield, dt, Vinf,
                                    dipole_d;
                                    t=t,
                                    prescribed_Cd=prescribed_Cd,
                                    p_per_step=p_per_step,
                                    sigmafactor=sigmafactor,
                                    overwrite_sigma=overwrite_sigma,
                                )
        else
            # Shed trailing-circulation particles
            VLM2VPM(self.wake_system, pfield, dt, Vinf; t=t,
                        prev_system=_get_prev_wake_system(self),
                        unsteady_shedcrit=unsteady_shedcrit,
                        shed_starting=shed_starting && nt==1,
                        p_per_step=p_per_step, sigmafactor=sigmafactor,
                        overwrite_sigma=overwrite_sigma, check=false,
                        omit_shedding=omit_shedding)
        end
    end
end

"""
Uniform vortex-sheet distribution for actuator surface model.
See [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_uniform(x) = 0 <= x <= 1 ? 1 : 0   # Uniform distribution


"""
Linear vortex-sheet distribution for actuator surface model.
See [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_linear(x) = x < 0 ? 0 :                           # Piece-wise linear distribution centered at quarter-chord
           x < 0.25 ? 0.4 + 3.04 * x/0.25 :         # as given by Kim 2015, "Improved actuator surface method for wind turbine application."
           x < 0.50 ? 3.44 - 3.2 * (x-0.25)/0.25 :
           x <=1.00 ? 0.24 - 0.24 * (x-0.5)/0.5  :
           0


"""
Pressure-like vortex-sheet distribution for actuator surface model.
See [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_pressure(x) = x <= 0 ? 0 :         # Pressure-like distribution: peaking by the LE with aerodynamic center at to quarter-chord.
                x <= 1 ? (1-exp(-(x/0.02)^3))/(4*pi*x) / 0.3266200204514099 :
                0

"""
Alias for [`FLOWUnsteady.g_linear`](@ref).
"""
const g_piecewiselinear = g_linear

function generate_static_particle_fun(pfield::vpm.ParticleField, pfield_static::vpm.ParticleField,
                                        self::VLMVehicle,
                                        sigma_vlm::Real, sigma_rotor::Real;
                                        vlm_vortexsheet=false,
                                        vlm_vortexsheet_overlap=2.125,
                                        vlm_vortexsheet_distribution=g_pressure,
                                        vlm_vortexsheet_sigma_tbv=nothing,
                                        save_path=nothing, run_name="", suff="_staticpfield",
                                        nsteps_save=1)

    if sigma_vlm<=0
        error("Invalid VLM smoothing radius $sigma_vlm.")
    elseif sigma_rotor<=0
        error("Invalid rotor smoothing radius $sigma_rotor.")
    end

    flag = save_path!=nothing

    function static_particles_function(pfield, args...)

        # Particles from vlm system
        _static_particles(pfield, self.vlm_system, sigma_vlm;
                                vortexsheet=vlm_vortexsheet,
                                vortexsheet_overlap=vlm_vortexsheet_overlap,
                                vortexsheet_distribution=vlm_vortexsheet_distribution,
                                vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv)
        if flag
            _static_particles(pfield_static, self.vlm_system, sigma_vlm;
                                vortexsheet=vlm_vortexsheet,
                                vortexsheet_overlap=vlm_vortexsheet_overlap,
                                vortexsheet_distribution=vlm_vortexsheet_distribution,
                                vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv)
        end

        # Particles from rotor systems
        for rotors in self.rotor_systems
            for rotor in rotors
                _static_particles(pfield, rotor, sigma_rotor)
                if flag; _static_particles(pfield_static, rotor, sigma_rotor); end;
            end
        end

        # Save vtk with static particles
        if flag
            if pfield_static.nt%nsteps_save==0
                vpm.save(pfield_static, run_name*suff; path=save_path,
                                    add_num=true, overwrite_time=nothing)
            end
            pfield_static.nt += 1
            pfield_static.t += 1
            for pi in vpm.get_np(pfield_static):-1:1
                vpm.remove_particle(pfield_static, pi)
            end
        end

        return nothing
    end

    return static_particles_function
end

save_vtk(self::VLMVehicle, args...;
                        optargs...) = save_vtk_base(self, args...; optargs...)

##### INTERNAL FUNCTIONS  ######################################################

function _static_particles(pfield::vpm.ParticleField,
                            system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
                            sigma::Real;
                            sigma_vpm=nothing,
                            vortexsheet::Bool=false,
                            vortexsheet_overlap::Real=2.125,
                            vortexsheet_distribution::Function=g_uniform,
                            vortexsheet_sigma_tbv=nothing,
                            vortices=1:3, # Bound vortices to add (1==AB, 2==ApA, 3==BBp)
                            )

    X, Gamma, dl = (zeros(3) for i in 1:3)

    # Adds a particle for every bound vortex of the VLM
    for i in 1:vlm.get_m(system)
        (Ap, A, B, Bp, _, _, _, gamma) = vlm.getHorseshoe(system, i)
        for (j, (x1, x2)) in enumerate(((A,B), (Ap,A), (B,Bp)))

            # Mid-point along bound vortex
            X .= x1
            X .+= x2
            X ./=2

            # Vortex strength
            Gamma .= x2
            Gamma .-= x1
            Gamma .*= gamma

            if !(j in vortices)            # Case that bound vortex is not added
                nothing

            elseif !vortexsheet            # Case of no vortex sheet

                vpm.add_particle(pfield, X, Gamma, sigma;
                                    vol=0, circulation=abs(gamma), static=true,
                                    # index=i
                                    )

            elseif j==2 || j==3            # Case of trailing vortex with vortex sheet

                # If sigma_tbv is given, it discretizes trailing vortices with
                # the same sigma than the wake
                this_sigma = vortexsheet_sigma_tbv != nothing ? vortexsheet_sigma_tbv : sigma

                # Length of bound vortex
                dl .= x2
                dl .-= x1
                dl ./= (1-vlm.pn)

                # Position of LE
                if j==2
                    X .= x1
                    X .+= dl
                else
                    X .= x2
                    X .-= dl
                end

                l = sqrt(dl[1]^2 + dl[2]^2 + dl[3]^2)   # Length (TE to lifting line)
                                                        # Number of particles to use
                np           = ceil(Int, l / (this_sigma/vortexsheet_overlap) )
                dl ./= np                   # Step length

                # Calculate normalization of distribution
                gnorm = 0
                for ni in 1:np
                    gnorm += vortexsheet_distribution( (ni-1)/(np-1) )
                end

                # Start at LE and shift by half a step to center particles
                dl ./= 2
                if j==2
                    X .+= dl
                else
                    X .-= dl
                end
                dl .*= 2

                circulation = 0             # Cumulative circulation

                # Add particles
                for ni in 1:np
                    if j==2
                        X .-= dl
                    else
                        X .+= dl
                    end

                    # Spread circulation among sheet particles
                    circulation += gamma * vortexsheet_distribution( (ni-1)/(np-1) ) / gnorm

                    Gamma .= dl
                    Gamma .*= circulation

                    vpm.add_particle(pfield, X, Gamma, this_sigma;
                                        vol=0, circulation=abs(circulation), static=true,
                                        index=i) # NOTE: Here I'm using the index to indicate
                                                 # the horseshoe that this particle belongs to
                end

            else                           # Case of spreading lifting vortex as a vortex sheet

                # Take the average between both bound vortices and extend
                # that length from TE to LE
                dl .= Ap
                dl .-= A
                dl .+= Bp
                dl .-= B
                dl ./= 2*(1-vlm.pn)

                # Move X from mid-point of bound vortex to mid-point of LE
                dl .*= -vlm.pn
                X .+= dl
                dl ./= -vlm.pn

                l = sqrt(dl[1]^2 + dl[2]^2 + dl[3]^2)   # Sheet width (TE to LE)
                                            # Number of particles to use
                np           = ceil(Int, l / (sigma/vortexsheet_overlap) )
                dl ./= np                   # Step width

                # Shift position by one step as preparation to adding particles
                X .-= dl

                # Calculate normalization of distribution
                gnorm = 0
                for ni in 1:np
                    gnorm += vortexsheet_distribution( (ni-1)/(np-1) )
                end

                # Add particles
                for ni in 1:np
                    X .+= dl

                    # Spread circulation among sheet particles
                    circulation = gamma * vortexsheet_distribution( (ni-1)/(np-1) ) / gnorm

                    Gamma .= x2
                    Gamma .-= x1
                    Gamma .*= circulation

                    vpm.add_particle(pfield, X, Gamma, sigma;
                                        vol=0, circulation=abs(circulation), static=true,
                                        index=i) # NOTE: Here I'm using the index to indicate
                                                 # the horseshoe that this particle belongs to
                end

            end

        end
    end
end
##### END OF VEHICLE ###########################################################
