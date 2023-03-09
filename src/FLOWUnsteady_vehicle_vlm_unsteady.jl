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
    UVehicle{N, M, R}(geometry; tilting_systems, rotors_systems,
                                        vlm_system, panel_system wake_system)

Geometry and subsystems that define a vehicle made out of FLOWVLM (Wing,
WingSystem, Rotor) and FLOWPanel components. These components are solved in
an unsteady scheme. `N` is the number of tilting system and `M` is the number
of rotor systems.

# ARGUMENTS
* `geometry::System`:               System of all components that make the
                                        geometry of the vehicle. Not all
                                        components in `geometry` will be solved,
                                        but they will all be rotated and
                                        translated according to the maneuver.

# OPTIONAL ARGUMENTS
* `tilting_systems::System{<:Any, N}`: System of all tilting components, where
                                        each component can be a subsystem of
                                        lifting surfaces and rotors that tilted
                                        together.
* `rotors_systems::System{Union{System, Rotor}, M}`: Rotor systems with rotors
                                        that share a common RPM.
* `vlm_system::FLOWVLM.WingSystem`: System of all FLOWVLM objects to be solved
                                        through the VLM solver.
* `panel_system::FLOWPanel.MultiBody`: System of all FLOWPanel objects to be
                                        solved through the panel solver.
* `wake_system::System`:            System of all components that will shed a
                                        VPM wake.

# State variables
* `V::Vector`                   : Current vehicle velocity
* `W::Vector`                   : Current vehicle angular velocity
* `previousstep::Array{Any}`    : Information about previous step
"""
struct UVLMVehicle{N, M, R} <: AbstractVLMVehicle{N, M, R}

    # Required inputs
    geometry::System

    # Optional inputs
    tilting_systems::System{<:Any, N}
    rotor_systems::System{<:Union{System{vlm.Rotor}, vlm.Rotor}, M}
    vlm_system::vlm.WingSystem
    panel_system::pnl.MultiBody
    wake_system::System

    # Internal properties
    V::Vector{R}                            # Current vehicle velocity
    W::Vector{R}                            # Current vehicle angular velocity
    previousstep::Array{Any}                # Information about previous step


    function UVLMVehicle{N, M, R}( geometry, tilting_systems, rotor_systems;
                                    vlm_system=vlm.WingSystem(),
                                    panel_system=pnl.MultiBody(),
                                    wake_system=nothing,
                                    V=zeros(3), W=zeros(3),
                                    previousstep=[],
                                    ) where {N, M, R}

        # Create wake system
        if wake_system==nothing

            flowvlm = System([vlm_system, rotor_systems])
            flowpanel = System([panel_system])

            _wake_system = generate_wake_system(; flowvlm_subsystem=flowvlm,
                                                  flowpanel_subsystem=flowpanel)
        else
            _wake_system = wake_system
        end

        return new(
                    geometry, tilting_systems, rotor_systems,
                    vlm_system, panel_system,
                    _wake_system,
                    V, W,
                    previousstep,
                )
    end
end

"""
    UVLMVehicle(system; optargs...)

Constructor with implicit `N`, `M`, and `R` parameters.
"""
function UVLMVehicle(system::System;
                        tilting_systems::System{<:Any, N}=System(),
                        rotor_systems::System{<:Any, M}=System(; C=vlm.Rotor),
                        V::Vector{R}=zeros(3), W::Vector{R}=zeros(3),
                        optargs...
                        ) where {N, M, R}

    return UVLMVehicle{N, M, R}(system, tilting_systems, rotor_systems;
                                                        V=V, W=W, optargs...)
end

"""Alias for [`FLOWUnsteady.UVLMVehicle`](@ref)"""
VLMVehicle = UVLMVehicle

function Base.deepcopy_internal(x::V, stackdict::IdDict) where V<:AbstractVLMVehicle
    if haskey(stackdict, x)
        return stackdict[x]
    end

    y = V(  Base.deepcopy_internal(x.geometry),
            Base.deepcopy_internal(x.tilting_systems),
            Base.deepcopy_internal(x.rotor_systems);
            vlm_system = Base.deepcopy_internal(x.vlm_system),
            panel_system = Base.deepcopy_internal(x.panel_system),
            wake_system = Base.deepcopy_internal(x.wake_system),
            V = Base.deepcopy_internal(x.V),
            W = Base.deepcopy_internal(x.W),
            previousstep=[]
        )

    stackdict[x] = y
    return y
end

##### FUNCTIONS  ###############################################################
function shed_wake(self::UVLMVehicle, Vinf::Function,
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

        flowvlm_wake_system = get_component(self.wake_system, "flowvlm")

        if shed_boundarylayer
            if dipole_d == 0
                error("Boundary layer shedding requested but no `d` was provided!")
            end

            # Shed only boundary layer (dragging line) particles
            VLM2VPM_draggingline(flowvlm_wake_system, _get_prev_wake_system(self),
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
            VLM2VPM(flowvlm_wake_system, pfield, dt, Vinf; t=t,
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
See [Alvarez dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_uniform(x) = 0 <= x <= 1 ? 1 : 0   # Uniform distribution


"""
Linear vortex-sheet distribution for actuator surface model.
See [Alvarez dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_linear(x) = x < 0 ? 0 :                           # Piece-wise linear distribution centered at quarter-chord
           x < 0.25 ? 0.4 + 3.04 * x/0.25 :         # as given by Kim 2015, "Improved actuator surface method for wind turbine application."
           x < 0.50 ? 3.44 - 3.2 * (x-0.25)/0.25 :
           x <=1.00 ? 0.24 - 0.24 * (x-0.5)/0.5  :
           0


"""
Pressure-like vortex-sheet distribution for actuator surface model.
See [Alvarez dissertation](https://scholarsarchive.byu.edu/etd/9589/), Sec. 6.3.2.
"""
g_pressure(x) = x <= 0 ? 0 :         # Pressure-like distribution: peaking by the LE with aerodynamic center at to quarter-chord.
                x <= 1 ? (1-exp(-(x/0.02)^3))/(4*pi*x) / 0.3266200204514099 :
                0

"""
Alias for [`FLOWUnsteady.g_linear`](@ref).
"""
const g_piecewiselinear = g_linear

function generate_static_particle_fun(pfield::vpm.ParticleField, pfield_static::vpm.ParticleField,
                                        self::UVLMVehicle,
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
        function addrotorparticles(rotor)
            _static_particles(pfield, rotor, sigma_rotor)
            if flag; _static_particles(pfield_static, rotor, sigma_rotor); end;
        end

        applytobottom(addrotorparticles, self.rotor_systems)

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

save_vtk(self::UVLMVehicle, args...;
                        optargs...) = save_vtk_base(self, args...; optargs...)

##### INTERNAL FUNCTIONS  ######################################################
function _static_particles(pfield::vpm.ParticleField,
                            vlm_system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
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
    for i in 1:vlm.get_m(vlm_system)
        (Ap, A, B, Bp, _, _, _, gamma) = vlm.getHorseshoe(vlm_system, i)
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
