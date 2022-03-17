#=##############################################################################
# DESCRIPTION
    Vehicle with vortex-lattice method (VLM) and VPM-based propeller models
    shedding VPM wakes.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


################################################################################
# UNSTEADY VLM VEHICLE TYPE
################################################################################
"""
    `UVLMVehicle(system; optargs...)`

Type handling all geometries and subsystems that define a flight vehicle made
out of VLM (Wing, WingSystem, Rotor) components.

# ARGUMENTS
* `system::vlm.WingSystem`:        System of all FLOWVLM objects. This system
                                    is considered as the entire vehicle. Not all
                                    components in this system will be solved,
                                    but they will all be rotated and translated
                                    during maneuver.
# OPTIONAL ARGUMENTS
* `tilting_systems::Tuple(vlm.WingSystem, ...)`:   Tuple of all FLOWVLM
                                    tilting objects, where `tilting_systems[i]`
                                    contains the i-th FLOWVLM system of lifting
                                    surfaces and rotors that tilt together.
* `rotors_systems::Tuple(Array{vlm.Rotor,1}, ...)`:   Tuple of groups of Rotors
                                    that share a common RPM.
* `vlm_system::vlm.WingSystem`:    System of all FLOWVLM objects to be solved
                                    through the VLM solver.
* `wake_system::vlm.WingSystem`:   System of all FLOWVLM objects that will
                                    shed a VPM wake.
* `grids::Array{gt.GridTypes, 1}`: Array of grids that will be translated and
                                    rotated along with `system`.
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

# Implicit N and M constructor
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

# Alias
const VLMVehicle = UVLMVehicle

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


function generate_static_particle_fun(pfield::vpm.ParticleField, pfield_static::vpm.ParticleField,
                                        self::VLMVehicle,
                                        sigma_vlm::Real, sigma_rotor::Real;
                                        vlm_vortexsheet=false,
                                        vlm_vortexsheet_overlap=2.125,
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
                                vortexsheet_overlap=vlm_vortexsheet_overlap)
        if flag
            _static_particles(pfield_static, self.vlm_system, sigma_vlm;
                                vortexsheet=vlm_vortexsheet,
                                vortexsheet_overlap=vlm_vortexsheet_overlap)
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
                            vortexsheet::Bool=false,
                            vortexsheet_overlap::Real=2.125,
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
                                    index=i)

            elseif j==2 || j==3            # Case of trailing vortex with vortex sheet
                # Length of bound vortex
                dl .= x2
                dl .-= x1

                l = sqrt(dl[1]^2 + dl[2]^2 + dl[3]^2)   # Length (TE to lifting line)
                                                        # Number of particles to use
                np           = ceil(Int, l / (sigma/vortexsheet_overlap) )
                dl ./= np                   # Step length


                # Start at x1 and shift by half a step to center particles
                dl ./= 2
                X .= x1
                X .-= dl
                dl .*= 2

                # Break vortex strength down
                Gamma ./= np

                # Add particles
                for ni in 1:np
                    X .+= dl

                    vpm.add_particle(pfield, X, Gamma, sigma;
                                        vol=0, circulation=abs(gamma), static=true,
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

                # Spread vortex strength among sheet particles
                # Gamma ./= np
                Gamma ./= np^(2/3)

                # Add particles
                for ni in 1:np
                    X .+= dl

                    vpm.add_particle(pfield, X, Gamma, sigma;
                                        # vol=0, circulation=abs(gamma/np), static=true,
                                        vol=0, circulation=abs(gamma/np^(2/3)), static=true,
                                        index=i) # NOTE: Here I'm using the index to indicate
                                                 # the horseshoe that this particle belongs to
                end

            end

        end
    end
end
##### END OF VEHICLE ###########################################################
