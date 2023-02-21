#=##############################################################################
# DESCRIPTION
    Vehicle type handling all defined geometries and their properties.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

################################################################################
# ABSTRACT VEHICLE TYPE
################################################################################
"""
    `AbstractVehicle{N, M, R}`

Type handling all geometries and subsystems that define a flight vehicle.

`N` indicates the number of tilting systems in this vehicle, while and `M`
indicates the number of rotor systems. `R` is a Real type.

Implementations must have the following properties:
    * `V::Array{R, 1}`          : Current velocity vector of the vehicle in the
                                    global coordinate system.
    * `W::Array{R, 1}`          : Current angular velocity vector of the vehicle
                                    in the global coordinate system.
"""
abstract type AbstractVehicle{N, M, R} end

##### FUNCTIONS REQUIRED IN IMPLEMENTATIONS ####################################
"""
    `get_ntltsys(self::AbstractVehicle)`
Return number of tilting systems.
"""
function get_ntltsys(self::AbstractVehicle)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `get_nrtrsys(self::AbstractVehicle)`
Return number of rotor systems.
"""
function get_nrtrsys(self::AbstractVehicle)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `add_dV(self::AbstractVehicle, dV)`
Increments the velocity of the vehicle by `dV`.
"""
function add_dV(self::AbstractVehicle, dV)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `add_dW(self::AbstractVehicle, dV)`
Increments the angular velocity of the vehicle by `dW`.
"""
function add_dW(self::AbstractVehicle, dW)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `set_V(self::AbstractVehicle, V)`
Set current vehicle velocity to `V`.
"""
function set_V(self::AbstractVehicle, V)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `set_W(self::AbstractVehicle, W)`
Set current vehicle angular velocity to `W`.
"""
function set_W(self::AbstractVehicle, W)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `nextstep_kinematic(self::AbstractVehicle, angles)`
Tilts every tilting system of this vehicle into its corresponding new angle,
where `angles[i]=[Ax, Ay, Az]` is the new angle of the i-th tilting system (in
degrees).
"""
function tilt_systems(self::AbstractVehicle, angles)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `nextstep_kinematic(self::AbstractVehicle, dt::Real)`
Translates and rotates the vehicle in a time step `dt` according to current
linear and angular velocity.
"""
function nextstep_kinematic(self::AbstractVehicle, dt::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `precalculations(self, pfield, t, dt)`
Precalculations before calling the solver and before shedding trialing wake.
"""
function precalculations(self::AbstractVehicle, Vinf::Function,
                                pfield::vpm.ParticleField, t::Real, dt::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
Shed VPM wake. If `unsteady_shedcrit<=0` it sheds wake due to loading
distribution, otherwise it will shed wake due to unsteady loading.
"""
function shed_wake(self::AbstractVehicle, Vinf::Function,
                            pfield::vpm.ParticleField, dt::Real; t=0.0,
                            unsteady_shedcrit=-1.0, p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing,
                            omit_shedding=[])
    error("$(typeof(self)) has no implementation yet!")
end

# """
# Solves the aerodynamics of this vehicle.
# """
# function solve(self::AbstractVehicle, Vinf::Function, pfield::vpm.ParticleField,
#                 wake_coupled::Bool, vpm_solver::String,  t::Real, dt::Real,
#                                                       rlx::Real, sigma::Real)
#     error("$(typeof(self)) has no implementation yet!")
# end

"""
Returns a function that generates an array of particles representing the surface
of the vehicle as a collection of vortex particles.
"""
function generate_static_particle_fun(self::AbstractVehicle,
                                            sigma_vlm::Real, sigma_rotor::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    save_vtk(self::AbstractVehicle, prefix; path="", optargs...)

Output VTK files with vehicle geometry and solution fields.
"""
function save_vtk(self::AbstractVehicle, filename;
                        path=nothing, num=nothing, optargs...)
    error("$(typeof(self)) has no implementation yet!")
end

##### COMMON FUNCTIONS  ########################################################
##### COMMON INTERNAL FUNCTIONS  ###############################################
##### END OF ABSTRACT VEHICLE ##################################################
