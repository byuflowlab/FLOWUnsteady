#=##############################################################################
# DESCRIPTION
    Vehicle type handling all defined geometries and their properties.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
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
    `nextstep_kinematic(self::AbstractVehicle, dt::Real)`
Translates and rotates the vehicle in a time step `dt` according to current
linear and angular velocity.
"""
function nextstep_kinematic(self::AbstractVehicle, dt::Real)
    error("$(typeof(self)) has no implementation yet!")
end

"""
    `save_vtk(self::AbstractVehicle, filename; path=nothing, num=nothing,
optargs...)`

Output VTK files with vehicle geometry and solution fields.
"""
function save_vtk(self::AbstractVehicle, filename;
                        path=nothing, num=nothing, optargs...)
    error("$(typeof(self)) has no implementation yet!")
end

##### COMMON FUNCTIONS  ########################################################
##### COMMON INTERNAL FUNCTIONS  ###############################################
##### END OF ABSTRACT VEHICLE ##################################################















################################################################################
# VEHICLE TYPE
################################################################################
"""
    `Vehicle(system; optargs...)`

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
struct VLMVehicle{N, M, R} <: AbstractVehicle{N, M, R}

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
    grid_O::Array{Array{R, 1}, 1}           # Origin of every grid
    # grid_Oaxis::Array{Array{R, 2}, 1}       # Axes of every grid

    VLMVehicle{N, M, R}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem(),
                    grids=Array{gt.GridTypes, 1}(),
                    V=zeros(3), W=zeros(3),
                    grid_O=Array{Array{Float64, 1}, 1}(),
                    # grid_Oaxis=Array{Array{Float64, 2}, 1}(),
                ) where {N, M, R} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system,
                    grids,
                    V, W,
                    grid_O,
                    # grid_Oaxis
                )
end

# Implicit N and M constructor
VLMVehicle(system::vlm.WingSystem;
        V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        grids=Array{gt.GridTypes, 1}(),
        optargs...
        ) where {N, M, R} = VLMVehicle{N, M, R}( system;
                                V=V, W=W,
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                grids=Array{gt.GridTypes, 1}(grids),
                                grid_O=[zeros(R, 3) for i in 1:length(grids)],
                                # grid_Oaxis=[eye(R, 3) for i in 1:length(grids)]
                                optargs...)


##### FUNCTIONS  ###############################################################
get_ntltsys(self::VLMVehicle) = typeof(self).parameters[1]

get_nrtrsys(self::VLMVehicle) = typeof(self).parameters[2]


function add_dV(self::VLMVehicle, dV)
    self.V .+= dV
    return nothing
end

function add_dW(self::VLMVehicle, dW)
    self.W .+= dW
    return nothing
end

function set_angles(self::VLMVehicle{N,M,R}, angles::NTuple{N, Array{R2, 1}}
                                                    ) where{N, M, R, R2<:Real}
    error("Missing implementation")
end

function nextstep_kinematic(self::VLMVehicle, dt::Real)
    dX = dt*self.V                  # Translation
    dA = 180/pi * dt*self.W         # Angular rotation (degrees)

    O = self.system.O + dX          # New origin of the system (translation)
    M = gt.rotation_matrix2([-a for a in dA]...) # Rotation matrix
    Oaxis = M*self.system.Oaxis     # New axes of the system (rotation)

    # Translation and rotation
    vlm.setcoordsystem(self.system, O, Oaxis)

    # Translation and rotation of every grid
    for i in 1:length(self.grids)
        self.grid_O[i] .+= dX
        # self.grid_Oaxis[i] .= M

        # Translation
        gt.lintransform!(self.grids[i], eye(3), dX)

        # Brings the grid back to the global origin
        gt.lintransform!(self.grids[i], eye(3), -self.grid_O[i])

        # Rotation and brings the grid back to its position
        gt.lintransform!(self.grids[i], M, self.grid_O[i])
    end

    return nothing
end

##### INTERNAL FUNCTIONS  ######################################################
function save_vtk(self::VLMVehicle, filename; path=nothing, num=nothing, optargs...)
    strn = vlm.save(self.system, filename; path=path, num=num, optargs...)

    for (i, grid) in enumerate(self.grids)
        strn *= gt.save(grid, filename*"_Grid$i"; path=path, num=num)
    end

    return strn
end

##### END OF VEHICLE ###########################################################
