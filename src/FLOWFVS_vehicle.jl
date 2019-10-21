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
    `AbstractVehicle{N, M}`

Type handling all geometries and subsystems that define a flight vehicle.

`N` indicates the number of tilting systems in this vehicle, while and `M`
indicates the number of rotor systems.
"""
abstract type AbstractVehicle{N, M} end

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
struct VLMVehicle{N, M} <: AbstractVehicle{N, M}

    # Required inputs
    system::vlm.WingSystem

    # Optional inputs
    tilting_systems::NTuple{N, vlm.WingSystem}
    rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}
    vlm_system::vlm.WingSystem
    wake_system::vlm.WingSystem
    grids::Array{gt.GridTypes, 1}

    VLMVehicle{N, M}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem(),
                    grids=Array{gt.GridTypes, 1}()
                ) where {N, M} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system,
                    grids
                )
end

# Implicit N and M constructor
VLMVehicle(system::vlm.WingSystem;
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        optargs...
        ) where {N, M} = VLMVehicle{N, M}( system;
                                        tilting_systems=tilting_systems,
                                        rotor_systems=rotor_systems, optargs...)


##### FUNCTIONS  ###############################################################
get_ntltsys(self::VLMVehicle) = typeof(self).parameters[1]

get_nrtrsys(self::VLMVehicle) = typeof(self).parameters[2]

##### INTERNAL FUNCTIONS  ######################################################
function save_vtk(self::VLMVehicle, filename; path=nothing, num=nothing, optargs...)
    strn = vlm.save(self.system, filename; path=path, num=num, optargs...)

    for (i, grid) in enumerate(self.grids)
        strn *= gt.save(grid, filename*"_Grid$i"; path=path, num=num)
    end

    return strn
end

##### END OF VEHICLE ###########################################################
