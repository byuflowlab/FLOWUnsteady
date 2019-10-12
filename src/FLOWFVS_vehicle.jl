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
# VEHICLE TYPE
################################################################################
"""
    `Vehicle(system; optargs...)`

Type handling all geometries and subsystems that define a flight vehicle.

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
"""
struct Vehicle{N, M}

    # Required inputs
    system::vlm.WingSystem

    # Optional inputs
    tilting_systems::NTuple{N, vlm.WingSystem}
    rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}
    vlm_system::vlm.WingSystem
    wake_system::vlm.WingSystem

    Vehicle{N, M}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem()
                ) where {N, M} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system
                )
end

# Implicit N and M constructor
Vehicle(system::vlm.WingSystem;
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        optargs...
        ) where {N, M} = Vehicle{N, M}( system;
                                        tilting_systems=tilting_systems,
                                        rotor_systems=rotor_systems)


##### FUNCTIONS  ###############################################################
"""
    `get_ntltsys(self::Vehicle)`
Return number of tilting systems.
"""
get_ntltsys(self::Vehicle) = typeof(self).parameters[1]

"""
    `get_nrtrsys(self::Vehicle)`
Return number of rotor systems.
"""
get_nrtrsys(self::Vehicle) = typeof(self).parameters[2]

##### INTERNAL FUNCTIONS  ######################################################

##### END OF VEHICLE ###########################################################
