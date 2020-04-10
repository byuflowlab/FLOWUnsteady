#=##############################################################################
# DESCRIPTION
    Vehicle with vortex-lattice method (VLM) and blade-element momentum (BEM)
    models assuming quasi-steady wakes. This means that at every time step the
    wake of VLM models are represented as rigid, semi-infinite filaments, and
    wakes of BEM models are obtained from a conservation of momentum
    assumptions. In this approach only VLM-on-VLM and VLM-on-BEM interactions
    are accounted for, meaning that the wake of each BEM rotor has no effect
    on other rotors and wings.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################
println("Rabbit0")
################################################################################
# QUASI-STEADY VLM VEHICLE TYPE
################################################################################
"""
    `QVLMVehicle(system; optargs...)`

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
                                    have a rigid, semi-infinite VLM wake.
* `grids::Array{gt.GridTypes, 1}`: Array of grids that will be translated and
                                    rotated along with `system`.
"""
@inherit QVLMVehicle VLMVehicle{1, 1, Float64} Any begin end
println("Rabbit7")

# Implicit N and M constructor
# QVLMVehicle(system::vlm.WingSystem;
#             V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
#             tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
#             rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
#             grids=Array{gt.GridTypes, 1}(),
#             optargs...
#             ) where {N, M, R} = QVLMVehicle{N, M, R}( system;
#                                     V=V, W=W,
#                                     tilting_systems=tilting_systems,
#                                     rotor_systems=rotor_systems,
#                                     grids=Array{gt.GridTypes, 1}(grids),
#                                     grid_O=[zeros(R, 3) for i in 1:length(grids)],
#                                     optargs...)


##### FUNCTIONS  ###############################################################

##### INTERNAL FUNCTIONS  ######################################################

##### END OF VEHICLE ###########################################################
