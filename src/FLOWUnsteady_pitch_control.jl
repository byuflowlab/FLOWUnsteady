#=##############################################################################
# DESCRIPTION
    Individual pitch control on FLOWUnsteady Vehicle rotors.

# AUTHORSHIP
  * Author          : Timothy P. Harlow
  * Created         : Feb 2025
  * Last updated    : March 2025
=###############################################################################

# ------------- 0) INDIVIDUAL PITCH CONTROL BLUEPRINTS --------------------------
"""
    PitchAngles{TF}(vehicle::AbstractVehicle{<:Any, <:Any, TF}, nsteps)

A struct to represent the pitch angles of each rotor blade.
"""
struct PitchAngles{TF}
    theta1::Vector{TF}
    theta2::Vector{TF}
    theta3::Vector{TF}

    O::Vector{TF}
end

"""
    PitchAngles(vehicle::AbstractVehicle{<:Any, <:Any, TF}, nsteps) where TF

Constructs a `PitchAngles` object for a given `vehicle` and number of time steps `nsteps`.

# Arguments
- `vehicle::AbstractVehicle{<:Any, <:Any, TF}`: The vehicle containing the rotor systems and blade information.
- `nsteps::Int`: The number of time steps for which pitch angles will be stored.

# Returns
- A `PitchAngles{TF}` instance containing:
  - `theta1`, `theta2`, `theta3`: Zero-initialized pitch angle arrays of size `(nsteps+1)`, representing the pitch angles for three blades.
  - `O`: The origin of the first blade (shared among all blades).

# Notes
- This constructor initializes three arrays (`theta1`, `theta2`, `theta3`) to store pitch angles over time.
- The blade origin `O` is extracted from the first blade in the rotor system.
"""
function PitchAngles(vehicle::AbstractVehicle{<:Any, <:Any, TF}, nsteps) where TF
    theta1 = zeros(TF, nsteps+1)
    theta2 = zeros(TF, nsteps+1)
    theta3 = zeros(TF, nsteps+1)

    blade1 = vehicle.rotor_systems[1][2]._wingsystem.wings[1]
    
    # Blade 1 origin (same as blade 2 and 3 origin)
    O = copy(blade1.O)

    return PitchAngles{TF}(theta1, theta2, theta3, O)
end

"""
    (pitchangles::PitchAngles)(sim::uns.Simulation{<:Any,<:Any,TF}) where TF

Given the `PitchAnlges` object `pitchangles`, set each blade pitch to a predetermined angle
"""
function (pitchangles::PitchAngles)(sim::Simulation{<:Any,<:Any,TF}) where TF
    theta1 = pitchangles.theta1[sim.nt+2] - pitchangles.theta1[sim.nt+1]
    theta2 = pitchangles.theta2[sim.nt+2] - pitchangles.theta2[sim.nt+1]
    theta3 = pitchangles.theta3[sim.nt+2] - pitchangles.theta3[sim.nt+1]
    
    R_y1 = [ cosd(theta1)   0   sind(theta1);
                0           1       0       ;
             -sind(theta1)  0   cosd(theta1) ]

    R_y2 = [ cosd(theta2)   0   sind(theta2);
                0           1       0       ;
             -sind(theta2)  0   cosd(theta2) ]

    R_y3 = [ cosd(theta3)   0   sind(theta3);
                0           1       0       ;
             -sind(theta3)  0   cosd(theta3) ]

    # Individual blade coordinate systems (individual pitch)
    blade1 = sim.vehicle.rotor_systems[1][2]._wingsystem.wings[1]
    blade2 = sim.vehicle.rotor_systems[1][2]._wingsystem.wings[2]
    blade3 = sim.vehicle.rotor_systems[1][2]._wingsystem.wings[3]

    # Current blade axis
    blade1_axis = blade1.Oaxis
    blade2_axis = blade2.Oaxis
    blade3_axis = blade3.Oaxis

    # Wind turbine origin
    O = pitchangles.O

    # Pitch each blade
    vlm.setcoordsystem(blade1, O, R_y1 * blade1_axis)
    vlm.setcoordsystem(blade2, O, R_y2 * blade2_axis)
    vlm.setcoordsystem(blade3, O, R_y3 * blade3_axis)

    return false
end

"""
    (pitchangles::PitchAngles)(x::Vector{Float64})

Takes in a vector `x` that contains the pitch angle at each time step for each blade. 
    `x` is of the structure [blade1_angles, blade2_angles, blade3_angles]
Loads values of `x` into `pitchangles.theta1`, etc.
"""
function (pitchangles::PitchAngles)(x::Vector{Float64})
    y = reshape(x,div(length(x),3),3)
    pitchangles.theta1 .= y[:,1]
    pitchangles.theta2 .= y[:,2]
    pitchangles.theta3 .= y[:,3]
end