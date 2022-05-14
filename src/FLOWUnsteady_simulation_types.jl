#=##############################################################################
# DESCRIPTION
    Simulation interface connecting vehicles and maneuvers.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################



################################################################################
# SIMULATION TYPE
################################################################################
"""
    `Simulation(vehicle, maneuver, Vref::Real, RPMref::R, ttot:R)`

Simulation interface. This type carries the simulation's options and connects
vehicle and maneuver together.
"""
mutable struct Simulation{V<:AbstractVehicle, M<:AbstractManeuver, R<:Real}
    # USER INPUTS: Simulation setup
    vehicle::V              # Vehicle
    maneuver::M             # Maneuver to be performed
    Vref::R                 # Reference velocity in this maneuver
    RPMref::R               # Reference RPM in this maneuver
    ttot::R                 # Total time in which to perform the maneuver

    # OPTION USER INPUTS
    Vinit::Any              # Initial vehicle velocity
    Winit::Any              # Initial vehicle angular velocity


    # INTERNAL PROPERTIES: Runtime parameters
    t::R                    # Dimensional time
    nt::Int                 # Current time step number

    Simulation{V, M, R}(
                            vehicle, maneuver, Vref, RPMref, ttot;
                            Vinit=nothing, Winit=nothing,
                            t=zero(R), nt=-1
                        ) where {V, M, R} = _check(vehicle, maneuver) ? new(
                            vehicle, maneuver, Vref, RPMref, ttot,
                            Vinit, Winit,
                            t, nt
                        ) : nothing
end


# Implicit V and M constructor
Simulation(v::AbstractVehicle, m::AbstractManeuver, n::Real, args...; optargs...
             ) = Simulation{typeof(v), typeof(m), typeof(n)}(v, m, n, args...;
                                                                     optargs...)


# Solvers of concrete AbstractVehicle implementations
for header_name in ["unsteady", "quasisteady"]
  include("FLOWUnsteady_simulation_types_"*header_name*".jl")
end


##### FUNCTIONS  ###############################################################

"""
    `nextstep_kinematic(self::Simulation, dt::Real)`

Takes a kinematic time step `dt` where the new velocity and angular velocity
is calculated and the vehicle is translated and rotated according to it. It also
updates the tilt angle and RPM of every system.

NOTE: No solver is run in this process, rather it uses the current aerodynamic
solution to calculate acceleration and moment.
"""
function nextstep_kinematic(self::Simulation, dt::Real)

    if self.nt==-1 # Setup step (first step of simulation)

        # Initial vehicle linear and angular velocity
        if self.Vinit!=nothing; set_V(self.vehicle, self.Vinit); end;
        if self.Winit!=nothing; set_W(self.vehicle, self.Winit); end;

        # Rotate tilting systems
        angles = get_angles(self.maneuver, self.t/self.ttot)
        tilt_systems(self.vehicle, angles)

    else
        # Linear velocity increment
        dV = calc_dV(self.maneuver, self.vehicle, self.t, dt, self.ttot, self.Vref)

        # Angular velocity increment
        dW = calc_dW(self.maneuver, self.vehicle, self.t, dt, self.ttot)

        # Update vehicle linear and angular velocity
        add_dV(self.vehicle, dV)
        add_dW(self.vehicle, dW)

        # Translates and rotates the vehicle according to current velocity
        nextstep_kinematic(self.vehicle, dt)

        # Rotate tilting systems
        angles = get_angles(self.maneuver, self.t/self.ttot)
        tilt_systems(self.vehicle, angles)

        self.t += dt
    end

    self.nt += 1
end


function save_vtk(self::Simulation, filename; optargs...)
    return save_vtk(self.vehicle, filename; num=self.nt, optargs...)
end

"""
Precalculations before calling the solver.
"""
function precalculations(self::Simulation, Vinf::Function,
                                pfield::vpm.ParticleField, t::Real, dt::Real)
    # Rotate rotors
    rotate_rotors(self, dt)

    # Calculate kinematic velocities
    precalculations(self.vehicle, Vinf, pfield, t, dt, self.nt)

    # # Bring the rotors back to their initial positions. This is needed to avoid
    # # double accounting for the RPM when the solver is run
    # NOTE: I've moved this to after shedding the wake and before running the
    # solver, otherwise the particles shedding wouldn't see the kinematic
    # velocity due to rotor RPM
    # rotate_rotors(self, -dt)

    return nothing
end

function rotate_rotors(self::Simulation{V, M, R}, dt::Real
                        ) where {V<:AbstractVLMVehicle, M<:AbstractManeuver, R}

    for (si, rotors) in enumerate(self.vehicle.rotor_systems)

        # RPM of this rotor system
        RPM = self.RPMref*get_RPM(self.maneuver, si, self.t/self.ttot)

        for rotor in rotors

            # Save solution fields
            sol1 = deepcopy(rotor._wingsystem.sol)
            sol2 = deepcopy(rotor.sol)
            Vinf = rotor._wingsystem.Vinf

            # Rotate rotor
            vlm.rotate(rotor, 360*RPM/60*dt)

            # Force regeneration of horseshoes
            if "Gamma" in keys(sol1)
                vlm.setVinf(rotor, Vinf)
                vlm.setRPM(rotor, RPM)
                vlm.getHorseshoe(rotor, 1)
            end

            # Paste solution fields back
            for (key,val) in sol1; vlm._addsolution(rotor._wingsystem, key, val); end;
            for (key,val) in sol2; rotor.sol[key] = val; end;

        end
    end
    return nothing
end


##### INTERNAL FUNCTIONS  ######################################################
"""
Checks that vehicle and maneuver are compatible.
"""
function _check(vehicle::AbstractVehicle, maneuver::AbstractManeuver;
                                                            raise_error=true)
    res = get_ntltsys(vehicle)==get_ntltsys(maneuver)
    res *= get_nrtrsys(vehicle)==get_nrtrsys(maneuver)

    if raise_error && res==false
        error("Encountered incompatible Vehicle and Maneuver!")
    end

    return res
end
##### END OF SIMULATION  #######################################################
