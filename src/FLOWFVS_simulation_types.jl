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
    precalculations(self.vehicle, Vinf, pfield, t, dt)

    # # Bring the rotors back to their initial positions. This is needed to avoid
    # # double accounting for the RPM when the solver is run
    # NOTE: I've moved this to after shedding the wake and before running the
    # solver, otherwise the particles shedding wouldn't see the kinematic
    # velocity due to rotor RPM
    # rotate_rotors(self, -dt)

    return nothing
end

function solve(self::Simulation{V, M, R}, Vinf::Function,
                pfield::vpm.ParticleField, wake_coupled::Bool,
                vpm_solver::String, dt::Real, rlx::Real, sigma::Real, rho::Real,
                speedofsound::Real; init_sol::Bool=false
                ) where {V<:VLMVehicle, M<:AbstractManeuver, R}


    # Bring the rotors back to their initial positions at beginning of this
    # time step before `precalculations`. This is needed to avoid double
    # accounting for the RPM when the solver is run
    rotate_rotors(self, -dt)

    vhcl = self.vehicle
    mnvr = self.maneuver
    t = self.t

    # TIME-STEPPING PROCEDURE:
    # -1) Solve one particle field time step
    # 0) Translate and rotate systems
    # 1) Recalculate horseshoes with kinematic velocity
    # 2) Paste previous Gamma solution to new system position after translation
    # 3) Shed semi-infinite wake after translation
    # 4) Calculate wake-induced velocity on VLM and Rotor system
    # 5) Solve VLM and Rotor system
    # 6) Shed unsteady-loading wake after new solution
    # 7) Save new solution as prev solution
    # Iterate
    #
    # On the first time step (pfield.nt==0), it only does steps (5) and (7),
    # meaning that the unsteady wake of the first time step is never shed.
    if t==0
        # NOTE: VLMs and Rotors solutions are losely coupled

        # Set Vinf on system to get horseshoes
        vlm.setVinf(vhcl.system, Vinf)

        # Solve VLMs
        # TODO: Add Rotor-on-VLM induced velocity
        if init_sol
            # NOTE: Here I use the semi-infinite wake for the first step, which is
            # may lead to some unphysical results when shedding unsteady loading wake
            vlm.solve(vhcl.vlm_system, Vinf; t=t, keep_sol=true)
        else
            vlm.solve(vhcl.vlm_system, Vinf; t=t, keep_sol=true,
                                                    vortexsheet=(X,t)->zeros(3))
        end

        # Calculate induced velocities to use in rotor solver
        ## Points where to calculate induced velocities
        Xs = _get_midXs(vhcl.rotor_systems)

        ## Particles for VLM-on-Rotor induced velocity
        static_particles = _static_particles(vhcl.vlm_system, sigma)

        ## Evaluate VPM-on-Rotor induced velocity + static particles
        Vinds = Vvpm_on_Xs(pfield, Xs, vpm_solver;
                                            static_particles=static_particles)

        # Solve Rotors
        for (si, rotors) in enumerate(vhcl.rotor_systems)

            # RPM of this rotor system
            RPM = self.RPMref*get_RPM(mnvr, si, t/self.ttot)

            for (ri, rotor) in enumerate(rotors)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems,  Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds, vhcl.rotor_systems, si, ri)

                # vlm.solvefromCCBlade(rotor, Vinf, RPM, rho;
                #                                 t=t, sound_spd=speedofsound)
                vlm.solvefromV(rotor, VindVkin, Vinf, RPM,
                                  rho; t=t, sound_spd=speedofsound)
            end
        end


    elseif wake_coupled==false
        vlm.solve(vhcl.vlm_system, Vinf; t=t, extraVinf=_extraVinf1,
                                                                keep_sol=true)
        error("There isn't a wake-decoupled solver yet!")

    else
        # ---------- 4) Calculate VPM velocity on VLM and Rotor system -----
        # Control points of VLM
        Xs = _get_Xs(vhcl.vlm_system, "CP"; t=t)

        # Generate static particles
        static_particles = Array{Float64, 1}[]

        ### Particles for Rotor induced velocity
        prev_rotor_systems = _get_prev_rotor_systems(vhcl)

        for (si, rotors) in enumerate(vhcl.rotor_systems)
            for (ri, rotor) in enumerate(rotors)

                vlm.getHorseshoe(rotor, 1)          # Force HS calculation
                vlm._addsolution(rotor, "Gamma",    # Give previous solution
                                prev_rotor_systems[si][ri]._wingsystem.sol["Gamma"])
                _static_particles(rotor, sigma; out=static_particles)

            end
        end

        # VPM-induced velocity (+ static particles) at every VLM control point
        Vvpm = Vvpm_on_Xs(pfield, Xs, vpm_solver; static_particles=static_particles)
        vlm._addsolution(vhcl.vlm_system, "Vvpm", Vvpm; t=t)


        # Calculate induced velocities to use in rotor solver
        ## Points where to calculate induced velocities
        Xs = _get_midXs(vhcl.rotor_systems)

        ### Particles for VLM-on-Rotor induced velocity
        # NOTE: If I keep the rotor particles wouldn't I be double-counting
        # the blade induced velocity and make the solution unstable?
        _static_particles(vhcl.vlm_system, sigma; out=static_particles)

        ## Evaluate VPM-on-Rotor induced velocity + static particles
        Vinds = Vvpm_on_Xs(pfield, Xs, vpm_solver;
                                            static_particles=static_particles)

        # ---------- 5) Solve VLM system -----------------------------------
        # Wake-coupled solution
        vlm.solve(vhcl.vlm_system, Vinf; t=t, extraVinf=_extraVinf2,
                                    keep_sol=true, vortexsheet=(X,t)->zeros(3))
        # Wake-decoupled solution
        # vlm.solve(vlm_system, Vinf; t=t, keep_sol=true)

        # Relaxes (rlx->1) or stiffens (rlx->0) the VLM solution
        if rlx > 0
            rlxd_Gamma = rlx*vhcl.vlm_system.sol["Gamma"] +
                                (1-rlx)*_get_prev_vlm_system(vhcl).sol["Gamma"]
            vlm._addsolution(vhcl.vlm_system, "Gamma", rlxd_Gamma)
        end

        # ---------- 5) Solve Rotor system ---------------------------------

        # Solve Rotors
        for (si, rotors) in enumerate(vhcl.rotor_systems)

            # RPM of this rotor system
            RPM = self.RPMref*get_RPM(mnvr, si, t/self.ttot)

            for (ri, rotor) in enumerate(rotors)

                # Calculate kinematic velocities
                Vkin = _Vkinematic_rotor(vhcl.rotor_systems,
                                         prev_rotor_systems, si, ri, dt)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems, Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds + Vkin, vhcl.rotor_systems,
                                                                        si, ri)

                # vlm.solvefromCCBlade(rotor, Vinf, RPM, rho;
                #                                 t=t, sound_spd=speedofsound)
                vlm.solvefromV(rotor, VindVkin, Vinf, RPM,
                                  rho; t=t, sound_spd=speedofsound)
            end
        end

        # Rotate rotors
        rotate_rotors(self, dt)
    end
end

function rotate_rotors(self::Simulation{V, M, R}, dt::Real
                                ) where {V<:VLMVehicle, M<:AbstractManeuver, R}

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
