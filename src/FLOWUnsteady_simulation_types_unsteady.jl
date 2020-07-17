#=##############################################################################
# DESCRIPTION
    Simulation interface connecting UVLMVehicle vehicles and maneuvers.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


function solve(self::Simulation{V, M, R}, Vinf::Function,
                pfield::vpm.ParticleField, wake_coupled::Bool,
                vpm_solver::String, dt::Real, rlx::Real, sigma::Real, rho::Real,
                speedofsound::Union{Real,Void}; init_sol::Bool=false, Uinds=nothing
                ) where {V<:UVLMVehicle, M<:AbstractManeuver, R}


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
    if self.nt==0
        # NOTE: VLMs and Rotors solutions are loosely coupled

        # Set Vinf on system to get horseshoes
        vlm.setVinf(vhcl.system, Vinf)

        # Solve VLMs
        # TODO: Add Rotor-on-VLM induced velocity
        if init_sol
            # NOTE: Here I use the semi-infinite wake for the first step, which
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

                if wake_coupled
                    vlm.solvefromV(rotor, VindVkin, Vinf, RPM,
                                  rho; t=t, sound_spd=speedofsound)
                else
                    vlm.solvefromCCBlade(rotor, Vinf, RPM, rho;
                                              t=t, sound_spd=speedofsound)
                end
            end
        end

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
        if wake_coupled
            vlm.solve(vhcl.vlm_system, Vinf; t=t, extraVinf=_extraVinf2,
                                    keep_sol=true, vortexsheet=(X,t)->zeros(3))
        # Wake-decoupled solution
        else
            vlm.solve(vhcl.vlm_system, Vinf; t=t, extraVinf=_extraVinf1,
                                                                keep_sol=true)
        end

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
                # println("Sherlock! FLOWUnsteady_simulation_types_unsteady.jl : 161")
                # println("\tRotor Index: ", ri)
                # println("\tTime:        ", t)
                # println("")

                # Calculate kinematic velocities
                Vkin = _Vkinematic_rotor(vhcl.rotor_systems,
                                         prev_rotor_systems, si, ri, dt)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems, Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds + Vkin, vhcl.rotor_systems,
                                                                        si, ri)
                if wake_coupled
                    vlm.solvefromV(rotor, VindVkin, Vinf, RPM,
                                  rho; t=t, sound_spd=speedofsound,
                                                Uinds = Uinds)
                else
                    vlm.solvefromCCBlade(rotor, Vinf, RPM, rho;
                                                t=t, sound_spd=speedofsound,
                                                Uinds = Uinds)
                end
            end
        end

        # Rotate rotors
        # NOTE: this means that the rotational velocity hadn't been included in
        # the kinematic velocitie up to this point
        rotate_rotors(self, dt)
    end
end
