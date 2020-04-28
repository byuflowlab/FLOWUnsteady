#=##############################################################################
# DESCRIPTION
    Simulation interface connecting QVLMVehicle vehicles and maneuvers.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################


function solve(self::Simulation{V, M, R}, Vinf::Function,
                pfield::vpm.ParticleField, wake_coupled::Bool,
                vpm_solver::String, dt::Real, rlx::Real, sigma::Real, rho::Real,
                speedofsound::Real; init_sol::Bool=false
                ) where {V<:QVLMVehicle, M<:AbstractManeuver, R}


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
        # NOTE: VLMs and Rotors solutions are losely coupled
        # NOTE: VLMs surfaces and their wakes are also losely coupled

        # Set Vinf on system to get horseshoes
        vlm.setVinf(vhcl.system, Vinf)

        # Solve VLMs
        # TODO: Add Rotor-on-VLM induced velocity
        # NOTE: Since there isn't a kinematic velocity in the first time step
        #   (t==0), the semi-infinite wake would start from a back initial
        #   solution if calculated in this step, so this step is rather going
        #   to waste and we wait for the next step to initiate the wake properly
        if init_sol
            # Here I use the freestream-wake for the first step
            vlm.solve(vhcl.vlm_system, Vinf; t=t, keep_sol=true)
        else
            # Here I completely ignore the wake
            vlm.solve(vhcl.vlm_system, Vinf; t=t, keep_sol=true,
                                                    vortexsheet=(X,t)->zeros(3))
        end

        # Calculate induced velocities to use in rotor solver
        ## Points where to calculate induced velocities
        Xs = _get_midXs(vhcl.rotor_systems)

        ## Evaluate VLM-on-Rotor induced velocity
        Vinds = [vlm.Vind(vhcl.vlm_system, X; t=t, ign_infvortex=init_sol)
                                                                    for X in Xs]

        # Solve Rotors
        for (si, rotors) in enumerate(vhcl.rotor_systems)

            # RPM of this rotor system
            RPM = self.RPMref*get_RPM(mnvr, si, t/self.ttot)

            for (ri, rotor) in enumerate(rotors)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems,  Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds, vhcl.rotor_systems, si, ri)

                vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; _Vinds=VindVkin,
                                              t=t, sound_spd=speedofsound)
            end
        end

    else

        # Initialize the wake system as a VLM of only the wake system on the
        # kinematic velocity
        if self.nt==1
            vlm.solve(vhcl.wake_system, Vinf; t=t, extraVinf=_extraVinf1,
                                keep_sol=true, vortexsheet=(X,t)->zeros(3))
        end

        # Previous rotor system for kinematic and induced velocity calculation
        prev_rotor_systems = _get_prev_rotor_systems(vhcl)

        # ---------- 4) Calculate VPM velocity on VLM and Rotor system -----

        # Control points of VLM
        Xs = _get_Xs(vhcl.vlm_system, "CP"; t=t)

        # Wake-induced velocity at every VLM control point
        Vinds = [vlm.Vind(vhcl.wake_system, X; t=t, only_infvortex=true)
                                                                    for X in Xs]

        # Rotor-induced velocity at every VLM control point (only blade-induced
        # velocities---this ignores the rotor slipstream)
        for (si, rotors) in enumerate(vhcl.rotor_systems)
            for (ri, rotor) in enumerate(rotors)

                vlm.getHorseshoe(rotor, 1)          # Force HS calculation
                vlm._addsolution(rotor, "Gamma",    # Give previous solution
                                prev_rotor_systems[si][ri]._wingsystem.sol["Gamma"])

                Vinds .+= [vlm.Vind(rotor, X; t=t, ign_infvortex=true)
                                                                    for X in Xs]
            end
        end

        vlm._addsolution(vhcl.vlm_system, "Vind", Vinds; t=t)

        # Calculate induced velocities to use in rotor solver
        ## Points where to calculate induced velocities
        Xs = _get_midXs(vhcl.rotor_systems)

        ## Evaluate wake-on-Rotor induced velocity
        Vinds = [vlm.Vind(vhcl.wake_system, X; t=t, only_infvortex=true)
                                                                    for X in Xs]
        ## Evaluate VLM-on-Rotor induced velocity
        Vinds .+= [vlm.Vind(vhcl.vlm_system, X; t=t, ign_infvortex=true)
                                                                    for X in Xs]
        ## Evaluate Rotor-on-Rotor (and self) induced velocity
        # NOTE: This include the blade-induced velocity of the rotor on itself,
        #   which is already accounted for in the momentum-balance part of BEMT.
        #   Remember to substract this later or the solver will get on a
        #   positive-feedback instability.
        for rotors in vhcl.rotor_systems
            for rotor in rotors
                Vinds .+= [vlm.Vind(rotor, X; t=t, ign_infvortex=true)
                                                                    for X in Xs]
            end
        end

        # ---------- 5) Solve VLM system -----------------------------------
        vlm.solve(vhcl.vlm_system, Vinf; t=t, extraVinf=_extraVinf3,
                                keep_sol=true, vortexsheet=(X,t)->zeros(3))

        # Relaxes (rlx->1) or stiffens (rlx->0) the VLM solution
        if rlx >= 0
            rlxd_Gamma = rlx*vhcl.vlm_system.sol["Gamma"] +
                                (1-rlx)*_get_prev_vlm_system(vhcl).sol["Gamma"]
            vlm._addsolution(vhcl.vlm_system, "Gamma", rlxd_Gamma)
        end

        # ---------- 5) Solve Rotor system ---------------------------------
        for (si, rotors) in enumerate(vhcl.rotor_systems)

            # RPM of this rotor system
            RPM = self.RPMref*get_RPM(mnvr, si, t/self.ttot)

            for (ri, rotor) in enumerate(rotors)

                # Calculate kinematic velocities
                Vkin = _Vkinematic_rotor(vhcl.rotor_systems,
                                         prev_rotor_systems, si, ri, dt)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems, Vinds, si, ri)

                # Calculate and substract the blade-induced velocity of the
                # rotor on itself
                this_Xs = _parse_midXs(vhcl.rotor_systems, Xs, si, ri)
                this_Vinds .-= [vlm.Vind(rotor, X; t=t, ign_infvortex=true)
                                                               for X in this_Xs]

                VindVkin = _format_blades(this_Vinds + Vkin, vhcl.rotor_systems,
                                                                        si, ri)

                vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; _Vinds=VindVkin,
                                            t=t, sound_spd=speedofsound)
            end
        end

        # Rotate rotors
        # NOTE: this means that the rotational velocity hadn't been included in
        # the kinematic velocitie up to this point
        rotate_rotors(self, dt)
    end
end
