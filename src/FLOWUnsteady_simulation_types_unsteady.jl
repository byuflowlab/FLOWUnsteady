#=##############################################################################
# DESCRIPTION
    Simulation interface connecting UVLMVehicle vehicles and maneuvers.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


function solve(self::Simulation{V, M, R}, Vinf::Function,
                pfield::vpm.ParticleField, wake_coupled::Bool,
                dt::Real, rlx::Real, sigma_vlm::Real, sigma_rotor::Real,
                rho::Real, speedofsound, staticpfield::vpm.ParticleField,
                hubtiploss_correction;
                init_sol::Bool=false, sigmafactor_vpmonvlm=1,
                extra_static_particles_fun = (args...; optargs...) -> nothing,
                debug=false
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
        # NOTE: VLMs and Rotors solutions are losely coupled

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
        static_particles_fun(pfield, args...) = _static_particles(pfield, vhcl.vlm_system, sigma_vlm)

        ## Evaluate VPM-on-Rotor induced velocity + static particles
        Vinds = Vvpm_on_Xs(pfield, Xs; static_particles_fun=static_particles_fun, dt=dt, fsgm=sigmafactor_vpmonvlm)

        # Solve Rotors
        for (si, rotors) in enumerate(vhcl.rotor_systems)

            # RPM of this rotor system
            RPM = self.RPMref*get_RPM(mnvr, si, t/self.ttot)

            for (ri, rotor) in enumerate(rotors)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems,  Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds, vhcl.rotor_systems, si, ri)

                if wake_coupled
                    vlm.solvefromV(rotor, VindVkin, Vinf, RPM, rho; t=t,
                                    sound_spd=speedofsound,
                                    hubtiploss_correction=hubtiploss_correction,
                                    debug=debug, verbosewarn=true)
                else
                    vlm.solvefromCCBlade(rotor, Vinf, RPM, rho;
                                              t=t, sound_spd=speedofsound,
                                              debug=debug, verbosewarn=true)
                end
            end
        end

    else
        # ---------- 4) Calculate VPM velocity on VLM and Rotor system -----
        m = vlm.get_m(vhcl.vlm_system)

        # Control points of VLM
        Xs_cp_vlm = _get_Xs(vhcl.vlm_system, "CP"; t=t)

        # Mid-points along VLM filaments for calc_aerodynamicforce()
        ## Nodes of every horseshoe
        Ap = _get_Xs(vhcl.vlm_system, "Ap")
        A = _get_Xs(vhcl.vlm_system, "A")
        B = _get_Xs(vhcl.vlm_system, "B")
        Bp = _get_Xs(vhcl.vlm_system, "Bp")
        ## Midpoints of bound vortices
        ApA = (Ap .+ A)/2
        AB = (A .+ B)/2
        BBp = (B .+ Bp)/2
        ## Organize them
        Xs_ApA_AB_BBp_vlm = vcat(ApA, AB, BBp)

        # Points at rotors where to calculate induced velocities
        Xs_rotors = _get_midXs(vhcl.rotor_systems)

        # Calculate VPM velocity on all points (VLM and rotors)
        Vvpm = Vvpm_on_Xs(pfield, vcat(Xs_cp_vlm, Xs_ApA_AB_BBp_vlm, Xs_rotors);
                            dt=dt, fsgm=sigmafactor_vpmonvlm,
                            static_particles_fun=extra_static_particles_fun)

        Vvpm_cp_vlm = Vvpm[1:m]
        Vvpm_ApA_AB_BBp_vlm = Vvpm[m+1:4*m]
        Vvpm_rotors = Vvpm[4*m+1:end]

        # Stitches previous rotor solutions for blade induced velocity
        prev_rotor_systems = _get_prev_rotor_systems(vhcl)
        allrotors = vlm.WingSystem()

        for (si, rotors) in enumerate(vhcl.rotor_systems)
            for (ri, rotor) in enumerate(rotors)

                vlm.addwing(allrotors, "S$(si)R$(ri)", rotor; reset=false)
                vlm.getHorseshoe(rotor, 1)          # Force HS calculation
                vlm._addsolution(rotor, "Gamma",    # Give previous solution
                                prev_rotor_systems[si][ri]._wingsystem.sol["Gamma"])
            end
        end

        # Particles for Rotor-on-VLM induced velocity (and Rotor-on-Rotor)
        static_particles_fun2(pfield, args...) = _static_particles(pfield, allrotors, sigma_rotor)

        # Evaluate Rotor-on-VLM induced velocity
        Vrotor_on_wing = Vvpm_on_Xs(staticpfield, vcat(Xs_cp_vlm, Xs_ApA_AB_BBp_vlm);
                                    static_particles_fun=static_particles_fun2, dt=dt, fsgm=sigmafactor_vpmonvlm)

        # Add and save VPM-on-VLM and Rotor-on-VLM induced velocity
        vlm._addsolution(vhcl.vlm_system, "Vvpm",
                                         Vvpm_cp_vlm + Vrotor_on_wing[1:m]; t=t)
        vlm._addsolution(vhcl.vlm_system, "Vvpm_ApA",
                        Vvpm_ApA_AB_BBp_vlm[1:m] + Vrotor_on_wing[m+1:2*m]; t=t)
        vlm._addsolution(vhcl.vlm_system, "Vvpm_AB",
                        Vvpm_ApA_AB_BBp_vlm[m+1:2*m] + Vrotor_on_wing[2*m+1:3*m]; t=t)
        vlm._addsolution(vhcl.vlm_system, "Vvpm_BBp",
                        Vvpm_ApA_AB_BBp_vlm[2*m+1:3*m] + Vrotor_on_wing[3*m+1:4*m]; t=t)

        # Calculate induced velocities to use in rotor solver
        ## Particles for VLM-on-Rotor induced velocity
        # NOTE: If I keep the rotor particles wouldn't I be double-counting
        # the blade induced velocity and make the solution unstable?
        # ANSWER: We are never double-accounting for the blade induced velocity
        function static_particles_fun3(pfield, args...)
            # Rotor static particles
            static_particles_fun2(pfield, args...)
            # VLM static particles
            _static_particles(pfield, vhcl.vlm_system, sigma_vlm)
        end

        ## Evaluate VLM-on-Rotor and Rotor-on-Rotor induced velocity
        Vvlmrotor_on_rotor = Vvpm_on_Xs(staticpfield, Xs_rotors;
                              static_particles_fun=static_particles_fun3, dt=dt, fsgm=sigmafactor_vpmonvlm)

        # Add VPM-on-Rotor, VLM-on-Rotor, and Rotor-on-Rotor induced velocity
        Vinds = Vvpm_rotors + Vvlmrotor_on_rotor

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

                # Calculate kinematic velocities
                Vkin = _Vkinematic_rotor(vhcl.rotor_systems,
                                         prev_rotor_systems, si, ri, dt)

                # Get velocities induced at every blade of this rotor
                this_Vinds = _parse_midXs(vhcl.rotor_systems, Vinds, si, ri)

                VindVkin = _format_blades(this_Vinds + Vkin, vhcl.rotor_systems,
                                                                        si, ri)
                if wake_coupled
                    vlm.solvefromV(rotor, VindVkin, Vinf, RPM, rho; t=t,
                                    sound_spd=speedofsound,
                                    hubtiploss_correction=hubtiploss_correction,
                                    debug=debug, verbosewarn=false)
                else
                    vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; t=t,
                                            sound_spd=speedofsound,
                                            debug=debug, verbosewarn=false)
                end
            end
        end

        # Rotate rotors
        # NOTE: this means that the rotational velocity hadn't been included in
        # the kinematic velocities up to this point
        rotate_rotors(self, dt)
    end
end
