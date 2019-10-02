#=##############################################################################
# DESCRIPTION
    Simulation driver.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

"""
    * `wake_system::vlm.System`     : VLM system of all FLOWVLM objects shedding
                                        wakes (typically `rotors`+`vlm_system`).
    * `vlm_system::vlm.System`      : VLM system of all FLOWVLM objects to be
                                        solved through the FLOWVLM solver
                                        (typically all lifting surfaces, but
                                        not rotors).
"""
function run_simulation(maneuver::Function,
                             system, rotors,
                             tilting_systems, rotors_tilting_systems,
                             wake_system, vlm_system,
                             fuselage;
                             # SIMULATION OPTIONS
                             Vcruise=125*0.44704,       # Cruise speed
                             RPMh_w=400,                # RPM of main wing rotors in hover
                             telapsed=30.0,             # Total time to perform maneuver
                             nsteps=100,                # Time steps
                             rand_RPM=true,             # Randomize RPM fluctuations
                             Vinf=(X,t)->zeros(3),      # Freestream velocity
                             # SOLVERS OPTIONS
                             vpmsolver="ExaFMM",        # VPM solver
                             # OUTPUT OPTIONS
                             save_path="temps/vahanasimulation00",
                             run_name="vahana",
                             create_savepath=true,      # Whether to create save_path
                             prompt=true,
                             verbose=true, v_lvl=1, verbose_nsteps=10,
                             nsteps_save=1,             # Save vtks every this many steps
                             nsteps_restart=-1,         # Save jlds every this many steps
                             save_code=module_path,     # Saves the source code in this path
                             )



    # asdasd
    ############################################################################
    # SOLVERS SETUP
    ############################################################################
    dt = telapsed/nsteps            # (s) time step
    nsteps_relax = 1                # Relaxation every this many steps

    ############################################################################
    # SIMULATION SETUP
    ############################################################################
    # Initiate particle field
    max_particles = ceil(Int, 0.1*nsteps*vlm.get_m(wake_system))
    pfield = vpm.ParticleField(max_particles, Vinf, nothing, vpmsolver)

    # NOTE: Here I'm adding some dummy particles for testing
    for i in 1:51
        vpm.addparticle(pfield, vcat(rand(3), zeros(3), 1.0, 1.0))
    end

    prev_vlm_system = nothing

    """
    Calculates kinematic and wake-induced velocity on VLM an adds them as a
    solution field
    """
    function V_on_VLM(PFIELD, T, DT; targetX="CP")
        Vvpm = Vvpm_on_VLM(PFIELD, vlm_system, vpmsolver; targetX=targetX)
        vlm._addsolution(vlm_system, "Vvpm", Vvpm; t=T)

        Vkin = Vtranslation(vlm_system, prev_vlm_system, DT; t=T,
                                                          targetX=targetX)
        vlm._addsolution(vlm_system, "Vkin", Vkin; t=T)
    end

    """
    Function for FLOWVLM to fetch and add extra velocities to what every
    control point imposes as the boundary condition
    """
    function extraVinf(i, t; wing=nothing)
        if wing==nothing; error("Logic error!"); end;
        return wing.sol["Vvpm"][i] + wing.sol["Vkin"][i]
    end

    Vaircraft, angles, RPMs = maneuver(; verbose=verbose, v_lvl=v_lvl+1,
                                         vis_nsteps=nsteps)

    # Set dummy Vinf and RPM for saving horseshoes
    vlm.setVinf(system, Vinf)
    for rotor in rotors; vlm.setRPM(rotor, RPMh_w*(RPMs(0.0)[1])); end;


    ############################################################################
    # SIMULATION RUNTIME FUNCTION
    ############################################################################

    prev_angles = Float64[angles(0)...]     # Previous angles (deg)
    O_fuselage = zeros(3)                   # Current origin of fuselage

    """
        This function gets called by `vpm.run_vpm!` at every time step.
    """
    function runtime_function(PFIELD, T, DT)

        # Saves the vlm system from the previous step
        prev_vlm_system = deepcopy(vlm_system)

        # ---------- TRANSLATION AND ROTATION OF SYSTEM ------------------------
        if PFIELD.nt!=0
            t = T/telapsed              # Non-dimensional time

            # Translation of aircraft
            V = Vcruise * Vaircraft(t)
            dX = V*DT

            vlm.setcoordsystem(system, system.O + dX, system.Oaxis)
            vlm.vtk.lintransform!(fuselage, eye(3), dX)

            O_fuselage .+=  dX

            # Rotation of tilting systems and aircraft (dangles[end] is aircraft)
            new_angles = [angles(t)...]
            dangles = new_angles - prev_angles

            for (j, dangle) in enumerate(dangles)

                Oaxis = vlm.vtk.rotation_matrix2(0, -dangle, 0)

                if j != size(dangles, 1)        # Titlting systems
                    sys = tilting_systems[j]
                else                            # Aircraft and fuselage
                    sys = system
                    vlm.vtk.lintransform!(fuselage, eye(3), -O_fuselage)
                    vlm.vtk.lintransform!(fuselage, Oaxis, O_fuselage)
                end

                vlm.setcoordsystem(sys, sys.O, Oaxis*sys.Oaxis)
            end

            prev_angles[:] = new_angles


            # Rotation of rotors in every tilting system
            rpms = RPMs(t)
            for j in 1:size(rpms, 1)
                for rotor in rotors_tilting_systems[j]
                    rpm = rpms[j] * (rand_RPM ? 1 + (rand()-0.5)*0.1 : 1.0)
                    rotation = 360*(RPMh_w*rpm)/60 * DT
                    vlm.rotate(rotor, rotation)
                end
            end

        end

        # Solve VLM system
        V_on_VLM(PFIELD, T, DT; targetX="CP")
        vlm.solve(vlm_system, Vinf; t=T, extraVinf=extraVinf, keep_sol=true) # TODO: UNCOMMENT THIS LINE TO IGNORE RIGID WAKE
                                            # vortexsheet=(X,t)->zeros(3))



        # Output vtks
        if save_path!=nothing && PFIELD.nt%nsteps_save==0
            vlm.save(system, run_name; save_horseshoes=true, path=save_path, num=PFIELD.nt)
            vlm.vtk.save(fuselage, run_name*"_FuselageGrid"; path=save_path, num=PFIELD.nt)
        end

        breakflag = false
        return breakflag
    end

    ############################################################################
    # RUN SIMULATION
    ############################################################################
    # Here it uses the VPM-time-stepping to run the simulation
    vpm.run_vpm!(pfield, dt, nsteps; save_path=save_path, run_name=run_name,
                      verbose=verbose,
                      create_savepath=create_savepath,
                      runtime_function=runtime_function, solver_method=vpmsolver,
                      nsteps_save=nsteps_save,
                      save_code=save_code,
                      prompt=prompt,
                      # nsteps_relax=nsteps_relax,
                      nsteps_restart=nsteps_restart,
                      # static_particles_function=generate_static_particles,
                      # beta=cs_beta, sgm0=sgm0,
                      # save_sigma=true, rbf_ign_iterror=true,
                      # rbf_itmax=rbf_itmax, rbf_tol=rbf_tol
                      )
end




"""
Returns the local translational velocity of every control point in `system`.
"""
function Vtranslation(system, prev_system, dt; t=0.0, targetX="CP")
    Vtrans = []

    for i in 1:vlm.get_m(system)
        if targetX=="CP"
            prev_X = vlm.getControlPoint(prev_system, i)
            cur_X = vlm.getControlPoint(system, i)
        else
            prev_X = vlm.getHorseshoe(prev_system, i; t=t)[vlm.VLMSolver.HS_hash[targetX]]
            cur_X = vlm.getHorseshoe(system, i; t=t)[vlm.VLMSolver.HS_hash[targetX]]
        end

        push!(Vtrans, -(cur_X-prev_X)/dt )
    end

    return Vtrans
end


"""
Returns the velocity induced by particle field on every control point
of `system`.
"""
function Vvpm_on_VLM(pfield, system, vpmsolver; targetX="CP")

    if targetX=="CP"
        Xs = [vlm.getControlPoint(system, i
                                        ) for i in 1:vlm.get_m(system)]
    else
        Xs = [vlm.getHorseshoe(system, i; t=t)[vlm.VLMSolver.HS_hash[targetX]
                                        ] for i in 1:vlm.get_m(system)]
    end

    if vpmsolver=="ExaFMM"
        Vvpm = vpm.conv_ExaFMM(pfield; Uprobes=Xs)[2]
    else
        warn("Evaluating VPM-on-VLM velocity without FMM")
        Vvpm = [vpm.reg_Uomega(pfield, X) for X in Xs]
    end

    return Vvpm
end
