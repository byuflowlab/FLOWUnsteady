#=##############################################################################
# DESCRIPTION
    Simulation driver.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


# TODO: Why is the VLM solver oscillating>??
# This is a mess, and I have no idea what is going on. I'd recommend
# scaling down to a simpler simulation like a single wing shedding a VPM wake,
# then make that wing heave, then add one rotor, etc.


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
                             tilting_systems, rotors_systems,
                             wake_system, vlm_system,
                             fuselage;
                             # SIMULATION OPTIONS
                             Vcruise=125*0.44704,       # Cruise speed
                             RPMh_w=400,                # RPM of main wing rotors in hover
                             telapsed=30.0,             # Total time to perform maneuver
                             nsteps=100,                # Time steps
                             rand_RPM=true,             # Randomize RPM fluctuations
                             Vinf=(X,t)->zeros(3),      # Freestream velocity
                             sound_spd=343,             # (m/s) speed of sound
                             rho=1.225,                 # (kg/m^3) air density
                             mu=1.81e-5,                # Air dynamic viscosity
                             # SOLVERS OPTIONS
                             vpm_solver="ExaFMM",       # VPM solver
                             vpm_timesch="rk",          # VPM time stepping scheme
                             vpm_strtch="transpose",    # VPM stretching scheme
                             nsteps_relax=1,            # Steps in between VPM relaxation
                             relaxfactor=0.3,           # VPM relaxation factor
                             p_per_step=1,              # Particle sheds per time step
                             sigmafactor=1.0,           # Particle core overlap
                             overwrite_sigma=nothing,   # Overwrite cores to this value (ignoring sigmafactor)
                             vlm_sigma=-1,              # VLM regularization
                             vlm_rlx=-1,                # VLM relaxation
                             wake_coupled=true,         # Couple VPM wake on VLM solution
                             shed_unsteady=true,        # Whether to shed unsteady-loading wake
                             extra_runtime_function=(PFIELD,T,DT)->false,
                             # OUTPUT OPTIONS
                             save_path="temps/vahanasimulation00",
                             run_name="vahana",
                             create_savepath=true,      # Whether to create save_path
                             prompt=true,
                             verbose=true, v_lvl=1, verbose_nsteps=10,
                             nsteps_save=1,             # Save vtks every this many steps
                             nsteps_restart=-1,         # Save jlds every this many steps
                             save_code=module_path,     # Saves the source code in this path
                             save_horseshoes=false,     # Save VLM horseshoes
                             )


    # THIS IS ONLY FOR TESTING. GET RID OF THIS TO SHED PROP WAKES
    # wake_system = vlm_system

    if wake_coupled==false
        warn("Running wake-decoupled simulation")
    end

    # asdasd
    ############################################################################
    # SOLVERS SETUP
    ############################################################################
    dt = telapsed/nsteps            # (s) time step
    nsteps_relax = 1                # Relaxation every this many steps

    if vlm_sigma<=0
        vlm.VLMSolver._blobify(false)
    else
        vlm.VLMSolver._blobify(true)
        vlm.VLMSolver._smoothing_rad(vlm_sigma)
    end


    # ---------------- SCHEMES -------------------------------------------------
    vpm.set_TIMEMETH(vpm_timesch)       # Time integration scheme
    vpm.set_STRETCHSCHEME(vpm_strtch)   # Vortex stretching scheme
    vpm.set_RELAXETA(relaxfactor/dt)    # Relaxation param
    # TODO: Set CS scheme up
    vpm.set_PSE(false)                  # Viscous diffusion through PSE
    vpm.set_CS(false)                   # Viscous diffusion through CS
    vpm.set_P2PTYPE(Int32(5))           # P2P kernel (1=Singular, 3=Winckelmans, 5=GausErf)

    ############################################################################
    # SIMULATION SETUP
    ############################################################################
    # Initiate particle field
    # max_particles = ceil(Int, 0.1*nsteps*vlm.get_m(wake_system))
    # TODO: Reduce the max number of particles
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(wake_system)+1)*p_per_step)
    pfield = vpm.ParticleField(max_particles, Vinf, nothing, vpm_solver)

    pfield.nu = mu/rho                  # Kinematic viscosity

    # TODO: Add particle removal

    prev_vlm_system = nothing
    prev_wake_system = nothing

    """
    Calculates kinematic and wake-induced velocity on VLM an adds them as a
    solution field
    """
    function V_on_VLM(PFIELD, T, DT; targetX="CP")
        Vvpm = Vvpm_on_VLM(PFIELD, vlm_system, vpm_solver; targetX=targetX)
        vlm._addsolution(vlm_system, "Vvpm", Vvpm; t=T)

        Vkin = Vtranslation(vlm_system, prev_vlm_system, DT; t=T,
                                                          targetX=targetX)
        vlm._addsolution(vlm_system, "Vkin", Vkin; t=T)
    end

    """
    Function for FLOWVLM to fetch and add extra velocities to what every
    control point imposes as the boundary condition
    """
    function extraVinf1(i, t; wing=nothing)
        if wing==nothing; error("Logic error!"); end;
        return wing.sol["Vkin"][i]
    end
    function extraVinf2(i, t; wing=nothing)
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

    prev_angles = [angles(0)...]            # Previous angles (deg)
    O_fuselage = zeros(3)                   # Current origin of fuselage

    """
        This function gets called by `vpm.run_vpm!` at every time step.
    """
    function runtime_function(PFIELD, T, DT)
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

    # TODO: Add vlm-on-vpm velocity

        # Saves the vlm system from the previous step
        prev_vlm_system = deepcopy(vlm_system)
        prev_wake_system = deepcopy(wake_system)


        if PFIELD.nt!=0
            # ---------- 0) TRANSLATION AND ROTATION OF SYSTEM -----------------
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

                Oaxis = vlm.vtk.rotation_matrix2([-a for a in dangle]...)

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


            # Rotation of rotors
            rpms = RPMs(t)
            cur_RPMs = Dict()
            for j in 1:size(rpms, 1)
                for rotor in rotors_systems[j]
                    rpm = rpms[j] * (rand_RPM ? 1 + (rand()-0.5)*0.1 : 1.0)
                    rotation = 360*(RPMh_w*rpm)/60 * DT
                    vlm.rotate(rotor, rotation)

                    cur_RPMs[rotor] = RPMh_w*rpm
                end
            end

            # ---------- 1) Recalculate horseshoes with kinematic velocity -----
            # Calculate kinematic velocity
            Vkin = Vtranslation(wake_system, prev_wake_system, DT; t=T,
                                                                targetX="CP")
            vlm._addsolution(wake_system, "Vkin", Vkin; t=T)

            # Recalculate horseshoes
            vlm.getHorseshoes(wake_system; t=T, extraVinf=extraVinf1)

            # ---------- 2) Paste previous Gamma solution ----------------------
            for i in 1:size(wake_system.wings, 1)
                wing = vlm.get_wing(wake_system, i)
                prev_wing = vlm.get_wing(prev_wake_system, i)

                if typeof(prev_wing) != vlm.Rotor
                    sol = deepcopy(prev_wing.sol["Gamma"])
                else
                    sol = deepcopy(prev_wing._wingsystem.sol["Gamma"])
                end

                vlm._addsolution(wing, "Gamma", sol; t=T)
            end

            # ---------- 3) Shed semi-infinite wake ----------------------------
            VLM2VPM(wake_system, PFIELD, DT, Vinf; t=T,
                            prev_system=prev_wake_system, unsteady_shedcrit=-1,
                            p_per_step=p_per_step, sigmafactor=sigmafactor,
                            overwrite_sigma=overwrite_sigma,
                            check=false)

            # ---------- 4) Calculate VPM velocity on VLM and Rotor system -----
            # V_on_VLM(PFIELD, T, DT; targetX="CP")
            Vvpm = Vvpm_on_VLM(PFIELD, wake_system, vpm_solver; targetX="CP")
            vlm._addsolution(wake_system, "Vvpm", Vvpm; t=T)

            # ---------- 5) Solve VLM system -----------------------------------
            # NOTE: Here is solving without VPM-induced velocity
            if wake_coupled
                vlm.solve(vlm_system, Vinf; t=T, extraVinf=extraVinf2, keep_sol=true,
                                                vortexsheet=(X,t)->zeros(3))
            else
                vlm.solve(vlm_system, Vinf; t=T, keep_sol=true)
            end

            # Relaxes (vlm_rlx->1) or stiffens (vlm_rlx->0) the VLM solution
            if vlm_rlx > 0
                rlxd_Gamma = vlm_rlx*vlm_system.sol["Gamma"] + (1-vlm_rlx)*prev_vlm_system.sol["Gamma"]
                vlm._addsolution(vlm_system, "Gamma", rlxd_Gamma)
            end

            # ---------- 5) Solve Rotor system ---------------------------------
            # for rotor in rotors
            #     RPM = cur_RPMs[rotor]
            #     # TODO: Add kinematic velocity
            #     # TODO: Add VPM-induced velocity and get rid of CCBlade
            #     vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; t=T, sound_spd=sound_spd,
            #                             Uinds=nothing, _lookuptable=false, _Vinds=nothing)
            # end
            # ---------- 6) Shed unsteady-loading wake with new solution -------
            if shed_unsteady
                VLM2VPM(wake_system, PFIELD, DT, Vinf; t=T,
                            prev_system=prev_wake_system, unsteady_shedcrit=0.01,
                            p_per_step=p_per_step, sigmafactor=sigmafactor,
                            overwrite_sigma=overwrite_sigma,
                            check=false)
            end
            # TODO: Verify that semi-infinite and unsteady wake is being placed half way the distance

        else # Case of first time step (pfield.nt==0)

            # Solve VLM system
            vlm.solve(vlm_system, Vinf; t=T)

            # # Solve Rotor system
            # rpms = RPMs(T/telapsed)
            # cur_RPMs = Dict()
            # for j in 1:size(rpms, 1)
            #     for rotor in rotors_systems[j]
            #         rpm = rpms[j] * (rand_RPM ? 1 + (rand()-0.5)*0.1 : 1.0)
            #         cur_RPMs[rotor] = RPMh_w*rpm
            #     end
            # end
            #
            # for rotor in rotors
            #     RPM = cur_RPMs[rotor]
            #     # TODO: Add kinematic velocity
            #       # NOTE: leave CCBlade for the deocupled case
            #     vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; t=T, sound_spd=sound_spd,
            #                             Uinds=nothing, _lookuptable=false, _Vinds=nothing)
            # end

        end

        breakflag = extra_runtime_function(PFIELD, T, DT)

        # Output vtks
        if save_path!=nothing && PFIELD.nt%nsteps_save==0
            vlm.save(system, run_name; save_horseshoes=save_horseshoes,
                                                path=save_path, num=PFIELD.nt)
            vlm.vtk.save(fuselage, run_name*"_FuselageGrid"; path=save_path,
                                                                num=PFIELD.nt)
        end
        return breakflag
    end

    ############################################################################
    # RUN SIMULATION
    ############################################################################
    # Here it uses the VPM-time-stepping to run the simulation
    vpm.run_vpm!(pfield, dt, nsteps; save_path=save_path, run_name=run_name,
                      verbose=verbose,
                      create_savepath=create_savepath,
                      runtime_function=runtime_function, solver_method=vpm_solver,
                      nsteps_save=nsteps_save,
                      save_code=save_code,
                      prompt=prompt,
                      nsteps_relax=nsteps_relax,
                      nsteps_restart=nsteps_restart,
                      save_sigma=true,
                      # static_particles_function=generate_static_particles,
                      # beta=cs_beta, sgm0=sgm0,
                      # rbf_ign_iterror=true,
                      # rbf_itmax=rbf_itmax, rbf_tol=rbf_tol
                      )
end




"""
Returns the local translational velocity of every control point in `system`.
"""
function Vtranslation(system, prev_system, dt; t=0.0, targetX="CP")

    cur_Xs = get_Xs(system, targetX; t=t)
    prev_Xs = get_Xs(prev_system, targetX; t=t)

    return [-(cur_Xs[i]-prev_Xs[i])/dt for i in 1:size(cur_Xs,1)]
end


"""
Returns the velocity induced by particle field on every position `Xs`
"""
function Vvpm_on_Xs(pfield, Xs::Array{T, 1}, vpm_solver) where {T}

    # Omit freestream
    Uinf = pfield.Uinf
    pfield.Uinf = (X, t)->zeros(3)

    # Evaluate velocity induced by particle field
    if vpm_solver=="ExaFMM"
        Vvpm = vpm.conv_ExaFMM(pfield; Uprobes=Xs)[2]
    else
        warn("Evaluating VPM-on-VLM velocity without FMM")
        Vvpm = [vpm.reg_Uomega(pfield, X) for X in Xs]
    end

    # Restore freestream
    pfield.Uinf = Uinf

    return Vvpm
end


"""
Returns the velocity induced by particle field on every control point
of `system`.
"""
function Vvpm_on_VLM(pfield, system, vpm_solver; targetX="CP")

    Xs = get_Xs(system, targetX; t=pfield.t)

    return Vvpm_on_Xs(pfield, Xs, vpm_solver)
end

function get_Xs(system, targetX::String; t=0.0)
    if targetX=="CP"
        Xs = [vlm.getControlPoint(system, i
                                        ) for i in 1:vlm.get_m(system)]
    else
        Xs = [vlm.getHorseshoe(system, i; t=t)[vlm.VLMSolver.HS_hash[targetX]
                                        ] for i in 1:vlm.get_m(system)]
    end
    return Xs
end

function get_Xs(system, targetXs::Array{T,1}; t=0.0) where{T}
    return vcat([get_Xs(system, targetX; t=t) for targetX in targetXs]...)
end




"""
Receives the FLOWVLM object `system` (Wing/WingSystem/Rotor), and adds vortex
particles to the particle field `pfield` at each trailing edge position where
an infinite vortex starts. `dt` indicates the length of the interval of time
that the vortex shedding represents.

Give it a previous system to detect differences in circulation and add
unsteady particles.
"""
function VLM2VPM(system, pfield, dt, Vinf;
                    t=0.0, prev_system=nothing, unsteady_shedcrit=-1.0,
                    p_per_step=1, sigmafactor=1.0, overwrite_sigma=nothing,
                    check=true, debug=false, tol=1e-6)

  if !(typeof(system) in [vlm.Wing, vlm.WingSystem, vlm.Rotor])
    error("Invalid system type $(typeof(system))")
  end

  m = vlm.get_m(system)   # Number of lattices

  # Velocity at horseshoe points Ap and Bp
  if prev_system==nothing
      Vinfs_Ap = vlm.getVinfs(system; t=t, target="Ap")
      Vinfs_Bp = vlm.getVinfs(system; t=t, target="Bp")
  else
      Vinfs_Ap = []
      Vinfs_Bp = []
      for i in 1:m
          Ap = vlm.getHorseshoe(system, i)[1]
          prev_Ap = vlm.getHorseshoe(prev_system, i)[1]
          push!(Vinfs_Ap, -(Ap-prev_Ap)/dt + Vinf(Ap, t))

          Bp = vlm.getHorseshoe(system, i)[4]
          prev_Bp = vlm.getHorseshoe(prev_system, i)[4]
          push!(Vinfs_Bp, -(Bp-prev_Bp)/dt + Vinf(Bp, t))
      end
  end

  # Adds a particle at each infinite vortex
  prev_HS = [nothing for i in 1:8]
  for i in 1:m  # Iterates over lattices
    HS = vlm.getHorseshoe(system, i)

    Ap, A, B, Bp, CP, infDA, infDB, Gamma = HS
    (prev_Ap, prev_A, prev_B, prev_Bp, prev_CP,
      prev_infDA, prev_infDB, prev_Gamma) = prev_HS
    cntgs = true          # Contiguous horseshoes flag

    if nothing in HS; error("Logic error! $HS"); end;
    if true in [isnan.(elem) for elem in HS]; error("Logic error! $HS"); end;

    # ----------- Case of left wing tip ---------------------------
    if i==1
      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma                             # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
      end

    # ----------- Case of wing tip on discontinuous wing --------
    elseif norm(Ap - prev_Bp) / norm(Bp - Ap) > tol
      if debug; println("Wing tip found at i=$i. $Ap!=$prev_Bp"); end;

      cntgs = false

      # Adds particle at previous Bp
      X = prev_Bp                               # Particle position
      gamma = -prev_Gamma                       # Infinite vortex circulation
      V = norm(Vinfs_Bp[i-1])                   # Freestream at X
      infD = -prev_infDB                        # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(prev_Bp-prev_Ap)/2)^2*V*dt # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
      end

      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma                             # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
      end

    # ----------- Case of contiguous horseshoes -----------------
    else

      # Verify logic
      if check
        crit1 = norm(Vinfs_Ap[i]-Vinfs_Bp[i-1]) / norm((Vinfs_Ap[i]+Vinfs_Bp[i-1])/2)
        crit2 = norm(infDA-prev_infDB) / norm((infDA+prev_infDB)/2)
        if crit1 > tol
          error("Logic error! Vinfs_Ap[i]!=Vinfs_Bp[i-1] "*
                  "( $(Vinfs_Ap[i])!=$(Vinfs_Bp[i-1]) )")
        elseif crit2 > tol
          error("Logic error! infDA!=prev_infDB "*
                  "( $(infDA)!=$(prev_infDB) )")
        elseif norm(infDA) - 1 > tol
          error("Logic error! norm(infDA)!= 1 "*
                  "( $(norm(infDA)) )")
        end
      end

      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma - prev_Gamma                # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
      end


      # ----------- Case of right wing tip --------------------------
      if i==m
        # Adds particle at Bp
        X = Bp                                    # Particle position
        gamma = -Gamma                            # Infinite vortex circulation
        V = norm(Vinfs_Bp[i])                     # Freestream at X
        infD = -infDB                             # Direction of vorticity
        sigma = sigmafactor*V*dt                  # Vortex blob radius
        vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
        l = -infD*V*dt                             # Distance the TE travels

        if unsteady_shedcrit<=0
            add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                            l, p_per_step; overwrite_sigma=overwrite_sigma)
        end
      end
    end

    # Adds unsteady particle
    if prev_system!=nothing && unsteady_shedcrit>0

      (p_Ap, p_A, p_B, p_Bp, p_CP,
          p_infDA, p_infDB, p_Gamma) = vlm.getHorseshoe(prev_system, i)
      X = (Ap+Bp)/2                               # LE midpoint position
      p_X = (p_Ap+p_Bp)/2                         # Previous LE midpoint position
      gamma = Gamma - p_Gamma                     # Bound circulation increase
      infD = (!cntgs || i==1 ? Ap : (prev_Ap+prev_Bp)/2) - X # Direction of circulation
      sigma = sigmafactor*norm(B-A)               # Vortex blob radius
      vol = 4/3*pi*(sigma/2)^3                    # Volume of particle
      l = -(X-p_X) + Vinf(X, t)*dt                # Distance the TE travels

      # Adds particle only if difference is greater than 1%
      if abs(gamma/p_Gamma) > unsteady_shedcrit
        add_particle(pfield, X, gamma, 1.0, 1.0, infD, sigma, vol,
                                        l, 1; overwrite_sigma=overwrite_sigma)
      end
    end

    prev_HS = HS
  end
end


function add_particle(pfield::vpm.ParticleField, X::Array{Float64, 1},
                        gamma::Float64, dt::Float64,
                        V::Float64, infD::Array{Float64, 1},
                        sigma::Float64, vol::Float64,
                        l::Array{T1, 1}, p_per_step::Int64;
                        overwrite_sigma=nothing) where {T1<:Real}

    Gamma = gamma*(V*dt)*infD       # Vectorial circulation

    # Decreases p_per_step for slowly moving parts of blade
    # aux = min((sigma/p_per_step)/overwrite_sigma, 1)
    # pps = max(1, min(p_per_step, floor(Int, 1/(1-(aux-1e-14))) ))
    pps = p_per_step

    if overwrite_sigma==nothing
        sigmap = sigma/pps
    else
        sigmap = overwrite_sigma
    end


    # Adds p_per_step particles along line l
    dX = l/pps
    for i in 1:pps
        particle = vcat(X + i*dX - dX/2, Gamma/pps, sigmap, vol/pps)
        vpm.addparticle(pfield, particle)
    end
end
