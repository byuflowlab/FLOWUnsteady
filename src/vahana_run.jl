
function run_simulation(maneuver::Function, system, rotors, moving_main,
                             moving_tandem, fuselage, props_w, props_tw;
                             # SIMULATION OPTIONS
                             Vcruise=125*0.44704,       # Cruise speed
                             RPMh_w=400,                # RPM of main wing rotors in hover
                             telapsed=30.0,             # Total time to perform maneuver
                             nsteps=100,                # Time steps
                             rand_RPM=true,             # Randomize RPM fluctuations
                             Vinf=(x,t)->1e-5*[1,0,-1], # (m/s) freestream velocity
                             # OUTPUT OPTIONS
                             save_path=nothing,
                             run_name="vahana",
                             prompt=true,
                             verbose=true, v_lvl=1,
                             paraview=true)


    # ------------------ PARAMETERS --------------------------------------------
    R = rotors[1].rotorR                # (m) rotor radius
    speedofsound = 342                  # (m) speed of sound
    rho = 1.225                         # (kg/m^3) air density
    mu = 1.81e-5                        # (kg/ms) air dynamic viscosity
    sound_spd = speedofsound


    # RPMh_w = Vtip * 60/(2*pi*R)         # RPM in main wing in hover
    Vtip = 2*pi*RPMh_w/60*R           # Velocity of blade tip in main wing in hover
    Mtip = Vtip/speedofsound
    revs_per_sec = RPMh_w/60

    println("\t"^(v_lvl)*"Tip Mach:\t$(round(Mtip, 2))")
    println("\t"^(v_lvl)*"RPM:\t\t$(ceil(Int, RPMh_w))")
    println("\t"^(v_lvl)*"Revs per sec:\t$(round(revs_per_sec, 1))")



    # ------------------ SIMULATION OPTIONS ------------------------------------
    dt = telapsed/nsteps            # (s) time step
    p_per_step = 4                  # Particle sheds per time step
    sigmafactor = 2.125*4           # Blade tip core overlap
    steady = false                  # If false, it sheds unsteady particles
    # steady = true
    max_wake_shedding = 5.86*3/2    # (m) sphere for wake removal
    wake_removal_O = zeros(3)       # Center of sphere
    lifting_surface = true          # Include lifting surface induce velocity
    wake_coupled = true             # Whether to couple rotors with wake
    # wake_coupled = false
    if !wake_coupled; println("*********************** DECOUPLED WAKE! ******"); end;
    coupling_method = "noniterative"# Coupling method
    nsteps_del = 20                 # Deletes particles every this many steps

    vlm_sigma = 0.1*R               # VLM smoothing of self-induced blade velocity
    lf_sigma = (1/50)*R             # VLM smoothing of velocity on particles

    nsteps_relax = 1                # Steps in between relaxation
    # nsteps_relax = -1
    relaxfactor = 0.3               # Relaxation factor (0=no relaxation)

    # Viscous scheme
    visc_pse = false                # Viscous scheme: Particle-strength exchange
    visc_cs = true                  # Viscous scheme: Core spreading
    cs_beta = 5*1.00025             # Core spreading reinitialization criteria
    # cs_beta = 1.00025
    # cs_beta = 1.000125
    rbf_itmax = 100                 # Maximum RBF iterations
    rbf_tol = 5e-2                  # RBF tolerance

    # ExaFMM options
    P2PType = 5                     # P2P kernel (1=Singular, 3=Winckelmans, 5=GausErf)
    vpm.set_P(Int32(4))             # Multipole expansion order
    vpm.set_NCRIT(Int32(50))        # Max number of bodies per leaf
    vpm.set_THETA(0.4)              # Neighborhood criteria
    vpm.set_PHI(1/3)                # Regularizing neighborhood criteria


    # ------------------ SIMULATION SETUP --------------------------------------
    # Overwrite all smoothing radii to this value
    global overwrite_sigma = (2*pi*RPMh_w/60)*dt*R / p_per_step
    sgm0 = sigmafactor*overwrite_sigma          # RBF reset sigma


    vlm.setVinf(system, Vinf)

    Vaircraft, angles, RPMs = maneuver(; verbose=verbose, v_lvl=v_lvl+1, vis_nsteps=nsteps)

    prev_angles = Float64[angles(0)...]
    O_fuselage = zeros(3)

    println("\t"^(v_lvl)*"Rotation per step:\t$(round(360*revs_per_sec*dt, 2)) deg")

    # Function that applies the kinematics to wings and fuselage
    function extraruntime_function(PFIELD, T, DT, ROTORS, angles_deg, NSTEPS, nstep)

        # Step in time
        if nstep!=0
            t = T

            # Translation
            V = Vcruise * Vaircraft(t/telapsed)
            dX = V*DT

            vlm.setcoordsystem(system, system.O + dX, system.Oaxis)
            vlm.vtk.lintransform!(fuselage, eye(3), dX)

            O_fuselage .+=  dX

            # Rotation of aircraft and lifting surfaces
            new_angles = [angles(t/telapsed)...]
            dangles = new_angles - prev_angles

            Oaxis_main = vlm.vtk.rotation_matrix2(0, -dangles[1], 0)
            Oaxis_tandem = vlm.vtk.rotation_matrix2(0, -dangles[2], 0)
            Oaxis_aircraft = vlm.vtk.rotation_matrix2(0, -dangles[3], 0)

            vlm.setcoordsystem(moving_main, moving_main.O, Oaxis_main*moving_main.Oaxis)
            vlm.setcoordsystem(moving_tandem, moving_tandem.O, Oaxis_tandem*moving_tandem.Oaxis)
            vlm.setcoordsystem(system, system.O, Oaxis_aircraft*system.Oaxis)

            vlm.vtk.lintransform!(fuselage, eye(3), -O_fuselage)
            vlm.vtk.lintransform!(fuselage, Oaxis_aircraft, O_fuselage)

            prev_angles[:] = new_angles

            wake_removal_O[:] = system.O + 5.86/2*system.Oaxis[1, :]

        end

        # Output vtks
        if save_path!=nothing
            vlm.save(system, run_name; save_horseshoes=true, path=save_path, num=nstep)
            vlm.vtk.save(fuselage, run_name*"_FuselageGrid"; path=save_path, num=nstep)
        end
    end

    # Hash between rotor and wing number
    hash_rotor_rpmi = Dict(vcat([(prop, 1) for prop in props_w],
                                [(prop, 2) for prop in props_tw]))

    # Angular velocity of main and tandem wing rotors
    function omegas(rotor, t)
        # Main and tandem RPM
        rpms = RPMs(t/telapsed)
        # Wing number
        rpmi = hash_rotor_rpmi[rotor]
        # Normalized RPM
        rpm = rpms[rpmi] * (rand_RPM ? 1 + (rand()-0.5)*0.1 : 1.0)
        # Actual RPM
        RPM = RPMh_w*rpm

        return 2*pi*RPM/60
    end

    vehicle_simulation(rotors, Vinf, omegas, nsteps, dt;
                        # Outputs
                        save_path=save_path, run_name=run_name,
                        create_savepath=true,
                        prompt=prompt, paraview=paraview,
                        verbose=verbose, v_lvl=v_lvl+1,
                        save_code=joinpath(this_module_path, "../"),
                        # Simulation parameters
                        rho=rho, mu=mu, sound_spd=sound_spd,
                        # Options
                        steady=steady,
                        p_per_step=p_per_step,
                        max_wake_shedding=max_wake_shedding,
                        wake_removal_O=wake_removal_O,
                        nsteps_del=nsteps_del,
                        nsteps_save=-1, nsteps_save_restart=nothing,
                        extraruntime_function=extraruntime_function,
                        # Particle field solver
                        sigmafactor=sigmafactor,
                        nsteps_relax=nsteps_relax, relaxfactor=relaxfactor, pse=visc_pse,
                        lifting_surface=lifting_surface,
                        vlm_sigma=vlm_sigma, lf_sigma=lf_sigma,
                        wake_coupled=wake_coupled, coupling_method=coupling_method,
                        plot_convergence=false,
                        P2PType=P2PType,
                        visc_cs=visc_cs, cs_beta=cs_beta, sgm0=sgm0,
                        rbf_itmax=rbf_itmax, rbf_tol=rbf_tol)
end
