#=##############################################################################
# DESCRIPTION
    Simulation driver.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################



function run_simulation(sim::Simulation, nsteps::Int;
                             # SIMULATION OPTIONS
                             rand_RPM=false,            # Randomize RPM fluctuations
                             Vinf=(X,t)->zeros(3),      # Freestream velocity
                             sound_spd=343,             # (m/s) speed of sound
                             rho=1.225,                 # (kg/m^3) air density
                             mu=1.81e-5,                # Air dynamic viscosity
                             # SOLVERS OPTIONS
                             vpm_formulation=vpm.formulation_sphere,
                             vpm_kernel=vpm.gaussianerf,# VPM kernel
                             vpm_UJ=vpm.UJ_fmm,         # VPM particle-to-particle interaction calculation
                             vpm_sgsmodel=vpm.sgs_none, # VPM LES subgrid scale model
                             vpm_sgsscaling=vpm.sgs_scaling_none, # Scaling of LES subgrid scale model
                             vpm_integration=vpm.rungekutta3, # VPM time integration scheme
                             vpm_transposed=true,       # VPM transposed stretching scheme
                             vpm_viscous=vpm.Inviscid(),# VPM viscous diffusion scheme
                             vpm_fmm=vpm.FMM(; p=4, ncrit=50, theta=0.4, phi=0.5), # VPM's FMM options
                             vpm_relaxation=vpm.pedrizzetti, # VPM relaxation scheme
                             vpm_relaxfactor=0.3,       # VPM relaxation factor
                             vpm_nsteps_relax=1,        # Steps in between VPM relaxation
                             vpm_surface=true,          # Whether to include surfaces in the VPM
                             max_particles=Int(1e5),    # Maximum number of particles
                             p_per_step=1,              # Particle sheds per time step
                             sigmafactor=1.0,           # Particle core overlap
                             overwrite_sigma=nothing,   # Overwrite cores to this value (ignoring sigmafactor)
                             vlm_sigma=-1,              # VLM regularization
                             vlm_fsgm=1,                # If !=1, replaces the tip loss correction with a singularization of particles by this factor
                             vlm_rlx=-1,                # VLM relaxation
                             vlm_init=false,            # Initialize the first step with the VLM semi-infinite wake solution
                             surf_sigma=-1,             # Vehicle surface regularization (for VLM-on-VPM, VLM-on-Rotor, and Rotor-on-VLM)
                             wake_coupled=true,         # Couple VPM wake on VLM solution
                             shed_unsteady=true,        # Whether to shed unsteady-loading wake
                             unsteady_shedcrit=0.01,    # Criterion for unsteady-loading shedding
                             omit_shedding=[],          # Indices of elements in `sim.vehicle.wake_system` on which omit shedding VPM particles
                             extra_runtime_function=(sim, PFIELD,T,DT)->false,
                             ground_point=[0.0,0,0],
                             ground_plane=[1.0,0,0],
                             ground_method::Ground=Panel(),
                             save_ground=true,
                             # OUTPUT OPTIONS
                             save_path="temps/vahanasimulation00",
                             run_name="FLOWUsimulation",
                             create_savepath=true,      # Whether to create save_path
                             prompt=true,
                             verbose=true, v_lvl=1, verbose_nsteps=10,
                             nsteps_save=1,             # Save vtks every this many steps
                             nsteps_restart=-1,         # Save jlds every this many steps
                             save_code="",              # Saves the source code in this path
                             save_horseshoes=false,     # Save VLM horseshoes
                             save_static_particles=true,# Whether to save particles to represent the VLM
                             save_wopwopin=true,        # Generate inputs for PSU-WOPWOP
                             )


    if wake_coupled==false
        @warn("Running wake-decoupled simulation")
    end

    if shed_unsteady==false
        @warn("Unsteady wake shedding is disabled!")
    end

    if surf_sigma<=0
        error("Received invalid vehicle surface regularization"*
                " (surf_sigma=$surf_sigma).")
    end

    if vlm_fsgm <= 0 && vlm_fsgm != -1
        error("Invalid `vlm_fsgm` value (`vlm_fsgm=$(vlm_fsgm)`).")
    end

    ############################################################################
    # SOLVERS SETUP
    ############################################################################
    dt = sim.ttot/nsteps            # (s) time step

    if vlm_sigma<=0
        vlm.VLMSolver._blobify(false)
    else
        vlm.VLMSolver._blobify(true)
        vlm.VLMSolver._smoothing_rad(vlm_sigma)
    end


    # ---------------- SCHEMES -------------------------------------------------
    # Set up viscous scheme
    if vpm.isinviscid(vpm_viscous) == false
        vpm_viscous.nu = mu/rho
        if vpm.iscorespreading(vpm_viscous) && overwrite_sigma!=nothing
            vpm_viscous.sgm0 = overwrite_sigma
        end
    end

    # Initiate particle field
    vpm_solver = [
                    (:formulation, vpm_formulation),
                    (:viscous, vpm_viscous),
                    (:kernel, vpm_kernel),
                    (:UJ, vpm_UJ),
                    (:sgsmodel, vpm_sgsmodel),
                    (:sgsscaling, vpm_sgsscaling),
                    (:integration, vpm_integration),
                    (:transposed, vpm_transposed),
                    (:relaxation, vpm_relaxation),
                    (:relax, vpm_relaxfactor != 0),
                    (:rlxf, vpm_relaxfactor),
                    (:fmm, vpm_fmm),
                 ]
    Xdummy = zeros(3)
    pfield = vpm.ParticleField(max_particles; Uinf=t->Vinf(Xdummy, t),
                                                                  vpm_solver...)
    staticpfield = vpm.ParticleField(_get_m_static(sim.vehicle);
                                         Uinf=t->Vinf(Xdummy, t), vpm_solver...)


    ############################################################################
    # SIMULATION RUNTIME FUNCTION
    ############################################################################

    """
        This function gets called by `vpm.run_vpm!` at every time step.
    """
    function runtime_function(PFIELD, T, DT; vprintln=(args...)->nothing)

        # Move tilting systems, and translate and rotate vehicle
        nextstep_kinematic(sim, dt)

        # Solver-specific pre-calculations
        precalculations(sim, Vinf, PFIELD, T, DT)

        # Shed semi-infinite wake
        shed_wake(sim.vehicle, Vinf, PFIELD, DT, sim.nt; t=T,
                            unsteady_shedcrit=-1,
                            p_per_step=p_per_step, sigmafactor=sigmafactor,
                            overwrite_sigma=overwrite_sigma,
                            omit_shedding=omit_shedding)

        # Solve aerodynamics of the vehicle
        solve(sim, Vinf, PFIELD, wake_coupled, DT, vlm_rlx,
                surf_sigma, rho, sound_spd, staticpfield;
                        init_sol=vlm_init, vlm_fsgm=vlm_fsgm)

        # Shed unsteady-loading wake with new solution
        if shed_unsteady
            shed_wake(sim.vehicle, Vinf, PFIELD, DT, sim.nt; t=T,
                        unsteady_shedcrit=unsteady_shedcrit,
                        p_per_step=p_per_step, sigmafactor=sigmafactor,
                        overwrite_sigma=overwrite_sigma,
                        omit_shedding=omit_shedding)
        end

        # Simulation-specific postprocessing
        breakflag = extra_runtime_function(sim, PFIELD, T, DT)

        # Output vtks
        if save_path!=nothing && PFIELD.nt%nsteps_save==0
            # return sim # sherlock
            strn = save_vtk(sim, run_name; path=save_path,
                            save_horseshoes=save_horseshoes,
                            save_wopwopin=save_wopwopin)
        end

        return breakflag
    end

    if vpm_surface
        static_particles_function = generate_static_particle_fun(pfield,
                                    sim.vehicle, surf_sigma;
                                    save_path=save_static_particles ? save_path : nothing,
                                    run_name=run_name)
    else
        static_particles_function = (pfield, t, dt)->nothing
    end

    if ground_effect
        ground_effect_function = generate_ground_effect_fun(ground_point, ground_plane, ground_method;
                                    save_ground, save_path=save_path,
                                    run_name=run_name)
    else
        ground_effect_function = (pfield, t, dt)->nothing
    end

    ############################################################################
    # RUN SIMULATION
    ############################################################################
    # Here it uses the VPM-time-stepping to run the simulation
    # return vpm.run_vpm!(pfield, dt, nsteps; # sherlock
    vpm.run_vpm!(pfield, dt, nsteps;
                      save_path=save_path, run_name=run_name*"_pfield",
                      verbose=verbose, verbose_nsteps=verbose_nsteps,
                      create_savepath=create_savepath,
                      runtime_function=runtime_function,
                      nsteps_save=nsteps_save,
                      save_code=save_code,
                      prompt=prompt,
                      nsteps_relax=vpm_nsteps_relax,
                      static_particles_function=static_particles_function,
                      ground_effect_function=ground_effect_function,
                      save_time=false
                      )

    return pfield
end





function add_particle(pfield::vpm.ParticleField, X::Array{Float64, 1},
                        gamma::Float64, dt::Float64,
                        V::Float64, infD::Array{Float64, 1},
                        sigma::Float64, vol::Float64,
                        l::Array{T1, 1}, p_per_step::Int64;
                        overwrite_sigma=nothing) where {T1<:Real}

    Gamma = gamma*(V*dt)*infD       # Vectorial circulation

    # Avoid adding empty particles to the computational domain, or ExaFMM would
    # blow up
    if sqrt(Gamma[1]^2 + Gamma[2]^2 + Gamma[3]^2) <= 5*eps(vpm.RealFMM)
        Gamma = 5*eps(vpm.RealFMM)*ones(3)
    end

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
        vpm.add_particle(pfield, X + i*dX - dX/2, Gamma/pps, sigmap;
                                                vol=vol/pps, circulation=max(abs(gamma), 5*eps(vpm.RealFMM)))
    end
end

"""
Returns the velocity induced by particle field on every position `Xs`
"""
function Vvpm_on_Xs(pfield::vpm.ParticleField, Xs::Array{T, 1}; static_particles_fun=(args...)->nothing, dt=0, fsgm=1) where {T}

    if length(Xs)!=0 && vpm.get_np(pfield)!=0
        # Omit freestream
        Uinf = pfield.Uinf
        pfield.Uinf = (t)->zeros(3)

        org_np = vpm.get_np(pfield)             # Original particles

        # Add static particles
        static_particles_fun(pfield, pfield.t, dt)

        # Singularize particles to correct tip loss
        if abs(fsgm) != 1
            for P in vpm.iterator(pfield)
                P.sigma .*= fsgm
            end
        end

        sta_np = vpm.get_np(pfield)             # Original + static particles

        # Add probes
        for X in Xs
            add_probe(pfield, X)
        end

        # Evaluate velocity field
        pfield.UJ(pfield)

        # Retrieve velocity at probes
        Vvpm = [Array(P.U) for P in vpm.iterator(pfield; start_i=sta_np+1)]

        # De-singularize particles
        if abs(fsgm) != 1
            for P in vpm.iterator(pfield)
                P.sigma ./= fsgm
            end
        end

        # Remove static particles and probes
        for pi in vpm.get_np(pfield):-1:(org_np+1)
            vpm.remove_particle(pfield, pi)
        end

        # Restore freestream
        pfield.Uinf = Uinf
    else
        Vvpm = [zeros(3) for i in 1:length(Xs)]
    end

    return Vvpm
end

add_probe(pfield::vpm.ParticleField, X) = vpm.add_particle(pfield, X, zeros(3), 1e-6; vol=0)
