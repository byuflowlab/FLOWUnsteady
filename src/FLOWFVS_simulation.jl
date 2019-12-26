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
                             rand_RPM=true,             # Randomize RPM fluctuations
                             Vinf=(X,t)->zeros(3),      # Freestream velocity
                             sound_spd=343,             # (m/s) speed of sound
                             rho=1.225,                 # (kg/m^3) air density
                             mu=1.81e-5,                # Air dynamic viscosity
                             # SOLVERS OPTIONS
                             vpm_solver="ExaFMM",       # VPM solver
                             vpm_timesch="rk",          # VPM time stepping scheme
                             vpm_strtch="transpose",    # VPM stretching scheme
                             max_particles=Int(1e5),    # Maximum number of particles
                             nsteps_relax=1,            # Steps in between VPM relaxation
                             relaxfactor=0.3,           # VPM relaxation factor
                             p_per_step=1,              # Particle sheds per time step
                             sigmafactor=1.0,           # Particle core overlap
                             overwrite_sigma=nothing,   # Overwrite cores to this value (ignoring sigmafactor)
                             vlm_sigma=-1,              # VLM regularization
                             vlm_rlx=-1,                # VLM relaxation
                             vpm_surface=true,          # Whether to include surfaces in the VPM
                             surf_sigma=-1,             # Vehicle surface regularization
                             wake_coupled=true,         # Couple VPM wake on VLM solution
                             shed_unsteady=true,        # Whether to shed unsteady-loading wake
                             unsteady_shedcrit=0.01,    # Criterion for unsteady-loading shedding
                             extra_runtime_function=(sim, PFIELD,T,DT)->false,
                             # OUTPUT OPTIONS
                             save_path="temps/vahanasimulation00",
                             run_name="vahana",
                             create_savepath=true,      # Whether to create save_path
                             prompt=true,
                             verbose=true, v_lvl=1, verbose_nsteps=10,
                             nsteps_save=1,             # Save vtks every this many steps
                             nsteps_restart=-1,         # Save jlds every this many steps
                             save_code=module_path,     # Saves the source code in this path
                             save_horseshoes=true,     # Save VLM horseshoes
                             )


    if wake_coupled==false
        warn("Running wake-decoupled simulation")
    end

    if shed_unsteady==false
        warn("Unsteady wake shedding is off!")
    end

    if surf_sigma<=0
        error("Received invalid vehicle surface regularization"*
                " (surf_sigma=$surf_sigma).")
    end

    ############################################################################
    # SOLVERS SETUP
    ############################################################################
    dt = sim.ttot/nsteps            # (s) time step
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
    pfield = vpm.ParticleField(max_particles, Vinf, nothing, vpm_solver)
    pfield.nu = mu/rho                  # Kinematic viscosity

    # TODO: Add particle removal

    ############################################################################
    # SIMULATION RUNTIME FUNCTION
    ############################################################################

    """
        This function gets called by `vpm.run_vpm!` at every time step.
    """
    function runtime_function(PFIELD, T, DT)

        # Move tilting systems, and translate and rotate vehicle
        nextstep_kinematic(sim, dt)

        # Solver-specific pre-calculations
        precalculations(sim, Vinf, PFIELD, T, DT)

        # Shed semi-infinite wake
        shed_wake(sim.vehicle, Vinf, PFIELD, DT; t=T,
                            unsteady_shedcrit=-1,
                            p_per_step=p_per_step, sigmafactor=sigmafactor,
                            overwrite_sigma=overwrite_sigma)

        # Solve aerodynamics of the vehicle
        solve(sim, Vinf, PFIELD, wake_coupled, vpm_solver, DT, vlm_rlx,
                surf_sigma, rho, sound_spd)

        # Shed unsteady-loading wake with new solution
        if shed_unsteady
            shed_wake(sim.vehicle, Vinf, PFIELD, DT; t=T,
                        unsteady_shedcrit=unsteady_shedcrit,
                        p_per_step=p_per_step, sigmafactor=sigmafactor,
                        overwrite_sigma=overwrite_sigma)
        end

        # Simulation-specific postprocessing
        breakflag = extra_runtime_function(sim, PFIELD, T, DT)

        # Output vtks
        if save_path!=nothing && PFIELD.nt%nsteps_save==0
            strn = save_vtk(sim, run_name; path=save_path,
                                        save_horseshoes=save_horseshoes)
        end

        return breakflag
    end

    if vpm_surface
        static_particles_function = generate_static_particle_fun(sim.vehicle, surf_sigma)
    else
        static_particles_function = nothing
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
                      nsteps_restart=nsteps_restart>0 ? nsteps_restart : nsteps,
                      save_sigma=true,
                      static_particles_function=static_particles_function,
                      # beta=cs_beta, sgm0=sgm0,
                      # rbf_ign_iterror=true,
                      # rbf_itmax=rbf_itmax, rbf_tol=rbf_tol
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


"""
Returns the velocity induced by particle field on every position `Xs`
"""
function Vvpm_on_Xs(pfield::vpm.ParticleField, Xs::Array{T, 1},
                    vpm_solver::String; static_particles=Array{Float64,1}[]
                                                                    ) where {T}

    if length(Xs)!=0 && vpm.get_np(pfield)!=0
        # Omit freestream
        Uinf = pfield.Uinf
        pfield.Uinf = (X, t)->zeros(3)

        # Evaluate velocity induced by particle field
        if vpm_solver=="ExaFMM"
            # Vvpm = vpm.conv_ExaFMM(pfield; Uprobes=Xs)[2]
            Vvpm = vpm.conv(pfield, "ExaFMM"; Uprobes=Xs,
                                            static_particles=static_particles)[2]
        else
            warn("Evaluating VPM-on-VLM velocity without FMM")
            if length(static_particles)!=0
                error("$vpm_solver has no implementation with static particles.")
            end
            Vvpm = [vpm.reg_Uomega(pfield, X) for X in Xs]
        end

        # Restore freestream
        pfield.Uinf = Uinf
    else
        Vvpm = [zeros(3) for i in 1:length(Xs)]
    end

    return Vvpm
end
