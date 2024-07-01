#=##############################################################################
# DESCRIPTION
    Simulation driver.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


"""

Run the FLOWUnsteady simulation `sim` in `nsteps` number of time steps.

```julia
run_simulation(

    sim::Simulation,                    # Simulation object
    nsteps::Int;                        # Total time steps in simulation

    # -------- SIMULATION OPTIONS -----------------------------------------
    Vinf            = (X, t)->zeros(3), # Freestream velocity
    sound_spd       = 343,              # (m/s) speed of sound
    rho             = 1.225,            # (kg/m^3) air density
    mu              = 1.81e-5,          # (Pa*s) air dynamic viscosity
    tquit           = Inf,              # (s) force quit the simulation at this time
    rand_RPM        = false,            # (experimental) randomize RPM fluctuations

    extra_runtime_function = (sim, PFIELD, T, DT; optargs...)->false,

    # -------- SOLVERS OPTIONS --------------------------------------------
    # Vortex particle method
    max_particles   = Int(1e5),         # Maximum number of particles
    max_static_particles = nothing,     # Maximum number of static particles (use `nothing` to automatically estimate it)
    p_per_step      = 1,                # Particle sheds per time step
    vpm_formulation = vpm.rVPM,         # VPM formulation (`vpm.rVPM` or `vpm.cVPM`)
    vpm_kernel      = vpm.gaussianerf,  # VPM kernel (`vpm.gaussianerf` or `vpm.winckelmans`)
    vpm_UJ          = vpm.UJ_fmm,       # VPM particle-to-particle interaction scheme (`vpm.UJ_fmm` or `vpm.UJ_direct`)
    vpm_SFS         = vpm.SFS_none,     # VPM LES subfilter-scale model (`SFS_none`, `SFS_Cd_threelevel_nobackscatter`, `SFS_Cd_twolevel_nobackscatter`, or `SFS_Cs_nobackscatter`)
    vpm_integration = vpm.rungekutta3,  # VPM time integration scheme (`vpm.euler` or `vpm.rungekutta3`)
    vpm_transposed  = true,             # VPM transposed stretching scheme
    vpm_viscous     = vpm.Inviscid(),   # VPM viscous diffusion scheme (`vpm.Inviscid()`, `vpm.CoreSpreading(nu, sgm0, zeta)`, or `vpm.ParticleStrengthExchange(nu)`)
    vpm_fmm         = vpm.FMM(; p=4, ncrit=50, theta=0.4, nonzero_sigma=false), # VPM's FMM settings
    vpm_relaxation  = vpm.pedrizzetti,  # VPM relaxation scheme (`vpm.norelaxation`, `vpm.correctedpedrizzetti`, or `vpm.pedrizzetti`)
    vpm_surface     = true,             # Whether to include surfaces in the VPM through ASM/ALM

    # Actuator surface/line model (ASM/ALM): VLM and blade elements
    vlm_vortexsheet = false,            # Whether to spread surface circulation as a vortex sheet in the VPM (turns ASM on; ALM if false)
    vlm_vortexsheet_overlap     = 2.125,# Overlap of particles that make the vortex sheet
    vlm_vortexsheet_distribution= g_pressure, # Vorticity distribution of vortex sheet (`g_uniform`, `g_linear`, or `g_pressure`)
    vlm_vortexsheet_sigma_tbv   = nothing, # Size of particles in trailing bound vortices (defaults to `sigma_vlm_surf` if not given)
    vlm_rlx         = -1,               # VLM relaxation (>0.9 can cause divergence, <0.2 slows simulation too much, deactivated with <0)
    vlm_init        = false,            # Initialize the first step with the VLM semi-infinite wake solution
    hubtiploss_correction = vlm.hubtiploss_nocorrection, # Hub and tip loss correction of rotors (ignored in quasi-steady solver)

    # Wake shedding
    wake_coupled        = true,         # Couple VPM wake -> VLM solution
    shed_unsteady       = true,         # Whether to shed vorticity from unsteady loading
    unsteady_shedcrit   = 0.01,         # Criterion for unsteady-loading shedding
    shed_starting       = false,        # Whether to shed starting vortex (only when `shed_unsteady=true`)
    shed_boundarylayer  = false,        # (experimental) whether to shed vorticity from boundary layer of surfaces
    boundarylayer_prescribedCd = 0.1,   # (experimental) prescribed Cd for boundary layer shedding used for wings
    boundarylayer_d     = 0.0,          # (experimental) dipole width for boundary layer shedding
    omit_shedding       = [],           # Indices of elements in `sim.vehicle.wake_system` on which omit shedding VPM particles

    # Regularization of solvers
    sigma_vlm_solver    = -1,           # Regularization of VLM solver (internal VLM-on-VLM)
    sigma_vlm_surf      = -1,           # (REQUIRED!) Size of embedded particles in ASM/ALM wing surfaces (for VLM-on-VPM and VLM-on-Rotor)
    sigma_rotor_surf    = -1,           # (REQUIRED!) Size of embedded particles in ALM blade surfaces (for Rotor-on-VPM, Rotor-on-VLM, and Rotor-on-Rotor)
    sigmafactor_vpm     = 1.0,          # Core overlap of wake particles
    sigmafactor_vpmonvlm = 1,           # (experimental) shrinks the particles by this factor when calculating VPM-on-VLM/Rotor induced velocities
    sigma_vpm_overwrite = nothing,      # Overwrite core size of wake to this value (ignoring `sigmafactor_vpm`)

    # -------- RESTART OPTIONS --------------------------------------------
    restart_vpmfile     = nothing,      # VPM restart file to restart simulation

    # -------- OUTPUT OPTIONS ---------------------------------------------
    save_path       = nothing,          # Where to save simulation
    run_name        = "flowunsteadysim",# Suffix of output files
    create_savepath = true,             # Whether to create `save_path`
    prompt          = true,             # Whether to prompt the user
    verbose         = true,             # Enable verbose
    v_lvl           = 0,                # Indentation level of verbose
    verbose_nsteps  = 10,               # Verbose every this many steps
    raisewarnings   = true,             # Whether to raise warnings
    debug           = false,            # Output extra states for debugging
    nsteps_save     = 1,                # Save vtks every this many steps
    nsteps_restart  = -1,               # Save jlds every this many steps (restart files)
    save_code       = "",               # Copy the source code in this path to `save_path`
    save_horseshoes = false,            # Whether to output VLM horseshoes in VTKs
    save_static_particles = true,       # Whether to save ASM/ALM embedded particles
    save_wopwopin   = false,            # Generate input files for PSU-WOPWOP

)
```


Even though a bast number of settings are exposed to the user, the only required
keyword arguments are `sigma_vlm_surf` and `sigma_rotor_surf`. Thus,
running the simulation can be as simple as

```julia
run_simulation(sim::Simulation, nsteps::Int;
                    sigma_vlm_surf = ..., sigma_rotor_surf = ...)
```
"""

function run_simulation(

           # sim::Simulation,                    # Simulation object
	    sim_cache,      
	    nsteps::Int;                        # Total time steps in simulation

            # -------- SIMULATION OPTIONS --------------------------------------
            Vinf            = (X, t)->zeros(3), # Freestream velocity
            sound_spd       = 343,              # (m/s) speed of sound
            rho             = 1.225,            # (kg/m^3) air density
            mu              = 1.81e-5,          # (Pa*s) air dynamic viscosity
            tquit           = Inf,              # (s) force quit the simulation at this time
            rand_RPM        = false,            # (experimental) randomize RPM fluctuations

            extra_runtime_function = (sim, PFIELD, T, DT; optargs...)->false, # See [`4)-Monitors-Definitions`](@ref)

            # -------- SOLVERS OPTIONS -----------------------------------------
            # Vortex particle method
            max_particles   = Int(1e5),         # Maximum number of particles
            max_static_particles = nothing,     # Maximum number of static particles (use `nothing` to automatically estimate it)
            p_per_step      = 1,                # Particle sheds per time step
            vpm_formulation = vpm.rVPM,         # VPM formulation (`vpm.rVPM` or `vpm.cVPM`)
            vpm_kernel      = vpm.gaussianerf,  # VPM kernel (`vpm.gaussianerf` or `vpm.winckelmans`)
            vpm_UJ          = vpm.UJ_fmm,       # VPM particle-to-particle interaction scheme (`vpm.UJ_fmm` or `vpm.UJ_direct`)
            vpm_SFS         = vpm.SFS_none,     # VPM LES subfilter-scale model (`SFS_none`, `SFS_Cd_threelevel_nobackscatter`, `SFS_Cd_twolevel_nobackscatter`, or `SFS_Cs_nobackscatter`)
            vpm_integration = vpm.rungekutta3,  # VPM time integration scheme (`vpm.euler` or `vpm.rungekutta3`)
            vpm_transposed  = true,             # VPM transposed stretching scheme
            vpm_viscous     = vpm.Inviscid(),   # VPM viscous diffusion scheme (`vpm.Inviscid()`, `vpm.CoreSpreading(nu, sgm0, zeta)`, or `vpm.ParticleStrengthExchange(nu)`)
            vpm_fmm         = vpm.FMM(; p=4, ncrit=50, theta=0.4, nonzero_sigma=false), # VPM's FMM settings
            vpm_relaxation  = vpm.pedrizzetti,  # VPM relaxation scheme (`vpm.norelaxation`, `vpm.correctedpedrizzetti`, or `vpm.pedrizzetti`)
            vpm_surface     = true,             # Whether to include surfaces in the VPM through ASM/ALM
            vpm_floattype   = Float64,

            # Actuator surface/line model (ASM/ALM): VLM and blade elements
            vlm_vortexsheet = false,            # Whether to spread surface circulation as a vortex sheet in the VPM (turns ASM on; ALM if false)
            vlm_vortexsheet_overlap     = 2.125,# Overlap of particles that make the vortex sheet
            vlm_vortexsheet_distribution= g_pressure, # Vorticity distribution of vortex sheet (`g_uniform`, `g_linear`, or `g_pressure`)
            vlm_vortexsheet_sigma_tbv   = nothing, # Size of particles in trailing bound vortices (defaults to `sigma_vlm_surf` if not given)
            vlm_rlx         = -1,               # VLM relaxation (>0.9 can cause divergence, <0.2 slows simulation too much, deactivated with <0)
            vlm_init        = false,            # Initialize the first step with the VLM semi-infinite wake solution
            hubtiploss_correction = vlm.hubtiploss_nocorrection, # Hub and tip loss correction of rotors (ignored in quasi-steady solver)

            # Wake shedding
            wake_coupled        = true,         # Couple VPM wake -> VLM solution
            shed_unsteady       = true,         # Whether to shed vorticity from unsteady loading
            unsteady_shedcrit   = 0.01,         # Criterion for unsteady-loading shedding
            shed_starting       = false,        # Whether to shed starting vortex (only when `shed_unsteady=true`)
            shed_boundarylayer  = false,        # (experimental) whether to shed vorticity from boundary layer of surfaces
            boundarylayer_prescribedCd = 0.1,   # (experimental) prescribed Cd for boundary layer shedding used for wings
            boundarylayer_d     = 0.0,          # (experimental) dipole width for boundary layer shedding
            omit_shedding       = [],           # Indices of elements in `sim.vehicle.wake_system` on which omit shedding VPM particles

            # Regularization of solvers
            sigma_vlm_solver    = -1,           # Regularization of VLM solver (internal VLM-on-VLM)
            sigma_vlm_surf      = -1,           # (REQUIRED!) Size of embedded particles in ASM/ALM wing surfaces (for VLM-on-VPM and VLM-on-Rotor)
            sigma_rotor_surf    = -1,           # (REQUIRED!) Size of embedded particles in ALM blade surfaces (for Rotor-on-VPM, Rotor-on-VLM, and Rotor-on-Rotor)
            sigmafactor_vpm     = 1.0,          # Core overlap of wake particles
            sigmafactor_vpmonvlm = 1,           # (experimental) shrinks the particles by this factor when calculating VPM-on-VLM/Rotor induced velocities
            sigma_vpm_overwrite = nothing,      # Overwrite core size of wake to this value (ignoring `sigmafactor_vpm`)

            # -------- RESTART OPTIONS -----------------------------------------
            restart_vpmfile     = nothing,      # VPM restart file to restart simulation

            # -------- OUTPUT OPTIONS ------------------------------------------
            save_path       = nothing,          # Where to save simulation
			toggle_save_vtk = true,				# Whether to save vtk output files
            run_name        = "flowunsteadysim",# Suffix of output files
            create_savepath = true,             # Whether to create `save_path`
            prompt          = true,             # Whether to prompt the user
            verbose         = true,             # Enable verbose
            v_lvl           = 0,                # Indentation level of verbose
            verbose_nsteps  = 10,               # Verbose every this many steps
            raisewarnings   = true,             # Whether to raise warnings
            debug           = false,            # Output extra states for debugging
            nsteps_save     = 1,                # Save vtks every this many steps
            nsteps_restart  = -1,               # Save jlds every this many steps (restart files)
            save_code       = "",               # Copy the source code in this path to `save_path`
            save_horseshoes = false,            # Whether to output VLM horseshoes in VTKs
            save_static_particles = true,       # Whether to save ASM/ALM embedded particles
            save_wopwopin   = false,            # Generate input files for PSU-WOPWOP

        )

	sim, extra_runtime_function = sim_cache(vpm_floattype)
	dt = sim.ttot/(nsteps-1)

    if wake_coupled==false && raisewarnings
        @warn("Running wake-decoupled simulation")
    end

    if shed_unsteady==false && raisewarnings
        @warn("Unsteady wake shedding is disabled!")
    end

    if sigma_vlm_surf<=0
        error("Received invalid vehicle surface regularization"*
                " (sigma_vlm_surf=$sigma_vlm_surf).")
    end

    if sigma_rotor_surf<=0
        error("Received invalid rotor surface regularization"*
                " (sigma_rotor_surf=$sigma_rotor_surf).")
    end

    if sigmafactor_vpmonvlm <= 0
        error("Invalid `sigmafactor_vpmonvlm` value (`sigmafactor_vpmonvlm=$(sigmafactor_vpmonvlm)`).")
    end

    ############################################################################
    # SOLVERS SETUP
    ############################################################################


    if sigma_vlm_solver<=0
        vlm.VLMSolver._blobify(false)
    else
        vlm.VLMSolver._blobify(true)
        vlm.VLMSolver._smoothing_rad(sigma_vlm_solver)
    end


    # ---------------- SCHEMES -------------------------------------------------
    # Set up viscous scheme
    if vpm.isinviscid(vpm_viscous) == false
        vpm_viscous.nu = mu/rho
        if vpm.iscorespreading(vpm_viscous) && sigma_vpm_overwrite!=nothing
            vpm_viscous.sgm0 = sigma_vpm_overwrite
        end
    end

    # Initiate particle field
    vpm_solver = [
                    (:formulation, vpm_formulation),
                    (:viscous, vpm_viscous),
                    (:kernel, vpm_kernel),
                    (:UJ, vpm_UJ),
                    (:SFS, vpm_SFS),
                    (:integration, vpm_integration),
                    (:transposed, vpm_transposed),
                    (:relaxation, vpm_relaxation),
                    (:fmm, vpm_fmm),
                 ]
    Xdummy = zeros(3)
    pfield = vpm.ParticleField(max_particles, vpm_floattype; Uinf=t->Vinf(Xdummy, t),
                                                                  vpm_solver...)

    max_staticp = max_static_particles==nothing ? 3*_get_m_static(sim.vehicle) : max_static_particles
    staticpfield = vpm.ParticleField(max_staticp, vpm_floattype; Uinf=t->Vinf(Xdummy, t),
                                                                  vpm_solver...)

    if vpm_surface && max_static_particles==nothing && vlm_vortexsheet && raisewarnings
        @warn("Vortex sheet representation of VLM has been requested, but "*
              "no `max_static_particles` has been provided. It will be set to "*
              "$(max_staticp) which may lead to particle overflow. Please "*
              "provide a higher `max_static_particles` in order to avoid "*
              "overflow.")
    end

    if restart_vpmfile!=nothing
        vpm.read!(pfield, restart_vpmfile; overwrite=true, load_time=false)
    end

    dt_lqr = 0.001
    window_lqr = 1
    dx_avg = zeros(7)
    u_bnds = [0.0 435.0; 0.0 435.0; 0.0 565.0]
    u_chng = [7.0, 1.5]*.2

    dx_d1 = zeros(7)
    x_d1 = zeros(7)
    x_filter = zeros(7)
    dx_filter = zeros(7)
    u_filter = [201.15602490219825
                200.75378536809873
                150.0]
#    u_filter = [154.0
 #               134.0
  #              159.0]

    # u_filter = [50.0537,    3.7369e-26,   170.967]

    sigma_u_1 = [0.001, 0.001, 0.01]
    sigma_u_2 = [0.015, 0.015, 0.015]
    window = 733
    x_data = zeros(7,window)
    dx_data = zeros(7,window)

    j = 0
    # f = open("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/output_file_linear_sigma.txt", "w")
    #     print(f, "")
    # close(f)

    get_value = (x)->begin
                if eltype(x) == Float64
                    return x
                else
                    return x.value
                end
            end


    A0 = [-0.00171334 -0.0     -0.0     -0.0 -0.0 -0.171217   -0.0
    0.0     -0.0403433   1.34745e-9  0.0  0.0  0.00417511  0.0
    0.0012942/6.88/1.5  -0.000371486/6.88/1.5 -8.77068e-9/6.88/1.5  0.0/6.88/1.5  0.0/6.88/1.5  6.48366e-12/6.88/1.5  0.0/6.88/1.5
    1.0     0.0      0.0     0.0  0.0  0.0      0.0
    0.0     1.0      0.0     0.0  0.0  0.0      0.0
    0.0     0.0      1.0     0.0  0.0  0.0      0.0
    -0.0829504  -3.40171    0.0     0.0  0.0  5.9371e-8   0.0]

    Af = [-0.0418141 -0.0273457 -0.000514883 -0.0 -0.0 -0.0612744 -0.0
    0.119318  -3.44298   0.0943827   0.0  0.0  4.20826   0.0
    -0.128936/6.88/1.5   4.5185/6.88/1.5    -0.611619/6.88/1.5    0.0/6.88/1.5   0.0/6.88/1.5  -5.52039/6.88/1.5    0.0/6.88/1.5 
    1.0     0.0     0.0      0.0  0.0  0.0     0.0
    0.0     1.0     0.0      0.0  0.0  0.0     0.0
    0.0     0.0     1.0      0.0  0.0  0.0     0.0
    -2.01608  -0.457086  0.0      0.0  0.0  0.558436  0.0]

    B0 = [-0.0    -0.0    0.00318955
    0.0488657  0.048768  0.0
    0.449674/6.88/1.5  -0.448775/6.88/1.5 -0.00240928/6.88/1.5
    0.0     0.0    0.0
    0.0     0.0    0.0
    0.0     0.0    0.0
    3.19684   3.18406  0.119691]

    Bf = [-0.000426102 -8.28577e-6  0.0148079
    0.0244114   0.000474692  0.000258473
    0.224673/6.88/1.5    -0.00436889/6.88/1.5   -0.0111871/6.88/1.5 
    0.0      0.0      0.0
    0.0      0.0      0.0
    0.0      0.0      0.0
    0.80034    0.000638717  0.915559]


    # A_matrices = DF.readdlm("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/vpm/A_matrices.txt", ' ')
    # B_matrices = DF.readdlm("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/vpm/B_matrices.txt", ' ')

    # u_prescribed = DF.readdlm("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/ctrl_inputs.txt", ' ')


    # A_matrix = [-0.118672  0.0727425 -0.0020318 -0.0 -0.0  -0.0698278 0.0
    #             0.330476 -3.58659   0.119771  0.0  0.0  3.44288 0.0
    #             0.375583 10.8196   -1.40345   0.0  0.0 -10.3861 0.0
    #             1.0    0.0     0.0     0.0  0.0  0.0 0.0
    #             0.0    1.0     0.0     0.0  0.0  0.0 0.0
    #             0.0    0.0     1.0     0.0  0.0  0.0 0.0
    #             -4.85777  -0.0750256  0.0     0.0  0.0  0.0720194  0.0]
    # B_matrix = [0.0362336  -0.00946937
    #             0.00801513  0.699808
    #             -0.114675  -12.4367
    #             0.0      0.0
    #             0.0      0.0
    #             0.0      0.0
    #             1.8231    0.0]
    end_steps = 100+70
    interval = 1
    n = window_lqr

    ############################################################################
    # SIMULATION RUNTIME FUNCTION
    ############################################################################

    """
        This function gets called by `vpm.run_vpm!` at every time step.
    """
    function runtime_function(PFIELD, T, DT; vprintln=(args...)->nothing)

#@show T

#@show 1

	if true
             i = Int(round(T/DT))
             # @show i
              #if i > 100
               #   error_stop_please
             # end
             n_spinup = sim.maneuver.nsteps_steady
             if i <= n_spinup
                 sim.maneuver.RPM = (u_filter[1]*60/2/pi*(i/n_spinup), u_filter[2]*60/2/pi*(i/n_spinup), u_filter[3]*60/2/pi*(i/n_spinup))
             end
             if i <= sim.maneuver.nsteps_steady
             #sim.maneuver.RPM = (u_filter[1]*60/2/pi,)
                 u = [sim.maneuver.RPM[1]*2*pi/60, sim.maneuver.RPM[2]*2*pi/60, sim.maneuver.RPM[3]*2*pi/60]
     #       u = [sim.maneuver.RPM[1]*2*pi/60]
                #  @show u Vinf(0, T)
                 if i > 10
                     for rotor_system in sim.vehicle.rotor_systems
                         for rotor in rotor_system
                             T, Q = vlm.calc_thrust_torque(rotor)
                            #  @show T
                         end
                     end
                 end
            end
#@show 2       
      if i > sim.maneuver.nsteps_steady+1
                 # vx, vz = sim.vehicle.V[[1,3]]
                 # theta_dot = sim.vehicle.W[2]
                 # rx, rz = sim.vehicle.system.O[[1,3]]
                 # axis = sim.vehicle.system.Oaxis    
                 # theta = -atand(axis[1,3], axis[1,1])
                 # x = [vx, vz, theta_dot, rx, rz, theta]
                 # if i == 1100
                 #     wing = sim.vehicle.vlm_system.wings[1]
                 #     tail = sim.vehicle.vlm_system.wings[2]
                 #     F_wing = wing.sol["Ftot"]
                 #     F_tail = tail.sol["Ftot"]
                 #     xn_wing, yn_wing, zn_wing = wing._xn, wing._yn, wing._zn
                 #     xn_tail, yn_tail, zn_tail = tail._xn, tail._yn, tail._zn
                 #     O_wing = wing.O
                 #     O_tail = tail.O 
                 #     Oaxis = sim.vehicle.system.Oaxis
                 #     @show F_wing F_tail xn_wing yn_wing zn_wing xn_tail yn_tail zn_tail O_wing O_tail Oaxis
                 # end
                 # if i == sim.maneuver.nsteps_steady + 50
                 #     error_stop_please
                # end
                 Fx = sim.vehicle.vlm_system.sol["Fx"]
                 Fz = sim.vehicle.vlm_system.sol["Fz"]
                 My = sim.vehicle.vlm_system.sol["My"]
                 F = [Fx, 0.0, Fz]
                 M = [0.0, My, 0.0]
                 istep = sim.maneuver.istep[] - sim.maneuver.nsteps_steady + 1
                 x = sim.maneuver.x_hist[:, istep]
                 dx = sim.maneuver.dx_hist[:, istep-1]
                 # for j in 1:window-1
                 #     x_data[:,j] = x_data[:,j+1]
                 #     dx_data[:,j] = dx_data[:,j+1]
                 # end
                 # x_data[:,end] = x
                 # dx_data[:,end] = dx
                 # if i < sim.maneuver.nsteps_steady+1+window
                 #     x_filter = x
                 #     dx_filter = dx
                 # else
                 #     for j in 1:length(x)
                 #         x_filter[j] = sum(x_data[j,:])/window
                 #         dx_filter[j] = sum(dx_data[j,:])/window
                 #     end
                 # end
                 # x_filter = 1/(2*sigma/dt + 1)*(x + x_d1 + x_filter*(2*sigma/dt-1))
                 # dx_filter = 1/(2*sigma/dt + 1)*(dx + dx_d1 + dx_filter*(2*sigma/dt-1))
                 u = [sim.maneuver.RPM[1]*2*pi/60, sim.maneuver.RPM[2]*2*pi/60, sim.maneuver.RPM[3]*2*pi/60]
      #            @show F M dx x u
                @show get_value.(x) 
                 # axis = sim.vehicle.system.Oaxis
                 # theta = -atand(axis[1,3], axis[1,1])
                 # d_theta = sim.vehicle.W[2]
                 # @show sim.vehicle.V sim.vehicle.W sim.vehicle.system.O theta

                #  f = open("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/output_file_linear_sigma.txt", "a")
                #      print(f, get_value(dx[1]), " ", get_value(dx[2]), " ", get_value(dx[3]), " ", get_value(x[1]), " ", get_value(x[2]), " ", get_value(x[3]), " ", get_value(x[4]), " ", get_value(x[5]), " ", get_value(x[6]), " ", get_value(x[7]), " ", get_value(u[1]), " ", get_value(u[2]), " ",  get_value(u[3]), "\n")
                #  close(f)
# @show 3
                 # @show xdot0
                 # xdot0 = [0.0, 0.0, 0.0, 45.0, 0.0, 0.0]
                  xdot0 = [-0.1498626359949432, 0.5401538717334969, -173.24419076868588, 50.0, 0.0, 0.0, 0.0]
                  if false #mod(i, Int(round(nsteps/n))) == 0
                      @show n-j
                      if n-j > end_steps
                          _, xp, up = FT.solve_lqr(A_matrix, B_matrix, sim.maneuver.R,
                                          sim.maneuver.Cf, x, u, dx,
                                          sim.maneuver.xf, sim.ttot - T, nt = n-j)
                      else
                          _, xp, up = FT.solve_lqr(A_matrix, B_matrix, sim.maneuver.R,
                                          sim.maneuver.Cf, x, u, dx,
                                          sim.maneuver.xf, DT*(end_steps-70)*interval, nt = end_steps-70)
                      end
                      u_new = up[:,1]
                      if u_new[1] > u_bnds[1,2] u_new[1] = u_bnds[1,2] end
                      if u_new[1] < u_bnds[1,1] u_new[1] = u_bnds[1,1] end
                      if u_new[2] > u_bnds[2,2] u_new[2] = u_bnds[2,2] end
                      if u_new[2] < u_bnds[2,1] u_new[2] = u_bnds[2,1] end
                      if u_new[3] > u_bnds[3,2] u_new[3] = u_bnds[3,2] end
                      if u_new[3] < u_bnds[3,1] u_new[3] = u_bnds[3,1] end       
                      dt_ctrl = dt*interval
                      u_filter = 1/(2*sigma/dt_ctrl + 1)*(u_new + u + u_filter*(2*sigma/dt_ctrl-1))
                     # du = u_new-u
                     # if abs(du[1]) > u_chng[1] u_new[1] = u[1] + u_chng[1]*sign(du[1]) end
                     # if abs(du[2]) > u_chng[2] u_new[2] = u[2] + u_chng[2]*sign(du[2]) end
                      sim.maneuver.RPM = (u_filter[1]*60/2/pi, )
                     sim.maneuver.angle = ([0.0, u_filter[2], 0.0], )
                      # @show up[:,end]
                      # if j == 0
                      #     @show up xp[1,:] xp[2,:] xp[5,:] xp[6,:]
                      # end
                      j += 1
                  end
	if mod(i, n) == 0
#@show 4
     #           s = JSON2.write(sim.vehicle)
     #           v = JSON2.read(s, )
                     # dx_filter = 1/(2*sigma/dt + 1)*(dx + dx_d1 + dx_filter*(2*sigma/dt-1))
                     index = 1#Int(trunc(round((i-sim.maneuver.nsteps_steady)/sim.maneuver.nsteps_dynamic * 3000)))
                    
                     # A_matrix = A_matrices[(index-1)*7 + 1:(index-1)*7 + 7,:]
                     # B_matrix = B_matrices[(index-1)*7 + 1:(index-1)*7 + 7,:]
                    A_matrix = A0
                    B_matrix = B0
                     # A_matrix = A0 .+ (i-sim.maneuver.nsteps_steady-2)/(nsteps - sim.maneuver.nsteps_steady-2)*(Af .- A0)
                     # B_matrix = B0 .+ (i-sim.maneuver.nsteps_steady-2)/(nsteps - sim.maneuver.nsteps_steady-2)*(Bf .- B0)

		            if eltype(dt_lqr) != eltype(DT) dt_lqr = eltype(DT)(dt_lqr) end                    
                    t_horizon = sim.maneuver.n_horizon*dt_lqr*interval
                    # nt = Int(round(t_horizon/dt_lqr))

                    # n = Int(round(dt_lqr/DT))
#@show 5
                    #  istep = sim.maneuver.istep[] - sim.maneuver.nsteps_steady
                    # @show sim.maneuver.t_hist[istep]+t_horizon
                     xf = sim.maneuver.x_ref(sim.maneuver.t_hist[istep-1]+t_horizon)

                     if eltype(dx_avg) != eltype(DT) dx_avg = zeros(eltype(DT), 7) end                    
                     for i in 1:length(dx_avg)
                        dx_avg[i] = sum(sim.maneuver.dx_hist[i, istep-window_lqr:istep-1])/window_lqr
                     end
                      @show get_value.(xf) get_value.(u)
               #     @show xf
                     _, xp, up = FT.solve_lqr(A_matrix, B_matrix, sim.maneuver.R,
                                     sim.maneuver.Cf, x, u, dx_avg,
                                    xf, t_horizon, nt = sim.maneuver.n_horizon)
#@show 6       
                    u_new = up[:,1]
                     if u_new[1] > u_bnds[1,2] u_new[1] = u_bnds[1,2] end
                     if u_new[1] < u_bnds[1,1] u_new[1] = u_bnds[1,1] end
                     if u_new[2] > u_bnds[2,2] u_new[2] = u_bnds[2,2] end
                     if u_new[2] < u_bnds[2,1] u_new[2] = u_bnds[2,1] end
                     if u_new[3] > u_bnds[3,2] u_new[3] = u_bnds[3,2] end
                     if u_new[3] < u_bnds[3,1] u_new[3] = u_bnds[3,1] end
     #                if i > sim.maneuver.nsteps_steady+200
     #                  sigma_u = 2     

                     if i > sim.maneuver.nsteps_steady+1500
                        #  sigma_u = [0.007, 0.005, 0.01]*(1 + (i - sim.maneuver.nsteps_steady-1500)/(nsteps - sim.maneuver.nsteps_steady-1500))
                        sigma_u = sigma_u_1 + (i - sim.maneuver.nsteps_steady-1500)/(nsteps - sim.maneuver.nsteps_steady-1500) * (sigma_u_2-sigma_u_1)
 		             elseif true#i > sim.maneuver.nsteps_steady+20
                         sigma_u = sigma_u_1#[0.015, 0.015, 0.015] # [0.007, 0.005, 0.01]
                     else
                         sigma_u = 0.005*ones(3)
                     end



                     if true #i <= sim.maneuver.nsteps_steady+40
                         u_filter = 1 ./(2*sigma_u/dt .+ 1) .*(u_new + u + u_filter .*(2*sigma_u/dt .- 1))
                     else
                         u_filter = u_new
                     end
                     # du = u_filter-u
                     # if abs(du[1]) > u_chng[1] u_filter[1] = u[1] + u_filter[1]*sign(du[1]) end
                     # if abs(du[2]) > u_chng[2] u_filter[2] = u[2] + u_filter[2]*sign(du[2]) end
     #                if i-sim.maneuver.nsteps_steady <= length(u_prescribed[:,1])
      #                   u_filter = u_prescribed[i-sim.maneuver.nsteps_steady,:]
       #              end
                     sim.maneuver.RPM = (u_filter[1]*60/2/pi, u_filter[2]*60/2/pi, u_filter[3]*60/2/pi)
                     # sim.maneuver.angle = ([0.0, u_filter[2], 0.0], )
                     # @show up[:,end]
                     # if j == 0
                     #     @show up xp[1,:] xp[2,:] xp[5,:] xp[6,:]
                     # end
                     j += 1
                     dx_d1 = deepcopy(dx)
                     x_d1 = deepcopy(x)
                 end
             end
         end

#@show 7


        
        # Move tilting systems, and translate and rotate vehicle
        nextstep_kinematic(sim, dt)

        # Solver-specific pre-calculations
        precalculations(sim, Vinf, PFIELD, T, DT)

        # Shed semi-infinite wake
        shed_wake(sim.vehicle, Vinf, PFIELD, DT, sim.nt; t=T,
                            unsteady_shedcrit=-1,
                            p_per_step=p_per_step, sigmafactor=sigmafactor_vpm,
                            overwrite_sigma=sigma_vpm_overwrite,
                            omit_shedding=omit_shedding)

        # Solve aerodynamics of the vehicle
        solve(sim, Vinf, PFIELD, wake_coupled, DT, vlm_rlx,
                sigma_vlm_surf, sigma_rotor_surf, rho, sound_spd,
                staticpfield, hubtiploss_correction;
                init_sol=vlm_init, sigmafactor_vpmonvlm=sigmafactor_vpmonvlm,
                debug=debug)

        # Shed unsteady-loading wake with new solution
        if shed_unsteady
            shed_wake(sim.vehicle, Vinf, PFIELD, DT, sim.nt; t=T,
                        unsteady_shedcrit=unsteady_shedcrit,
                        shed_starting=shed_starting,
                        p_per_step=p_per_step, sigmafactor=sigmafactor_vpm,
                        overwrite_sigma=sigma_vpm_overwrite,
                        omit_shedding=omit_shedding)
        end

        if shed_boundarylayer
            shed_wake(sim.vehicle, Vinf, PFIELD, DT, sim.nt; t=T,
                        unsteady_shedcrit=-1,
                        p_per_step=p_per_step, sigmafactor=sigmafactor_vpm,
                        overwrite_sigma=sigma_vpm_overwrite,
                        omit_shedding=omit_shedding,
                        shed_boundarylayer=true,
                        prescribed_Cd=boundarylayer_prescribedCd,
                        dipole_d=boundarylayer_d)
        end

        # Simulation-specific postprocessing
        breakflag = extra_runtime_function(sim, PFIELD, T, DT; vprintln=vprintln)

        # Output vtks
        if toggle_save_vtk && save_path!=nothing && PFIELD.nt%nsteps_save==0
            strn = save_vtk(sim, run_name; path=save_path,
                            save_horseshoes=save_horseshoes,
                            save_wopwopin=save_wopwopin)
        end



        breakflag2 = sim.t >= sim.ttot

        if breakflag2
            vprintln("Quitting time $(tquit) (s) has been reached. Simulation will now end.")
        end
#@show 7
        return breakflag || breakflag2
    end

    if vpm_surface
        static_particles_function = generate_static_particle_fun(pfield, staticpfield,
                                    sim.vehicle, sigma_vlm_surf, sigma_rotor_surf;
                                    vlm_vortexsheet=vlm_vortexsheet,
                                    vlm_vortexsheet_overlap=vlm_vortexsheet_overlap,
                                    vlm_vortexsheet_distribution=vlm_vortexsheet_distribution,
                                    vlm_vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv,
                                    save_path=save_static_particles ? save_path : nothing,
                                    run_name=run_name, nsteps_save=nsteps_save)
    else
        static_particles_function = (pfield, t, dt)->nothing
    end

    ############################################################################
    # RUN SIMULATION
    ############################################################################
    # Here it uses the VPM-time-stepping to run the simulation
	
    vpm.run_vpm!(pfield, dt, nsteps;
                      save_path=toggle_save_vtk ? save_path : nothing, run_name=run_name*"_pfield",
                      verbose=verbose, verbose_nsteps=verbose_nsteps,
                      v_lvl=v_lvl,
                      create_savepath=create_savepath,
                      runtime_function=runtime_function,
                      nsteps_save=nsteps_save,
                      save_code=save_code,
                      prompt=prompt,
                      static_particles_function=static_particles_function,
                      save_time=false
                      )

    # path = sim.maneuver.x_hist
    return pfield, sim
end





function add_particle(pfield::vpm.ParticleField{TVPM,<:Any,<:Any,<:Any,<:Any,<:Any,<:Any}, X,
                        gamma, dt,
                        V, infD,
                        sigma, vol,
                        l::Array{TF, 1}, p_per_step::Int64;
                        overwrite_sigma=nothing) where {TF,TVPM}

    Gamma = gamma*(V*dt)*infD       # Vectorial circulation

    # Avoid adding empty particles to the computational domain, or ExaFMM will
    # blow up
    if sqrt(Gamma[1]^2 + Gamma[2]^2 + Gamma[3]^2) <= 5*eps(TVPM)
        Gamma = 5*eps(TVPM)*ones(3)
    end

    circulation = max(abs(gamma), 5*eps(TVPM))

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
                            vol=vol/pps, circulation=circulation)
    end
end

"""
Returns the velocity induced by particle field on every position `Xs`
"""
function Vvpm_on_Xs(
        pfield::vpm.ParticleField{TF, <:Any, <:Any, <:Any, <:Any, <:Any, <:Any, <:Any, <:Any, <:Any},
        Xs; static_particles_fun=(args...)->nothing, dt=zero(TF), fsgm=1) where {TF}

    if length(Xs)!=0 && vpm.get_np(pfield)!=0
        # Omit freestream
        Uinf = pfield.Uinf
        # Warning: Commented out for compatibility with Duals
        # pfield.Uinf = (t)->zeros(TF, 3)


        org_np = vpm.get_np(pfield)             # Original particles

        # Singularize particles to correct tip loss
        # NOTE: This doesn't include static particles, but there shouldn't be
        #       any in the field at this point anyways
        if abs(fsgm) != 1
            for P in vpm.iterator(pfield)
                P.sigma .*= fsgm
            end
        end

        # Add static particles
        static_particles_fun(pfield, pfield.t, dt)


        sta_np = vpm.get_np(pfield)             # Original + static particles

        # Add probes
        for X in Xs
            add_probe(pfield, X)
        end

        # Evaluate velocity field
        # @show pfield.particles[1].U

        pfield.UJ(pfield)

        # @show "hello"
        # @show pfield.particles[1].U

        # Retrieve velocity at probes
        Vvpm = [Array(vpm.get_U(P)) for P in vpm.iterator(pfield; start_i=sta_np+1)]

        # Remove static particles and probes
        for pi in vpm.get_np(pfield):-1:(org_np+1)
            vpm.remove_particle(pfield, pi)
        end

        # De-singularize particles
        if abs(fsgm) != 1
            for P in vpm.iterator(pfield)
                P.sigma ./= fsgm
            end
        end

        # Restore freestream
        pfield.Uinf = Uinf
    else
        Vvpm = [zeros(TF, 3) for i in 1:length(Xs)]
    end

    return Vvpm
end

add_probe(pfield::vpm.ParticleField, X) = vpm.add_particle(pfield, X, zeros(3), 1e-6; vol=0)
