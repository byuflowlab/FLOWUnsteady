#=##############################################################################
# DESCRIPTION
    Simulation postprocessing.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Mar 2021
  * License   : MIT
=###############################################################################


"""
Generate a grid around the vehicle where velocity and vorticity fields are
probed and outputted as VTK files.
"""
function postprocessing_fluiddomain(read_path, save_path, nums;
                                    # PROCESSING OPTIONS
                                    fdx=1/80,                # Scaling of cell length
                                    static_particles=false,  # Add static particles
                                    fsgm=1.0,                # Scaling of smoothing radii
                                    maxparticles=Int(2e6),
                                    # SIMULATION INFORMATION
                                    ttot=30.0,
                                    nsteps=4*5400,
                                    tstart=0,                # Starting time of simulation
                                    start_kinmaneuver=false,
                                    Vref=0.25 * 125*0.44704,
                                    # OUTPUT OPTIONS
                                    run_name="vahana2",
                                    prompt=true, paraview=false, debug=false,
                                    verbose=true, v_lvl=0)

    # fsgm          = 0.35                             # Scaling of smoothing radii

    # Maneuver parameters
    # ttot          = 30.0                             # Total time to perform maneuver
    # nsteps        = 4*5400                           # Time steps of complete maneuver
    dt            = ttot/nsteps
    # Vref          = 0.25 * 125*0.44704               # Reference velocity magnitude

    maneuver      = generate_maneuver_vahana()

    t0 = tstart*!start_kinmaneuver

    # Functions for calculating position and orientation of vehicle over time
    V(t)          = Vref*maneuver.Vvehicle(t/ttot)
    Xt(t)         = QuadGK.quadgk(V, t0, t0+t, rtol=1e-3)[1]
    Xn(n)         = Xt(n*dt)
    angles(n)     = maneuver.anglevehicle(tstart/ttot + n/nsteps)

    # ----------------- SETUP FLUID DOMAIN GRID ----------------------------------------
    b             = 5.86                             # Some characteristic length
    L             = 1.6*b                            # Length of fluid domain
    dx            = fdx*b                            # Cell length

    Lx, Ly, Lz    = 1.2*L, L, 0.6*L                  # Length in each direction
    dx, dy, dz    = dx, 1*dx, 0.5*dx                 # Cell length in each direction
    nx, ny, nz    = Lx/dx, Ly/dy, Lz/dz              # Number of cells in each direction

    X0            = [0.35*L, 0, 0.125*L]              # Center of fluid domain

    P_min         = -[Lx, Ly, Lz]/2
    P_max         = [Lx, Ly, Lz]/2
    NDIVS         = Int.(ceil.([nx, ny, nz]))

    file_pref = run_name*"_pfield"
    other_file_prefs = static_particles ? [run_name*"_staticpfield"] : []
    other_read_paths = static_particles ? [read_path] : []
    file_pref_out = run_name*"-fdom-"

    # origin(pfield, num) = X0 + Xn(num) #NOTE: UNCOMMENT THIS
    origin(pfield, num) = X0
    orientation(pfield, num) = gt.rotation_matrix(-angles(num)...)

    evaluate_fluiddomain_vtk(P_min, P_max, NDIVS, maxparticles,
                             nums, read_path, file_pref;
                                        # O=X0,
                                        origin=origin,
                                        orientation=orientation,
                                        scale_sigma=fsgm,
                                        other_file_prefs=other_file_prefs,
                                        other_read_paths=other_read_paths,
                                        verbose=verbose, v_lvl=v_lvl,
                                        debug=debug, save_path=save_path,
                                        file_pref=file_pref_out
                                        )

    # # Genereate fluid domain grid
    # if verbose; println("\t"^v_lvl*"Defining fluid domain..."); end;
    # grid_fdom     = gt.Grid(P_min, P_max, NDIVS)
    #
    # if verbose; println("\t"^(v_lvl+1)*"Number of nodes:\t$(grid_fdom.nnodes)"); end;
    #
    #
    # # Visualize computational and fluid domain
    # str           = save_path
    #
    # if debug
    #     if verbose; println("\t"^v_lvl*"Saving grid..."); end;
    #     str *= gt.save(grid_fdom, "fdom.$(num)"; path=save_path)
    # end
    #
    #
    # # ----------------- CALCULATE FLUID DOMAIN -----------------------------------------
    # settings_fname = run_name*"_pfield_settings.jld"   # Partiel field settings file
    # maxparticles   = grid_fdom.nnodes + maxvpmparticles# Maximum number of particle in field
    #
    # UJ             = vpm.UJ_fmm                      # Method for UJ evaluation
    #
    # overwrite_settings = (
    #                         (:maxparticles, maxparticles),
    #                         (:sgsmodel, vpm.sgs_none),
    #                         (:Uinf, (args...) -> zeros(3)),
    #                         (:sgsscaling, vpm.sgs_scaling_none)
    #                      )
    #
    # # Generate particle field
    # pfield = vpm.generate_particlefield(settings_fname;
    #                                         path=read_path,
    #                                         overwrite_settings=overwrite_settings)
    #
    # prevT = zeros(3)
    # prevM = Float64[i==j for i in 1:3, j in 1:3]
    #
    #
    # if verbose; println("\t"^(v_lvl)*"Iterating over steps"); end;
    # for (numi, num) in enumerate(nums)
    #
    #     if verbose
    #         println("\t"^(v_lvl+1)*"Time step: $(num)\t($(numi) out of $(length(nums)))")
    #     end
    #
    #     # # Bring grid back to origin
    #     # gt.lintransform!(grid_fdom, Float64[i==j for i in 1:3, j in 1:3], -prevT; reset_fields=true)
    #     #
    #     # # Reorient grid with global axes
    #     # gt.lintransform!(grid_fdom, collect(prevM'), zeros(3); reset_fields=true)
    #     #
    #     # # Bring grid back to origin and reorient with global axes at the same time
    #     # gt.lintransform!(grid_fdom, collect(prevM'), -prevM*prevT; reset_fields=true)
    #     #
    #     # # Translate and rotate grid
    #     # this_X = Xn(num)
    #     # T = X0 + this_X                           # Translation of fluid domain
    #     # M = gt.rotation_matrix(-angles(num)...)   # Orientation of fluid domain
    #     # gt.lintransform!(grid_fdom, M, T; reset_fields=true)
    #
    #     # Bring grid back to origin and reorient with global axes
    #     # and translate and rotate to new position; all at the same time
    #     T = X0 + Xn(num)                          # Translation of fluid domain
    #     M = gt.rotation_matrix(-angles(num)...)   # Orientation of fluid domain
    #     gt.lintransform!(grid_fdom, collect(prevM')*M,
    #                             T - collect(M')*prevM*prevT; reset_fields=true)
    #
    #     prevT, prevM = T, M
    #
    #     # Read particle field
    #     vpm.read!(pfield, run_name*"_pfield.$(num).h5";
    #                                             path=read_path, overwrite=true)
    #
    #     # Read embedded particles
    #     if static_particles
    #         vpm.read!(pfield, run_name*"_staticpfield.$(num).h5";
    #                                             path=read_path, overwrite=false)
    #     end
    #
    #     np = vpm.get_np(pfield)
    #
    #     # Rescale smoothing radii
    #     meansigma = 0
    #     meannormGamma = 0
    #     for P in vpm.iterate(pfield)
    #         P.sigma[1] *= fsgm
    #         meansigma += P.sigma[1]
    #         meannormGamma += sqrt(P.Gamma[1]^2 + P.Gamma[2]^2 + P.Gamma[3]^2)
    #     end
    #     meansigma /= np
    #     meannormGamma /= np
    #
    #     # Add grid nodes to the particle field
    #     Gamma = 1e-4*meannormGamma*ones(3)
    #     sigma = 0.5*meansigma
    #
    #     for ni in 1:grid_fdom.nnodes
    #         vpm.add_particle(pfield, view(grid_fdom.nodes, 1:3, ni), Gamma, sigma)
    #     end
    #
    #     # Evaluate UJ
    #     vpm._reset_particles(pfield)
    #     t = @elapsed UJ(pfield)
    #     if verbose; println("\t"^(v_lvl+2)*"Evaluate UJ:\t$(round(t, digits=1)) s"); end;
    #
    #     t = @elapsed begin
    #     Us = collect(vpm.get_U(pfield, i) for i in np+1:vpm.get_np(pfield))
    #     Ws = collect(vpm.get_W(pfield, i) for i in np+1:vpm.get_np(pfield))
    #
    #     gt.add_field(grid_fdom, "U", "vector", Us, "node")
    #     gt.add_field(grid_fdom, "W", "vector", Ws, "node")
    #
    #     # Save fluid domain
    #     gt.save(grid_fdom, "fdom.$(num)"; path=save_path, time=num)
    #     end
    #
    #     if verbose; println("\t"^(v_lvl+2)*"Save VTK:\t$(round(t, digits=1)) s"); end;
    #
    #     if debug
    #         str *= vpm.save(pfield, "debug_pfield"; path=save_path)
    #     end
    # end
    #
    # str *= "fdom...vtk"
    #
    # if paraview || debug
    #     run(`paraview --data=$(str)`)
    # end
    #
    # return str
end

"""
Generate inputs for PSU-WOPWOP and run it.
"""
function postprocessing_aeracoustics()

    # Path from where to read aerodynamic solution
    # read_path       = "temps/vahana2_sim16/"
    read_path       = "/home/flowlab/ealvarez/simulations/vahana2_sim16/"
    # Path where to save PSU-WOPWOP's outputs
    save_path       = "/home/flowlab/ealvarez/simulations/vahana2_sim16-psw00/"
    # Path to PSU-WOPWOP binary (not included in FLOWUnsteady)
    wopwopbin       = "/home/flowlab/ealvarez/code/wopwop3_serial"
    # Run name (prefix of rotor files to read)
    run_name        = "vahana2"

    # ------------ PARAMETERS --------------------------------------------------
    # NOTE: Make sure that these parameters match what was used in the
    #       aerodynamic solution.

    # rotorsystems[si][ri] is the number of blades of the ri-th rotor in the si-th system
    rotorsystems    = [[5, 5], [2, 2], [2, 2], [5, 5, 5, 5]]
    nsteps          = 6480-4               # when the aero solution ended
    tend            = nsteps/21600 * 30.0  # (s) when the aero solution ended



    # Simulation parameters
    RPM             = 60                   # RPM is just a reference value to go from nrevs to simulation time
    rho             = 1.225                # (kg/m^3) air density
    speedofsound    = 342.35               # (m/s) speed of sound

    # Solver parameters
    ww_nrevs        = tend*(RPM/60)        # Number of revolutions in PSU-WOPWOP
    ww_nsteps_per_rev = nsteps/ww_nrevs    # Number of steps per revolution in PSU-WOPWOP
    num_min         = 0                    # Start reading aero files from this number
    periodic        = false                # Periodic aerodynamic solution
    const_geometry  = false                # Whether to run PSW on constant geometry from num_min
    CW              = nothing              # Clock-wise rotation of constant geometry
    highpass        = 0*0.0001             # High pass filter. Set to >0 to get rid of 0th freq in OASPL

    # Observer definition: Circular array of microphones
    sph_R           = 1.905                # (m) radial distance from rotor hub
    sph_nR          = 0
    sph_nphi        = 0
    sph_ntht        = 72                   # Number of microphones
    sph_thtmin      = 0                    # (deg) first microphone's angle
    sph_thtmax      = 360                  # (deg) last microphone's angle
    sph_phimax      = 180
    sph_rotation    = [90, 0, 0]           # Rotation of grid of microphones
    obs_name = "circle_mic_array"          # Observer file name

    # Observer definition: Single microphone
    Rmic = 0*1.905                         # (m) radial distance from rotor hub
    anglemic = 90*pi/180                   # (rad) microphone angle from plane of rotation (- below, + above)
                                           # 0deg is at the plane of rotation, 90deg is upstream
    # microphoneX = nothing                  # Comment and uncomment this to switch from array to single microphone
    microphoneX = Rmic*[-sin(anglemic), cos(anglemic), 0]


    # ------------ RUN PSU-WOPWOP ----------------------------------------------
    uns.run_noise_wopwop(read_path, run_name, RPM, rho, speedofsound, rotorsystems,
                            ww_nrevs, ww_nsteps_per_rev, save_path, wopwopbin;
                            # ---------- OBSERVERS -------------------------
                            sph_R=sph_R,
                            sph_nR=sph_nR, sph_ntht=sph_ntht,
                            sph_nphi=sph_nphi, sph_phimax=sph_phimax,
                            sph_rotation=sph_rotation,
                            sph_thtmin=sph_thtmin, sph_thtmax=sph_thtmax,
                            microphoneX=microphoneX,
                            # ---------- SIMULATION OPTIONS ----------------
                            periodic=periodic,
                            # ---------- INPUT OPTIONS ---------------------
                            num_min=num_min,
                            const_geometry=const_geometry,
                            axisrot="automatic",
                            CW=CW,
                            highpass=highpass,
                            # ---------- OUTPUT OPTIONS --------------------
                            verbose=true, v_lvl=0,
                            prompt=false, debug_paraview=false,
                            debuglvl=0,                     # WW debug level
                            observerf_name="observergrid",  # .xyz file with observer grid
                            case_name="runcase",            # Name of case to create and run
                            );
end
