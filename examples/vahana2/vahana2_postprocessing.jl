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
                                    # OUTPUT OPTIONS
                                    run_name="vahana2",
                                    prompt=true, paraview=false, debug=false,
                                    verbose=true, v_lvl=0)

    fsgm          = 0.35                             # Scaling of smoothing radii

    # Maneuver parameters
    ttot          = 30.0                             # Total time to perform maneuver
    nsteps        = 4*5400                           # Time steps of complete maneuver
    dt            = ttot/nsteps
    Vref          = 0.25 * 125*0.44704               # Reference velocity magnitude

    maneuver      = generate_maneuver_vahana()

    # Functions for calculating position and orientation of vehicle over time
    V(t)          = Vref*maneuver.Vvehicle(t/ttot)
    Xt(t)         = QuadGK.quadgk(V, 0, t, rtol=1e-3)[1]
    Xn(n)         = Xt(n*dt)
    angles(n)     = maneuver.anglevehicle(n/nsteps)

    # ----------------- SETUP FLUID DOMAIN GRID ----------------------------------------
    b             = 5.86                             # Some characteristic length
    L             = 1.4*b                            # Length of fluid domain
    dx            = fdx*b                            # Cell length

    Lx, Ly, Lz    = 1.0*L, L, 0.5*L                  # Length in each direction
    dx, dy, dz    = dx, 1*dx, 0.5*dx                 # Cell length in each direction
    nx, ny, nz    = Lx/dx, Ly/dy, Lz/dz              # Number of cells in each direction

    X0            = [0.4*L, 0, 0.125*L]              # Center of fluid domain

    P_min         = -[Lx, Ly, Lz]/2
    P_max         = [Lx, Ly, Lz]/2
    NDIVS         = Int.(ceil.([nx, ny, nz]))

    # Genereate fluid domain grid
    if verbose; println("\t"^v_lvl*"Defining fluid domain..."); end;
    grid_fdom     = gt.Grid(P_min, P_max, NDIVS)

    if verbose; println("\t"^(v_lvl+1)*"Number of nodes:\t$(grid_fdom.nnodes)"); end;


    # Visualize computational and fluid domain
    str           = save_path

    if debug
        if verbose; println("\t"^v_lvl*"Saving grid..."); end;
        str *= gt.save(grid_fdom, "fdom.$(num)"; path=save_path)
    end


    # ----------------- CALCULATE FLUID DOMAIN -----------------------------------------
    settings_fname = run_name*"_pfield_settings.jld"   # Partiel field settings file
    maxparticles   = grid_fdom.nnodes + 200000       # Maximum number of particle in field

    UJ             = vpm.UJ_fmm                      # Method for UJ evaluation

    overwrite_settings = (
                            (:maxparticles, maxparticles),
                            (:sgsmodel, vpm.sgs_none),
                            (:Uinf, (args...) -> zeros(3)),
                            (:sgsscaling, vpm.sgs_scaling_none)
                         )

    # Generate particle field
    pfield = vpm.generate_particlefield(settings_fname;
                                            path=read_path,
                                            overwrite_settings=overwrite_settings)

    prevT = zeros(3)
    prevM = Float64[i==j for i in 1:3, j in 1:3]


    if verbose; println("\t"^(v_lvl)*"Iterating over steps"); end;
    for (numi, num) in enumerate(nums)

        if verbose
            println("\t"^(v_lvl+1)*"Time step: $(num)\t($(numi) out of $(length(nums)))")
        end

        # # Bring grid back to origin
        # gt.lintransform!(grid_fdom, Float64[i==j for i in 1:3, j in 1:3], -prevT; reset_fields=true)
        #
        # # Reorient grid with global axes
        # gt.lintransform!(grid_fdom, collect(prevM'), zeros(3); reset_fields=true)
        #
        # # Bring grid back to origin and reorient with global axes at the same time
        # gt.lintransform!(grid_fdom, collect(prevM'), -prevM*prevT; reset_fields=true)
        #
        # # Translate and rotate grid
        # this_X = Xn(num)
        # T = X0 + this_X                           # Translation of fluid domain
        # M = gt.rotation_matrix(-angles(num)...)   # Orientation of fluid domain
        # gt.lintransform!(grid_fdom, M, T; reset_fields=true)

        # Bring grid back to origin and reorient with global axes
        # and translate and rotate to new position; all at the same time
        T = X0 + Xn(num)                          # Translation of fluid domain
        M = gt.rotation_matrix(-angles(num)...)   # Orientation of fluid domain
        gt.lintransform!(grid_fdom, collect(prevM')*M,
                                T - collect(M')*prevM*prevT; reset_fields=true)

        prevT, prevM = T, M

        # Read particle field
        vpm.read!(pfield, run_name*"_pfield.$(num).h5";
                                                path=read_path, overwrite=true)

        # Read embedded particles
        if static_particles
            vpm.read!(pfield, run_name*"_staticpfield.$(num).h5";
                                                path=read_path, overwrite=false)
        end

        np = vpm.get_np(pfield)

        # Rescale smoothing radii
        meansigma = 0
        meannormGamma = 0
        for P in vpm.iterate(pfield)
            P.sigma[1] *= fsgm
            meansigma += P.sigma[1]
            meannormGamma += sqrt(P.Gamma[1]^2 + P.Gamma[2]^2 + P.Gamma[3]^2)
        end
        meansigma /= np
        meannormGamma /= np

        # Add grid nodes to the particle field
        Gamma = 1e-4*meannormGamma*ones(3)
        sigma = 0.5*meansigma

        for ni in 1:grid_fdom.nnodes
            vpm.add_particle(pfield, view(grid_fdom.nodes, 1:3, ni), Gamma, sigma)
        end

        # Evaluate UJ
        vpm._reset_particles(pfield)
        t = @elapsed UJ(pfield)
        if verbose; println("\t"^(v_lvl+2)*"Evaluate UJ:\t$(round(t, digits=1)) s"); end;

        t = @elapsed begin
        Us = collect(vpm.get_U(pfield, i) for i in np+1:vpm.get_np(pfield))
        Ws = collect(vpm.get_W(pfield, i) for i in np+1:vpm.get_np(pfield))

        gt.add_field(grid_fdom, "U", "vector", Us, "node")
        gt.add_field(grid_fdom, "W", "vector", Ws, "node")

        # Save fluid domain
        gt.save(grid_fdom, "fdom.$(num)"; path=save_path, time=num)
        end

        if verbose; println("\t"^(v_lvl+2)*"Save VTK:\t$(round(t, digits=1)) s"); end;

        if debug
            str *= vpm.save(pfield, "debug_pfield"; path=save_path)
        end
    end

    str *= "fdom...vtk"

    if paraview || debug
        run(`paraview --data=$(str)`)
    end

    return str
end
