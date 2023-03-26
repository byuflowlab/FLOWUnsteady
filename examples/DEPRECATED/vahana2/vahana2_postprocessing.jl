#=##############################################################################
# DESCRIPTION
    Simulation postprocessing.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Mar 2021
  * License   : MIT
=###############################################################################

import LinearAlgebra: pinv

dot(A, B) = sum(a*b for (a,b) in zip(A, B))
norm(A) = sqrt(mapreduce(x->x^2, +, A))
function crossp!(out, A, B)
    out[1] += A[2]*B[3] - A[3]*B[2]
    out[2] += A[3]*B[1] - A[1]*B[3]
    out[3] += A[1]*B[2] - A[2]*B[1]
end
cross(A, B) = (out = zeros(3); crossp!(out, A, B); return out;)


"""
Generate a grid around the vehicle where velocity and vorticity fields are
probed and outputted as VTK files.
"""
function postprocessing_fluiddomain(read_path, save_path, nums;
                                    # PROCESSING OPTIONS
                                    fdx=1/80,                # Scaling of cell length
                                    static_particles=false,  # Add static particles
                                    maxparticles=Int(2e6),
                                    maxsigma=Inf,            # Any particles larger than this get shrunk to this value
                                    maxmagGamma=Inf,         # Any vortex strengths larger than this get clipped to this value
                                    # SIMULATION INFORMATION
                                    ttot=30.0,
                                    nsteps=4*5400,
                                    tstart=0,                # Starting time of simulation
                                    start_kinmaneuver=false,
                                    Vref=0.25 * 125*0.44704,
                                    # OUTPUT OPTIONS
                                    run_name="vahana2",
                                    prompt=true, paraview=false, debug=false,
                                    verbose=true, v_lvl=0,
                                    optargs...)

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

    # Lx, Ly, Lz    = 1.2*L, L, 0.6*L                # Length in each direction
    Lx, Ly, Lz    = 1.08*L, L, 0.55*L
    dx, dy, dz    = dx, 1*dx, 0.5*dx                 # Cell length in each direction
    nx, ny, nz    = Lx/dx, Ly/dy, Lz/dz              # Number of cells in each direction

    # X0            = [0.35*L, 0, 0.125*L]             # Center of fluid domain
    X0            = [0.40*L, 0, 0.115*L]

    P_min         = -[Lx, Ly, Lz]/2
    P_max         = [Lx, Ly, Lz]/2
    NDIVS         = Int.(ceil.([nx, ny, nz]))

    origin(pfield, num) = X0 + Xn(num)
    # origin(pfield, num) = X0
    orientation(pfield, num) = gt.rotation_matrix(-angles(num)...)


    # --------------  PROCESSING SETUP -----------------------------------------------
    file_pref = run_name*"_pfield"
    other_file_prefs = static_particles ? [run_name*"_staticpfield"] : []
    other_read_paths = static_particles ? [read_path] : []
    file_pref_out = run_name*"-fdom-"

    # Create save path
    if save_path != read_path
        gt.create_path(save_path, prompt)
    end

    # Copy this driver file
    cp(splitdir(@__FILE__)[1], save_path*"/code"; force=true)


    # Pre-processing function
    function preprocessing(pfield, num, grids)
        count = 0
        count2 = 0
        count3 = 0
        countnan = 0
        sigma_mean = 0
        magGamma_mean = 0
        magGamma2_mean = 0

        for P in vpm.iterate(pfield; include_static=true)

            # Check for large sigma
            if P.sigma[1] > maxsigma
                sigma_mean += P.sigma[1]
                P.sigma[1] = maxsigma
                count += 1
            end

            # Check for Gamma with NaN value
            nangamma = !prod(isnan.(P.Gamma) .== false)

            if nangamma
                P.Gamma .= 1e-9
                countnan += 1
            end

            # Check for blown-up Gamma
            magGamma = norm(P.Gamma)

            if magGamma > maxmagGamma
                for i in 1:3; P.Gamma[i] = maxmagGamma * P.Gamma[i]/magGamma; end;
                magGamma_mean += magGamma
                count2 += 1
            end

            # Makes sure that the minimum Gamma is different than zero
            if magGamma < 1e-13
                P.Gamma .+= 1e-12
                magGamma2_mean += magGamma
                count3 += 1
            end

            if isnan(magGamma_mean)
                println("NaN found in magGamma_mean!")
                @show count
                @show count2
                @show countnan
            end

        end

        sigma_mean /= count
        magGamma_mean /= count2
        magGamma2_mean /= count3

        if verbose
            if count!=0
                println("\t"^(v_lvl+1)*"Shrunk $count particles."*
                                       " Average sigma of shrunk particles:\t$(sigma_mean)")
            end
            if countnan!=0
                println("\t"^(v_lvl+1)*"Found and corrected $(countnan) NaN values.")
            end
            if count2!=0
                println("\t"^(v_lvl+1)*"Reset $count2 particle strengths (too strong)."*
                                       " Average magGamma of reset particles:\t$(magGamma_mean)")
            end
            if count3!=0
                println("\t"^(v_lvl+1)*"Reset $count3 particle strengths (too weak)."*
                                       " Average magGamma of reset particles:\t$(magGamma2_mean)")
            end
        end
    end

    evaluate_fluiddomain_vtk(P_min, P_max, NDIVS, maxparticles,
                             nums, read_path, file_pref;
                                        # O=X0,
                                        origin=origin,
                                        orientation=orientation,
                                        other_file_prefs=other_file_prefs,
                                        other_read_paths=other_read_paths,
                                        verbose=verbose, v_lvl=v_lvl,
                                        debug=debug, save_path=save_path,
                                        file_pref=file_pref_out,
                                        userfunction_pfield=preprocessing,
                                        optargs...
                                        )
end




"""
Integrate forces along each rotor blade outputting them as a VTK point at the
center of pressure of each blade at each time step. The force of each bladed is
then agglomarated as a rotor force and output as a VTK point at the rotor
centroid at each time step.
The force along each wing is also integrated and output as a VTK point at its
center of pressure.
Moments are also calculated about the centroid each VTK and about the
centroid of each rotor.
"""
function postprocessing_forceandmoment(read_path, save_path, nums;
                                        # PROCESSING OPTIONS
                                        to_exclude=["_loft", "_compact",
                                                    "_Grid", "_wake",
                                                    "vahana2_TandemWing_FixedWing_vlm"],
                                        # OUTPUT OPTIONS
                                        out_suffix="-forceandmoment",
                                        prompt=true, debug=false,
                                        divcells=2,
                                        verbose=true, v_lvl=0)

    if verbose; println("\t"^(v_lvl)*"Getting ready to process force and moments on $(read_path)"); end;

    # Create save path
    gt.create_path(save_path, prompt)

    # Copy this driver file
    cp(@__FILE__, joinpath(save_path, splitext(splitdir(@__FILE__)[2])[1]*".jl"); force=true)


    Threads.@threads for (numi, num) in collect(enumerate(nums)) # Iterate over time steps
    # for (numi, num) in collect(enumerate(nums)) # Iterate over time steps

        if verbose && (numi-1)%ceil(Int, length(nums)/10)==0
            println("\t"^(v_lvl+1)*"Processing step $(num) ($(numi) out of $(length(nums)))")
        end

        # List all VTK files filtering those than contain something in to_exclude
        vtk_files = filter(f -> prod(contains.(f, (".vtk", ".$(num)."))) && prod(.!contains.(f, to_exclude)), readdir(read_path))

        # Identify the prefix of each rotor vtk
        rotor_prefs = unique(f[1:filter(si->si<=findfirst("Blade", f)[end], findall(s->s=='_', f))[end]-1]
                                                    for f in filter(f->contains(f,"Blade"), vtk_files))

        if verbose && numi==1
            println("\t"^(v_lvl+2)*"Recognized the following rotors: $(rotor_prefs)")
        end

        # ----------- CALCULATE CENTROID OF ROTORS -----------------------------
        rotors_properties = Dict()

        # Calculate the centroid of each rotor
        for rotor_pref in rotor_prefs                   # Iterate over rotors

            centroid = zeros(3)

            # VTK file of each blade in this rotor
            vtk_files_rotor = filter(f -> contains(f, rotor_pref), vtk_files)

            # Calculate centroid of each blade
            for blade_file in vtk_files_rotor           # Iterate over blades
                points, cells, cell_types, data = gt.read_vtk(blade_file; path=read_path)
                centroid .+= calc_centroid_vtk(points, cells)
            end

            centroid ./= length(vtk_files_rotor)

            # Save rotor centroid and initiate other properties
            rotor_properties = Dict(
                                    "centroid"  => centroid,
                                    "axis"      => zeros(3),
                                    "force"     => zeros(3),
                                    "moment"    => zeros(3),
                                    "radius"    => 0,
                                    "nblades"   => 0,
                                    "prevcentroid"      => zeros(3)
            )
            rotors_properties[rotor_pref] = rotor_properties
        end


        # ----------- INTEGRATE FORCES AND MOMENTS -----------------------------
        for filename in vtk_files                       # Iterate over VTK files

            points, cells, cell_types, data = gt.read_vtk(filename; path=read_path)

            if "Ftot" in keys(data["CELL_DATA"])        # Case that force was calculated

                # NOTE: Assuming that this is a VLM file, here we consider only the first
                #       half of the cells since the other half are the control points
                ncells = Int(length(cells)/divcells)

                # Centroid of each cell
                centroids = calc_centroid_cells(points, cells; divcells=divcells)

                # Centroid of this VTK
                centroid = calc_centroid_vtk(points, cells; divcells=divcells)

                # Integrate force
                Fcells = view(data["CELL_DATA"]["Ftot"], :, 1:ncells)

                ## If this is a blade, transform the distribution into actual force
                if true in contains.(filename, rotor_prefs)
                    for (ci, cell) in enumerate(cells[1:ncells]) # Iterate over cells

                        ## Determine the radial length of the cell
                        LE1, TE1, TE2, LE2 = (view(points, :, pi+1) for pi in cell)
                        chordwise = TE1 - LE1
                        radialwise = LE2 - LE1
                        radialwise .-= dot(radialwise, chordwise) * chordwise ./ norm(chordwise)^2

                        ## Convert distribution into force per panel
                        Fcells[:, ci] .*= norm(radialwise)
                    end
                end

                ## Integrate
                F = reshape(mapslices(sum, Fcells, dims=2), :)

                # Calculate center of pressure
                ## Integrate moment about origin of global coordinate system
                Mglob = zeros(3)
                for ci in 1:ncells
                    Fcell = view(Fcells, :, ci)
                    Xcell = view(centroids, :, ci)
                    crossp!(Mglob, Xcell, Fcell)
                end
                # ## Solve the system x0 × F = M
                # Xpressure = pinv(   [  0   F[3] -F[2];
                #                     -F[3]  0     F[1];
                #                      F[2] -F[1]   0  ]) * Mglob
                ## NOTE: That system is under-determined (one of the eqs is
                ##  redundant), so here I constrain it to minimze its distance
                ##  to the centroid
                ## NOTE: Turns out that this is still not very reliable
                RHS = [Mglob[1], Mglob[2], dot(centroid, F)]
                Xpressure = inv([  0   F[3]  -F[2];
                                 -F[3]  0     F[1];
                                  F[1] F[2]   F[3]]) * RHS


                # Calculate moment about centroid
                M = cross(Xpressure .- centroid, F)


                # Check if this is the blade of a rotor
                if true in contains.(filename, rotor_prefs)

                    # This blade's rotor
                    rotor_pref = rotor_prefs[findfirst(pref -> contains(filename, pref), rotor_prefs)]
                    rotor = rotors_properties[rotor_pref]

                    # Add blade force to rotor
                    rotor["force"] .+= F

                    # Add blade moment to rotor about rotor's centroid
                    Xrotor = rotor["centroid"]
                    ## ATTEMPT 1: Using the equivalent force (NOTE: Not very
                    ##   reliable unless Xpressure is correct)
                    # crossp!(rotor["moment"], Xpressure .- Xrotor, F)

                    ## Integrating the force about the rotor's centroid
                    Xcell = zeros(3)
                    for ci in 1:ncells
                        Fcell = view(Fcells, :, ci)
                        Xcell .= view(centroids, :, ci)
                        Xcell .-= Xrotor
                        crossp!(rotor["moment"], Xcell, Fcell)
                    end

                    # Update radius
                    distances = (sqrt(sum( (xc-x0)^2 for (xc, x0) in zip(view(centroids, :, ci), Xrotor) )) for ci in 1:ncells )
                    R, Ri = findmax(collect(distances))
                    rotor["radius"] += R

                    # Update axis of rotation
                    # if rotor["nblades"] != 0  # Undetermined in nblades==1
                    #     prevcentroid = rotor["prevcentroid"]
                    #     crossp!(rotor["axis"], prevcentroid .- Xrotor, centroid .- Xrotor)
                    # end
                    # rotor["prevcentroid"] .= centroid
                    # NOTE: The previous approach fails when there are only two
                    #           blades since blade centerlines are collinear.
                    #           Here I determine it from each blade instead
                    #           and hope that the little offset between
                    #           radius point and centroid will cancel
                    crossp!(rotor["axis"], centroid .- Xrotor, centroids[:, Ri] .- Xrotor)

                    # Update number of blades
                    rotor["nblades"] += 1

                end

                # Save this VTK's force and moment at center of pressure
                fname_out = filename[1:findfirst(".$(num).", filename)[1]-1]*out_suffix

                # points = [Xpressure]
                points = [centroid]
                point_data = [
                                Dict("field_name" => "force",
                                     "field_type" => "vector",
                                     "field_data" => [F]),

                                 Dict("field_name" => "moment",
                                      "field_type" => "vector",
                                      "field_data" => [M]),

                                  Dict("field_name" => "centerofpressure",
                                       "field_type" => "vector",
                                       "field_data" => [Xpressure .- points[1]]),

                                  Dict("field_name" => "centroid", # centroid relative to center of pressure
                                       "field_type" => "vector",
                                       "field_data" => [centroid .- points[1]])
                             ]

                gt.generateVTK(fname_out, points; point_data=point_data,
                                                        path=save_path, num=num)
            else
                @warn("Ftot field not found in data of $(filename). "*
                      "File will be ignored.")
            end
        end

        # ----------- SAVE ROTOR VTKS ------------------------------------------
        for rotor_pref in rotor_prefs

            rotor = rotors_properties[rotor_pref]

            # Calculate this rotor's center of pressure (relative to centroid)
            ## Solve the system x0 × F = M
            F = rotor["force"]
            M = rotor["moment"]
            centroid = rotor["centroid"]
            # Xpressure = pinv(   [  0   F[3] -F[2];
            #                     -F[3]  0     F[1];
            #                      F[2] -F[1]   0  ]) * M
            ## NOTE: That system is under-determined (one of the eqs is
            ##  redundant), so here I constrain it to minimze its distance
            ##  to the centroid
            RHS = [M[1], M[2], dot(centroid, F)]
            Xpressure = inv([   0   F[3]  -F[2];
                              -F[3]  0     F[1];
                               F[1] F[2]   F[3]]) * RHS

            # Update radius
            rotor["radius"] /= rotor["nblades"]

            # Update axis of rotation
            rotor["axis"] *= rotor["radius"]/norm(rotor["axis"])

            # Save VTK
            fname_out = rotor_pref*out_suffix

            points = [centroid]
            point_data = [
                            Dict("field_name" => "force",
                                 "field_type" => "vector",
                                 "field_data" => [F]),

                             Dict("field_name" => "moment",
                                  "field_type" => "vector",
                                  "field_data" => [M]),

                              Dict("field_name" => "centerofpressure",
                                   "field_type" => "vector",
                                   "field_data" => [Xpressure .- points[1]]),

                               Dict("field_name" => "centroid",
                                    "field_type" => "vector",
                                    "field_data" => [centroid .- points[1]]),

                                Dict("field_name" => "axis",
                                     "field_type" => "vector",
                                     "field_data" => [rotor["axis"]]),

                                Dict("field_name" => "radius",
                                     "field_type" => "scalar",
                                     "field_data" => [rotor["radius"]]),

                                 Dict("field_name" => "nblades",
                                      "field_type" => "scalar",
                                      "field_data" => [rotor["nblades"]])
                         ]

            gt.generateVTK(fname_out, points; point_data=point_data,
                                                    path=save_path, num=num)
        end
    end

    println("Forces and moments saved under $save_path")
end



"""
Calculate statistics of the solution over a given range of time steps given
by `nums`. Use this to calculate, for instance, the mean load distribution and
fluctuations on wings.
"""
function postprocessing_statistics(read_path, save_path, nums;
                                        # PROCESSING OPTIONS
                                        idens=[""], # Use this to agglomerate mutliple simulations
                                        to_exclude=["_loft", "_compact",
                                                    "_Grid", "_wake",
                                                    "vahana2_TandemWing_FixedWing_vlm",
                                                    "_Blade"],
                                        # OUTPUT OPTIONS
                                        prompt=true, debug=false,
                                        verbose=true, v_lvl=0)


    read_paths      = [read_path*iden*"/" for iden in idens]  # Path to simulations to process


    # -------------- PROCESSING OPTIONS --------------------------------------------
    # Cartesian statistical operations to perform
    operations_cart = [
                        ("mean", gt._agglomerate_mean, 1),
                        ("cart-variance", gt._agglomerate_variance, 2),
                        # ("cart-corr1", gt._agglomerate_correlation1, 2),
                        # ("cart-corr2", gt._agglomerate_correlation2, 2),
                        # ("cart-corr3", gt._agglomerate_correlation3, 2),
                      ]

    # Cylindrical statistical operations to perform
    # Oaxis     = Float64[i==j for i in 1:3, j in 1:3] # Orientation of jet
    # axial_dir = Oaxis[:, 1]                 # Axial direction
    operations_cyl = [
                      # ("cyl-mean", (args...; optargs...) -> gt._agglomerate_cyl_mean(axial_dir, args...; optargs...), 1),
                      # ("cyl-variance", (args...; optargs...) -> gt._agglomerate_cyl_variance(axial_dir, args...; optargs...), 2),
                      # ("cyl-corr1", (args...; optargs...) -> gt._agglomerate_cyl_correlation1(axial_dir, args...; optargs...), 2),
                      # ("cyl-corr2", (args...; optargs...) -> gt._agglomerate_cyl_correlation2(axial_dir, args...; optargs...), 2),
                      # ("cyl-corr3", (args...; optargs...) -> gt._agglomerate_cyl_correlation3(axial_dir, args...; optargs...), 2),
                    ]

    # Statistical operations
    operations = vcat(operations_cart, operations_cyl)

    # --------------  PROCESSING SETUP ---------------------------------------------
    if verbose; println("\t"^(v_lvl)*"Getting ready to process statistics on $(read_paths)"); end;

    # Create save path
    if !(save_path in read_paths)
        gt.create_path(save_path, prompt)
    end

    # Copy this driver file
    cp(@__FILE__, joinpath(save_path, splitext(splitdir(@__FILE__)[2])[1]*".jl"); force=true)

    # Identify lines to average together
    to_average_prefixes = unique([f[1:findfirst(".", f)[1]-1] for f in readdir(read_paths[1])
                                        if contains(f, ".vtk") && prod(.!contains.(f, to_exclude))])


    # -------------- PROCESS SIMULATION --------------------------------------------
    if verbose; println("\n"*"\t"^(v_lvl)*"Iterating over VTK files"); end;

    Threads.@threads for pref in to_average_prefixes

        if verbose
            println("\t"^(v_lvl+1)*"Averaging files with prefix $(pref)")
        end

        # this_operations = contains(pref, "Line-Axial") ? operations_cart : operations
        this_operations = operations

        # files = vcat([ [joinpath(read_path, f) for f in readdir(read_path) if contains(f, ".vtk") && contains(f, pref)] for read_path in read_paths]...)
        files = vcat([ [joinpath(read_path, f) for f in readdir(read_path)
                                    if contains(f, ".vtk") && contains(f, pref) && parse(Int, f[findfirst(".", f)[1]+1:findlast(".", f)[1]-1]) in nums]
                                        for read_path in read_paths]...)

        t = @elapsed gt.calculate_statistics_vtk(files, save_path;
                                                    operations=this_operations,
                                                    # read_path=read_path,
                                                    read_path="",
                                                    out_filename=pref*"-statistics",
                                                    verbose=verbose, v_lvl=v_lvl+2)

        if verbose
            println("\t"^(v_lvl+2)*"Process statistics:\t$(round(t, digits=1)) s")
        end
    end

    println("Statistics saved under $save_path")
end


"""
Auxiliary function for calculating the centroid of each cell in a VTK
"""
function calc_centroid_cells(points, cells; divcells=2)
    # NOTE: Assuming that this is a VLM file, here we consider only the first
    #       half of the cells since the other half are the control points
    ncells = Int(length(cells)/divcells)
    centroids = zeros(3, ncells)

    for (celli, cell) in enumerate(view(cells, 1:ncells))
        for pi in cell
            centroids[:, celli] .+= view(points, 1:3, pi+1)
        end
        centroids[:, celli] ./= length(cell)
    end

    return centroids
end

"""
Auxiliary function for calculating the centroid of a VTK
"""
function calc_centroid_vtk(points, cells, args...; optargs...)

    # Calculate the centroid of each cell
    centroids = calc_centroid_cells(points, cells, args...; optargs...)

    # Calculate centroid of vtk
    vtk_centroid = mapslices(sum, centroids, dims=2)
    vtk_centroid ./= size(centroids, 2)

    return reshape(vtk_centroid, :)
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
