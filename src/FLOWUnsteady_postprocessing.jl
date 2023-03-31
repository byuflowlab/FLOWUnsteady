#=##############################################################################
# DESCRIPTION
    Tools for postprocessing simulations.

# ABOUT
  * Created   : Mar 2023
  * License   : MIT
=###############################################################################


"""
    postprocess_statistics(read_path, save_path, nums;
                            # PROCESSING OPTIONS
                            idens           = [""],     # Use this to agglomerate multiple simulations
                            to_exclude      = [],       # Exclude file names containing these words
                            cyl_axial_dir   = nothing,  # Calculate cylindrical statistics if given an axial axis (vector)
                            # OUTPUT OPTIONS
                            prompt=true, debug=false,
                            verbose=true, v_lvl=0)

Calculate statistics of the simulation VTK outputs over a given range of time
steps given by `nums`. Use this to calculate, for instance, the mean load
distribution and fluctuations on wings and rotors.
"""
function postprocess_statistics(read_path, save_path, nums;
                                    # PROCESSING OPTIONS
                                    idens           = [""],     # Use this to agglomerate multiple simulations
                                    to_exclude      = [],       # Exclude file names containing these words
                                    cyl_axial_dir::Union{Nothing, Vector{Float64}} = nothing,  # Calculate cylindrical statistics if given an axial axis (vector)
                                    # OUTPUT OPTIONS
                                    prompt=true, debug=false,
                                    verbose=true, v_lvl=0)


    read_paths      = [read_path*iden for iden in idens]  # Path to simulations to process


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
    if cyl_axial_dir != nothing
        operations_cyl = [
                          ("cyl-mean", (args...; optargs...) -> gt._agglomerate_cyl_mean(cyl_axial_dir, args...; optargs...), 1),
                          ("cyl-variance", (args...; optargs...) -> gt._agglomerate_cyl_variance(cyl_axial_dir, args...; optargs...), 2),
                          # ("cyl-corr1", (args...; optargs...) -> gt._agglomerate_cyl_correlation1(axial_dir, args...; optargs...), 2),
                          # ("cyl-corr2", (args...; optargs...) -> gt._agglomerate_cyl_correlation2(axial_dir, args...; optargs...), 2),
                          # ("cyl-corr3", (args...; optargs...) -> gt._agglomerate_cyl_correlation3(axial_dir, args...; optargs...), 2),
                        ]
    else
        operations_cyl = []
    end

    # Statistical operations
    operations = vcat(operations_cart, operations_cyl)

    # --------------  PROCESSING SETUP ---------------------------------------------
    if verbose; println("\t"^(v_lvl)*"Getting ready to process statistics on $(read_paths)"); end;

    # Create save path
    if !(save_path in read_paths)
        gt.create_path(save_path, prompt)
    end

    # Identify vtks to average together
    to_average_prefixes = unique([f[1:findfirst(".", f)[1]-1] for f in readdir(read_paths[1])
                                        if contains(f, ".vtk") && prod(.!contains.(f, to_exclude))])


    # -------------- PROCESS SIMULATION --------------------------------------------
    if verbose; println("\n"*"\t"^(v_lvl)*"Iterating over VTK files"); end;

    # Threads.@threads for pref in to_average_prefixes
    for pref in to_average_prefixes

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
    postprocess_bladeloading(read_path;
                                O           = zeros(3),     # Rotor center
                                rotor_axis  = [-1, 0, 0],   # Rotor centerline axis
                                Ftot_axis   = nothing,      # Use a different centerline axis for forces if given
                                filename    = "singlerotor_Rotor_Blade1_vlm-statistics.vtk", # File name
                                fieldsuff   = "-mean"       # Suffix of fields "Gamma" and "Ftot", if any
                                )

Read a blade VTK file `filename` under directory `read_path` and returns
the circulation and force components of the load distribution along the blade.

Return: `rs, Gamma, Np, Tp, Rp, Zhat, Rhat, That, Ftot`
"""
function postprocess_bladeloading(read_path;
                                    O           = zeros(3),     # Rotor center
                                    rotor_axis  = [-1, 0, 0],   # Rotor centerline axis
                                    Ftot_axis   = nothing,      # Use a different centerline axis for forces if given
                                    filename    = "singlerotor_Rotor_Blade1_vlm-statistics.vtk", # File name
                                    fieldsuff   = "-mean"       # Suffix of fields "Gamma" and "Ftot", if any
                                    )

    @assert isapprox(norm(rotor_axis), 1.0) "Non-unitary rotor axis $(rotor_axis)"

    _Ftot_axis = Ftot_axis==nothing ? rotor_axis : Ftot_axis

    points, cells, cell_types, data = gt.read_vtk(filename; path=read_path)

    maxind = Int(size(cells, 1)/2)
    Xs = [mean([points[:, pi+1] for pi in cell]) for cell in cells[1:maxind]]
    Rs = [(X-O) - rotor_axis*dot(X-O, rotor_axis) for X in Xs]

    # Blade centerline
    Rhat = mean(Rs)
    Rhat /= norm(Rhat)

    rs = [dot(R, Rhat) for R in Rs]

    # Tangent vector
    That = cross(_Ftot_axis, Rhat)

    # Grabs only the rectangular cells
    Gamma = data["CELL_DATA"]["Gamma"*fieldsuff][1:maxind]
    Ftot = data["CELL_DATA"]["Ftot"*fieldsuff][:, 1:maxind]

    nr = length(rs)
    Np = zeros(nr)              # Normal component
    Rp = zeros(nr)              # Radial component
    Tp = zeros(nr)              # Tangential component

    for i in 1:nr
        F = Ftot[:, i]
        Np[i] = dot(F, _Ftot_axis)
        Rp[i] = dot(F, Rhat)
        Tp[i] = dot(F, That)
    end

    return rs, Gamma, Np, Tp, Rp, _Ftot_axis, Rhat, That, Ftot
end

"""
    generate_preprocessing_fluiddomain_pfield(maxsigma, maxmagGamma;
                                                    verbose=true, v_lvl=1)

Generate function for pre-processing particle fields before generating fluid
domain: shrink oversized particles and correct blown-up particles.

Pass the output function to [`FLOWUnsteady.computefluiddomain`](@ref) through
the keyword argument `userfunction_pfield`. For example:
```julia
preprocess_pfield = generate_preprocessing_fluiddomain_pfield(maxsigma, maxmagGamma;
                                                                verbose=true, v_lvl=1)

computefluiddomain( ... ; userfunction_pfield=preprocess_pfield, ...)
```
"""
function generate_preprocessing_fluiddomain_pfield(maxsigma, maxmagGamma; verbose=true, v_lvl=1)


    function preprocessing(pfield, args...; optargs...)

        count_sigma, count_nan, count_Gamma = 0, 0, 0
        mean_sigma, mean_Gamma = 0.0, 0.0

        for P in vpm.iterate(pfield; include_static=true)

            # Check for large sigma
            if P.sigma[1] > maxsigma
                mean_sigma += P.sigma[1]
                P.sigma[1] = maxsigma
                count_sigma += 1
            end

            # Check for Gamma with NaN value
            nangamma = !prod(isnan.(P.Gamma) .== false)
            if nangamma
                P.Gamma .= 1e-12
                count_nan += 1
            end

            # Check for blown-up Gamma
            magGamma = norm(P.Gamma)
            if magGamma > maxmagGamma
                P.Gamma *= maxmagGamma / magGamma
                mean_Gamma += magGamma
                count_Gamma += 1
            end

            # Makes sure that the minimum Gamma is different than zero
            if magGamma < 1e-14
                P.Gamma .+= 1e-14
            end

            if isnan(magGamma)
                @warn "NaN found in mean_Gamma! The simulation is likely blown up"
                @show count_sigma
                @show count_Gamma
                @show count_nan
            end

        end
        mean_sigma /= count_sigma
        mean_Gamma /= count_Gamma

        if verbose

            if count_sigma != 0
                println("\t"^(v_lvl)*"Shrunk $(count_sigma) particles."*
                        " Average sigma of shrunk particles: $(mean_sigma)")
            end
            if count_nan != 0
                println("\t"^(v_lvl)*"Found and corrected $(count_nan) NaN values.")
            end
            if count_Gamma != 0
                println("\t"^(v_lvl)*"Reset $(count_Gamma) particle strengths."*
                        " Average magGamma of reset particles: $(mean_Gamma)")
            end

        end
    end

    return preprocessing
end
