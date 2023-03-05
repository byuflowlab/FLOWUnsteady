#=##############################################################################
# DESCRIPTION
    Auxiliary functions for rotor geometry generation and calculations.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

"""
    generate_rotor(Rtip, Rhub, B,
                        chorddist, twistdist, sweepdist, heightdist,
                        airfoil_contours;

                        # MORE ROTOR PARAMETERS
                        pitch=0.0,
                        CW=true,

                        # DISCRETIZATION SETTINGS
                        n=10, blade_r=1.0,
                        rediscretize_airfoils=true,
                        rfl_n_lower=15, rfl_n_upper=15,
                        spline_k=5, spline_s=0.001, spline_bc="extrapolate",

                        # AIRFOIL PROCESSING
                        data_path=FLOWUnsteady.default_database,
                        read_polar=vlm.ap.read_polar,
                        xfoil=false,
                        alphas=[i for i in -20:1.0:20], ncrit=9,
                        ReD=5e5, altReD=nothing, Matip=0.0,

                        # OUTPUT OPTIONS
                        verbose=false, verbose_xfoil=false, v_lvl=1,
                        plot_disc=true, save_polars=nothing)

Generates a `FLOWVLM.Rotor` from direct inputs.

# ARGUMENTS
* `Rtip::Real`      :   (m) rotor radius (from centerline to blade tip)
* `Rhub::Real`      :   (m) hub radius (from centerline to blade root)
* `B::Int`          :   Number of blades
* `chorddist::Matrix`   : Chord distribution (`chorddist[:, 1] = r/R`, `chorddist[:, 2] = c/R`
* `twistdist::Matrix`   : Twist distribution (`twistdist[:, 1] = r/R`, `twistdist[:, 2] = degs`
* `sweepdist::Matrix`   : LE sweep distribution (`sweepdist[:, 1] = r/R`, `sweepdist[:, 2] = x/R`
* `heightdist::Matrix`  : LE height distribution (`heightdist[:, 1] = r/R`, `heightdist[:, 2] = z/R`
* `airfoil_contours::Array{ Tuple{Float64, Array{Float64, 2}, String} }`    :
                            Airfoil contours along the span. It must follow the
                            pattern `(pos, contour, polarfile) = airfoil_contours[i]`,
                            where `pos = (r-Rhub)/(Rtip-Rhub)` is the spanwise
                            position (with root=0, tip=1), `contour` is the
                            airfoil profile (`contour[:, 1] = x/c`, `contour[:, 2] = y/c`),
                            and `polarfile` is any file from
                            [airfoiltools.com](http://airfoiltools.com/) with
                            the airfoil lookup table (airfoil polar).

The function allows `airfoil_contours::Array{ Tuple{Float64, String, String} }`,
following the pattern `(pos, contourfile, polarfile) = airfoil_contours[i]`
where `contourfile` is a CSV file with columns `x/c` and `y/c`.

# KEYWORD ARGUMENTS

## **Extra rotor parameters**
* `pitch::Real`         :   (deg) rotor collective pitch
* `CW::Bool`            :   Whether the rotor rotates in the clockwise (`true`)
                            or counter-clockwise (`false`)

## **Discretization**
* `n::Int`              :   Number of blade elements per blade.
* `r::Real`             :   Spacing between elements at the tip, divided by
                            the spacing between elements at the root.
* `spline_k::Int`, `spline_s::Real`, `spline_bc::String` : To discretize the
                            blade, the blade distributions are splined and
                            re-discretize into `n` elements. These splines
                            are done through the [Dierckx package](https://github.com/kbarbary/Dierckx.jl),
                            with `spline_k` the order of the spline, `spline_s`
                            the smoothing parameter, and `spline_bc` the
                            boundary condition.
* `rediscretize_airfoils`   : If true, it will spline the airfoil contours and
                                re-discretize them. It will discretize the lower
                                side of the contour into `rfl_n_lower` panels,
                                and the upper side into `rfl_n_upper` panels.
                                This is necessary unless all the airfoil
                                contours already have the same number of points.

## **Airfoil processing**
* `data_path::String`   :   Path to database where to read the airfoil polars
                            from.
* `read_polar::Function` :  Function used for parsing the airfoil files. Use
                            `vlm.ap.read_polar` for files that are direct
                            outputs from XFOIL. Use `vlm.ap.read_polar2` for CSV
                            files.
* `xfoil::Bool`         :   If true, the polar files will be ignored and XFOIL
                            will be used to generate the polars instead. This
                            will be done sweeping AOA as in `alphas`
                            (in degrees) and `ncrit` for inflow turbulence
                            parameter.
* `ReD::Real`, `Matip::Real`, `altReD::Tuple{Real, Real, Real}`


`ReD` is the diameter Reynolds number based on rotational speed calculated
as `ReD = (omega*R)*(2*R)/nu`, and `Matip` is the rotational+freestream Mach
number at the tip. These number will be used for running XFOIL to compute
airfoil polars, and will be ignored if airfoil polars are prescribed.

Give it `altReD = (RPM, J, nu)`, and it will calculate the chord-based Reynolds
accounting for the effective velocity at every station, ignoring `ReD` (this is
more accurate, but not needed).

**NOTE:** If `Matip` is different than zero while running XFOIL, you must
deactive compressibility corrections in `run_simulation` by using
`sound_spd=nothing`. Otherwise, compressibility effects will be double accounted
for.

## **Outputs**
* `verbose::Bool`       :   Whether to verbose while generating the rotor
* `verbose_xfoil::Bool` :   Whether to verbose while running XFOIL
* `v_lvl::Int`          :   Indentation level for printing the verbose
* `plot_disc`           :   If true, it will plot the discretization process
                            of the blade, which is useful for verification and
                            debugging. It will also plot the airfoil polars.
* `save_polars::String` :   Give it a path to a directory and it will save the
                            polars plot in that directory

"""
function generate_rotor(Rtip::Real, Rhub::Real, B::Int,
                        chorddist::Array{Float64,2},
                        pitchdist::Array{Float64,2},
                        sweepdist::Array{Float64,2},
                        heightdist::Array{Float64,2},
                        airfoil_contours::Array{Tuple{Float64,Array{Float64, 2},String},1};
                        # INPUT OPTIONS
                        data_path=default_database,
                        read_polar=vlm.ap.read_polar,
                        # PROCESSING OPTIONS
                        pitch=0.0,
                        n=10, CW=true, blade_r=1.0,
                        ReD=5*10^5, altReD=nothing, Matip=0.0,
                        ncrit=9, alphas=[i for i in -20:1.0:20],
                        xfoil=false,
                        spline_k=5, spline_s=0.001, splines_s=nothing, spline_bc="extrapolate",
                        turbine_flag=false,
                        rfl_n_lower=15, rfl_n_upper=15,
                        rediscretize_airfoils=true,
                        # OUTPUT OPTIONS
                        verbose=false, verbose_xfoil=false, v_lvl=1,
                        save_polars=nothing, save_polar_pref="airfoilpolar",
                        plot_disc=true, figsize_factor=2/3)

    if verbose; println("\t"^v_lvl*"Generating geometry..."); end;
    n_bem = n

    # Splines
    _spl_chord = Dierckx.Spline1D(chorddist[:, 1]*Rtip, chorddist[:, 2]*Rtip;
                                        k= size(chorddist)[1]>2 ? spline_k : 1,
                                        s=splines_s!=nothing ? splines_s[1] : spline_s, bc=spline_bc)
    _spl_theta = Dierckx.Spline1D(pitchdist[:, 1]*Rtip, pitchdist[:, 2];
                                        k= size(pitchdist)[1]>2 ? spline_k : 1,
                                        s=splines_s!=nothing ? splines_s[2] : spline_s, bc=spline_bc)
    _spl_LE_x = Dierckx.Spline1D(sweepdist[:, 1]*Rtip, sweepdist[:, 2]*Rtip;
                                        k= size(sweepdist)[1]>2 ? spline_k : 1,
                                        s=splines_s!=nothing ? splines_s[3] : spline_s, bc=spline_bc)
    _spl_LE_z = Dierckx.Spline1D(heightdist[:, 1]*Rtip, heightdist[:, 2]*Rtip;
                                        k= size(heightdist)[1]>2 ? spline_k : 1,
                                        s=splines_s!=nothing ? splines_s[4] : spline_s, bc=spline_bc)
    spl_chord(x) = Dierckx.evaluate(_spl_chord, x)
    spl_theta(x) = pitch + Dierckx.evaluate(_spl_theta, x)
    spl_LE_x(x) = Dierckx.evaluate(_spl_LE_x, x)
    spl_LE_z(x) = Dierckx.evaluate(_spl_LE_z, x)

    # Geometry for CCBlade & FLOWVLM
    r = [Rhub + i*(Rtip-Rhub)/n_bem for i in 0:n_bem] # r is discretized in n+1 sections
    chord = [spl_chord(ri) for ri in r]
    theta = [spl_theta(ri) for ri in r]
    LE_x = [spl_LE_x(ri) for ri in r]
    LE_z = [spl_LE_z(ri) for ri in r]


    if verbose; println("\t"^v_lvl*"Generating airfoil data..."); end;

    if Matip != 0 && xfoil
        @warn("*"^73*"\n"*
              "Tip Mach number received (Matip=$(Matip)). This will be used to"*
              " pre-correct the airfoil polars in XFOIL for compressibility"*
              " effects. However, FLOWUnsteady also automatically corrects for"*
              " compressibility. Make sure to run simulation as"*
              " `run_simulation(...; sound_spd=nothing, ...)` to avoid double"*
              "-accounting for compressibility.\n"*"*"^73)
    end

    airfoils = []
    Mas = xfoil ? [] : nothing
    for (rfli, (pos, contour, file_name)) in enumerate(airfoil_contours)
        x, y = contour[:,1], contour[:,2]
        # Calls XFOIL to calculate polars of each airfoil
        if xfoil
            roR = (Rhub + pos*(Rtip-Rhub))/Rtip       # (r/R) position along blade.

            if altReD!=nothing
                RPM, J, nu = altReD
                this_ReD = calc_ReD(nu, 2*Rtip, RPM, J, roR*Rtip) # Diameter Reynolds at r
                this_Re = ceil(Int, this_ReD*spl_chord(roR*Rtip)/(2Rtip)) # Chord Reynolds

                if Matip != 0
                    speedofsnd = (2*pi*RPM/60)*Rtip / Matip
                    this_Ma = calc_Veff(2*Rtip, RPM, J, roR*Rtip) / speedofsnd
                else
                    this_Ma = Matip*roR
                end
            else
                    # Chord-based Reynolds number at
                this_Re = Int(ceil(ReD*roR*spl_chord(roR*Rtip)/(2*Rtip))) # this position
                this_Ma = Matip*roR
            end

            push!(Mas, this_Ma)

            if verbose; print("\t"^(v_lvl+1)*"Running XFOIL on airfoil"*
                        " $(rfli) out of $(length(airfoil_contours))..."); end;
            polar = vlm.ap.runXFOIL(x, y, this_Re;
                                    alphas=alphas,
                                    verbose=verbose_xfoil, Mach=this_Ma,
                                    iter=100, ncrit=ncrit)
            if verbose; println(" done."); end;

            if save_polars != nothing
                if !(isdir(save_polars)); mkdir(save_polars); end;
                vlm.ap.save_polar2(polar, save_polar_pref*"-sec$(rfli)-Re$(ceil(Int, this_Re))"; path=save_polars)
            end

        else # Reads polars from files
            if verbose; println("\t"^(v_lvl+1)*"$file_name"); end;
            polar = read_polar(file_name; path=joinpath(data_path, "airfoils"), x=x, y=y)
        end

        push!(airfoils, (pos, polar))
    end

    if verbose; println("\t"^v_lvl*"Generating FLOWVLM Rotor..."); end;
    propeller = vlm.Rotor(CW, r, chord, theta, LE_x, LE_z, B, airfoils, turbine_flag)

    vlm.initialize(propeller, n; r_lat=blade_r, verif=plot_disc,
                    genblade_args=[(:spl_k,spline_k), (:spl_s,spline_s)],
                    rfl_n_lower=rfl_n_lower, rfl_n_upper=rfl_n_upper,
                    figsize_factor=figsize_factor,
                    rediscretize=rediscretize_airfoils)

    if plot_disc
        formatpyplot()
        fig = plt.figure("flowunsteady-discr", figsize=[7*2,5*1]*figsize_factor)
        fig.suptitle("FLOWUnsteady rediscretization verification")
        axs = fig.subplots(1, 2)

        ax = axs[1]
        ax.plot(chorddist[:, 1], chorddist[:, 2], "ok", label="Chord data", alpha=0.75)
        ax.plot(r/Rtip, chord/Rtip, "--or", label="Chord Spline", alpha=0.75)
        ax.plot(sweepdist[:, 1], sweepdist[:, 2], "^k", label="LE-x data", alpha=0.75)
        ax.plot(r/Rtip, LE_x/Rtip, "--^g", label="LE-x Spline", alpha=0.75)
        ax.plot(heightdist[:, 1], heightdist[:, 2], "*k", label="LE-z data", alpha=0.75)
        ax.plot(r/Rtip, LE_z/Rtip, "--*b", label="LE-z Spline", alpha=0.75)
        ax.set_xlabel(L"$r/R$")
        ax.set_ylabel(L"$c/R$, $x/R$, $z/R$")
        ax.legend(loc="best", frameon=false, fontsize=6)
        ax.grid(true, color="0.8", linestyle="--")

        ax = axs[2]
        ax.plot(pitchdist[:, 1], pitchdist[:, 2], "ok", label="Twist data", alpha=0.75)
        ax.plot(r/Rtip, theta, "--^r", label="Twist Spline", alpha=0.75)
        ax.set_xlabel(L"$r/R$")
        ax.set_ylabel(L"Twist $\theta$ ($^\circ$)")
        ax.legend(loc="best", frameon=false, fontsize=8)
        ax.grid(true, color="0.8", linestyle="--")

        fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        fig = plt.figure("polars", figsize=[7*3, 5*1]*7/9)
        axs = fig.subplots(1, 3)
        for (i,(pos, polar)) in enumerate(airfoils)
            vlm.ap.plot(polar; geometry=true, label="pos=$(round(pos, digits=3)), Re=$(ceil(Int, vlm.ap.get_Re(polar)))"*
                                        (Mas!=nothing ? ", Ma=$(round(Mas[i], digits=2))" : ""),
                    cdpolar=false, fig_id="prelim_curves", title_str="",
                    rfl_figfactor=figsize_factor, fig=fig, axs=axs)
        end
        axs[3].legend(loc="best", frameon=true, fontsize=7)
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        # fig = plt.figure("prelim_curves_rfl")
        # fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    end

    return propeller
end


function generate_rotor(Rtip::Real, Rhub::Real, B::Int,
                        chorddist::Array{Float64,2},
                        pitchdist::Array{Float64,2},
                        sweepdist::Array{Float64,2},
                        heightdist::Array{Float64,2},
                        airfoil_files::Array{Tuple{Float64,String,String},1};
                        # INPUT OPTIONS
                        data_path=def_data_path, optargs...)

    # Read airfoil contours
    # Airfoils along the blade as
    # airfoil_contours=[ (pos1, contour1, polar1), (pos2, contour2, pol2), ...]
    # with contour=(x,y) and pos the position from root to tip between 0 and 1.
    # pos1 must equal 0 (root airfoil) and the last must be 1 (tip airfoil)
    airfoil_contours = Tuple{Float64,Array{Float64, 2},String}[]
    airfoil_path = joinpath(data_path, "airfoils")
    for (r, rfl_file, clcurve_file) in airfoil_files
        x,y = gt.readcontour(rfl_file; delim=",", path=airfoil_path, header_len=1)
        rfl = hcat(x,y)

        push!(airfoil_contours, (r, rfl, clcurve_file))
    end

    return generate_rotor(Rtip, Rhub, B,
                            chorddist, pitchdist, sweepdist, heightdist,
                            airfoil_contours; data_path=data_path, optargs...)
end

"""
    generate_rotor(Rtip, Rhub, B, blade_file::String;
                        data_path=FLOWUnsteady.default_database, optargs...)

Generates a `FLOWVLM.Rotor` reading the blade geometry from the blade file
`blade_file` found in the database `data_path`.
"""
function generate_rotor(Rtip::Real, Rhub::Real, B::Int, blade_file::String;
                        data_path=def_data_path, optargs...)

    (chorddist, pitchdist,
     sweepdist, heightdist,
     airfoil_files, spl_k,
     spl_s) = read_blade(blade_file; data_path=data_path)

    return generate_rotor(Rtip, Rhub, B, chorddist, pitchdist, sweepdist,
                            heightdist, airfoil_files;
                            data_path=data_path,
                            spline_k=spl_k, spline_s=spl_s, optargs...)
end

"""
    generate_rotor(rotor_file::String;
                        data_path=FLOWUnsteady.default_database, optargs...)

Generates a `FLOWVLM.Rotor` reading the full rotor geometry from the rotor file
`rotor_file` found in the database `data_path`.
"""
function generate_rotor(rotor_file::String;
                        data_path=def_data_path, optargs...)

    Rtip, Rhub, B, blade_file = read_rotor(rotor_file; data_path=data_path)

    return generate_rotor(Rtip, Rhub, B, blade_file;
                            data_path=data_path, optargs...)
end

function read_rotor(rotor_file::String; data_path=def_data_path)

    # Path to rotor files
    rotor_path = joinpath(data_path, "rotors")

    data = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, rotor_file)))
    Rtip = Meta.parse(data[1, 2])
    Rhub = Meta.parse(data[2, 2])
    B = Meta.parse(data[3, 2])
    blade_file = String(data[4, 2])

    return Rtip, Rhub, B, blade_file
end

function read_blade(blade_file::String; data_path=def_data_path)

    # Path to rotor files
    rotor_path = joinpath(data_path, "rotors")

    # Read blade
    files = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, blade_file)))
    chorddist = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, files[1, 2])))
    pitchdist = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, files[2, 2])))
    sweepdist = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, files[3, 2])))
    heightdist = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, files[4, 2])))
    airfoil_files = DataFrames.DataFrame(CSV.File(joinpath(rotor_path, files[5, 2])))
    spl_k = Meta.parse(files[6, 2])
    spl_s = Meta.parse(files[7, 2])

    # Convert DataFrames to concrete types
    chorddist = Array{Float64, 2}(chorddist)
    pitchdist = Array{Float64, 2}(pitchdist)
    sweepdist = Array{Float64, 2}(sweepdist)
    heightdist = Array{Float64, 2}(heightdist)

    af = airfoil_files
    airfoil_files = [(Float64(af[i, 1]), String(af[i, 2]), String(af[i, 3]))
                        for i in 1:size(af, 1)]

    return chorddist, pitchdist, sweepdist, heightdist, airfoil_files, spl_k, spl_s
end

"""
    Given a diameter-based Reynolds number \$\\mathrm{Re}_D(r)\$ at a certain
radial position r and an advance ratio J, return the rotational speed
"""
calc_n(ReD, J, r, D, nu) = nu/( D * sqrt( (2*pi*r)^2 + (J*D)^2 ) ) * ReD
calc_RPM(args...) = 60*calc_n(args...)

calc_Veff(D, RPM, J, r) = RPM/60 * sqrt((2*pi*r)^2 + (J*D)^2)

"Reynolds number at radial station r (m)"
calc_ReD(nu, D, args...) = D/nu * calc_Veff(D, args...)
