#=##############################################################################
# DESCRIPTION
    Auxiliary functions for rotor geometry generation and calculations.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

"""
Generates the Rotor object. `pitch` is the pitch of the blades in degrees,
`n` is the number of lattices in the VLM.

`ReD` is the diameter Reynolds number based on rotational speed calculated
as ReD = (omega*R)*(2*R)/nu, and `Matip` is the rotational+freestream Mach
number at the tip. This values are used for calculate airfoil properties
through XFOIL (chord Reynolds), so ignore them if airfoil properties are
prescribed.

Give it `altReD = [RPM, J, nu]`, and it'll calculate the chord-based Reynolds
accounting for the effective velocity at every station ignoring ReD (this is
more accurate, but not needed).

NOTE: If Matip is different than zero while running XFOIL, remember to deactive
compressibility corrections when running `vlm.solvefromCCBlade()` by giving it
`sound_spd=nothing`
"""
function generate_rotor(Rtip::Real, Rhub::Real, B::Int,
                        chorddist::Array{Float64,2},
                        pitchdist::Array{Float64,2},
                        sweepdist::Array{Float64,2},
                        heightdist::Array{Float64,2},
                        airfoil_files::Array{Tuple{Float64,String,String},1};
                        # INPUT OPTIONS
                        data_path=def_data_path,
                        # PROCESSING OPTIONS
                        pitch=0.0,
                        n=10, CW=true, r_lat=1, # RMA added r_lat=1.0
                        ReD=5*10^5, altReD=nothing, Matip=0.0,
                        xfoil=false,
                        rotor_file="apc10x7.jl",
                        spline_k=5, spline_s=0.001, spline_bc="extrapolate",
                        turbine_flag=false,
                        rfl_n_lower=15, rfl_n_upper=15,
                        # OUTPUT OPTIONS
                        verbose=false,
                        plot_disc=true, v_lvl=1)
    if verbose; println("\t"^v_lvl*"Generating geometry..."); end;
    n_bem = n

    # Read airfoil contours
    # Airfoils along the blade as
    # airfoil_contours=[ (pos1, contour1, polar1), (pos2, contour2, pol2), ...]
    # with contour=(x,y) and pos the position from root to tip between 0 and 1.
    # pos1 must equal 0 (root airfoil) and the last must be 1 (tip airfoil)
    airfoil_contours = []
    airfoil_path = joinpath(data_path, "airfoils")
    for (r, rfl_file, clcurve_file) in airfoil_files
        x,y = gt.readcontour(rfl_file; delim=",", path=airfoil_path, header_len=1)
        rfl = hcat(x,y)

        push!(airfoil_contours, (r, rfl, clcurve_file))
    end

    # Splines
    _spl_chord = Dierckx.Spline1D(chorddist[:, 1]*Rtip, chorddist[:, 2]*Rtip;
                                        k= size(chorddist)[1]>2 ? spline_k : 1,
                                        s=spline_s, bc=spline_bc)
    _spl_theta = Dierckx.Spline1D(pitchdist[:, 1]*Rtip, pitchdist[:, 2];
                                        k= size(pitchdist)[1]>2 ? spline_k : 1,
                                        s=spline_s, bc=spline_bc)
    _spl_LE_x = Dierckx.Spline1D(sweepdist[:, 1]*Rtip, sweepdist[:, 2]*Rtip;
                                        k= size(sweepdist)[1]>2 ? spline_k : 1,
                                        s=spline_s, bc=spline_bc)
    _spl_LE_z = Dierckx.Spline1D(heightdist[:, 1]*Rtip, heightdist[:, 2]*Rtip;
                                        k= size(heightdist)[1]>2 ? spline_k : 1,
                                        s=spline_s, bc=spline_bc)
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


    if verbose; println("\t"^v_lvl*"Generating airfoils..."); end;

    airfoils = []
    Mas = xfoil ? [] : nothing
    for (pos, contour, file_name) in airfoil_contours
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
                    this_Ma = calc_Veff(2*R, RPM, J, roR*Rtip) / speedofsnd
                else
                    this_Ma = Matip*roR
                end
            else
                    # Chord-based Reynolds number at
                this_Re = Int(ceil(ReD*roR*spl_chord(roR*Rtip)/(2*Rtip))) # this position
                this_Ma = Matip*roR
            end

            push!(Mas, this_Ma)

            polar = vlm.ap.runXFOIL(x, y, this_Re;
            alphas=[i for i in -20:1.0:20],
            verbose=verbose, Mach=this_Ma,
            iter=100, alpha_ite=10)
            # Reads polars from files
        else
            if verbose; println("\t"^(v_lvl+1)*"$file_name"); end;
            # checkfiletype = CSV.read("airfoils/sui-arl.csv",delim=",",rows=1)
            # if length(checkfiletype.colindex) == 4
            #     polar = vlm.ap.read_polar2(file_name; path=joinpath(data_path,"airfoils"), x=x, y=y)
            # else    
                polar = vlm.ap.read_polar(file_name; path=joinpath(data_path,"airfoils"), x=x, y=y)
            # end
        end

        push!(airfoils, (pos, polar))
    end

    if verbose; println("\t"^v_lvl*"Generating FLOWVLM Rotor..."); end;
    propeller = vlm.Rotor(CW, r, chord, theta, LE_x, LE_z, B, airfoils, turbine_flag)

    vlm.initialize(propeller, n; r_lat=r_lat, verif=plot_disc,
                    genblade_args=[(:spl_k,spline_k), (:spl_s,spline_s)],
                    rfl_n_lower=rfl_n_lower, rfl_n_upper=rfl_n_upper)

    if plot_disc
        fig = figure("discretization_verif", figsize=(7*2,5*1))
        suptitle("Discretization Verification")

        subplot(121)
        plot(chorddist[:, 1], chorddist[:, 2], "ok", label="Chord data", alpha=0.75)
        plot(r/Rtip, chord/Rtip, "--or", label="Chord Spline", alpha=0.75)
        plot(sweepdist[:, 1], sweepdist[:, 2], "^k", label="LE-x data", alpha=0.75)
        plot(r/Rtip, LE_x/Rtip, "--^g", label="LE-x Spline", alpha=0.75)
        plot(heightdist[:, 1], heightdist[:, 2], "*k", label="LE-z data", alpha=0.75)
        plot(r/Rtip, LE_z/Rtip, "--*b", label="LE-z Spline", alpha=0.75)
        xlabel(L"$r/R$")
        ylabel(L"$c/R$, $x/R$, $z/R$")
        legend(loc="best", frameon=true)
        grid(true, color="0.8", linestyle="--")

        subplot(122)
        plot(pitchdist[:, 1], pitchdist[:, 2], "ok", label="Twist data", alpha=0.75)
        plot(r/Rtip, theta, "--^r", label="Twist Spline", alpha=0.75)
        xlabel(L"$r/R$")
        ylabel(L"Twist $\theta$ ($^\circ$)")
        legend(loc="best", frameon=true)
        grid(true, color="0.8", linestyle="--")

        for (i,(pos, polar)) in enumerate(airfoils)
            vlm.ap.plot(polar; geometry=true, label="pos=$pos, Re=$(vlm.ap.get_Re(polar))"*
                                        (Mas!=nothing ? ", Ma=$(round(Mas[i],2))" : ""),
                    cdpolar=false, fig_id="prelim_curves", title_str="Re sweep")
        end
    end

    return propeller
end

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


function generate_rotor(rotor_file::String;
                        data_path=def_data_path, optargs...)

    Rtip, Rhub, B, blade_file = read_rotor(rotor_file; data_path=data_path)

    return generate_rotor(Rtip, Rhub, B, blade_file;
                            data_path=data_path, optargs...)
end

function read_rotor(rotor_file::String; data_path=def_data_path)

    # Path to rotor files
    rotor_path = joinpath(data_path, "rotors")

    data = CSV.read(joinpath(rotor_path, rotor_file))
    Rtip = parse(data[1, 2])
    Rhub = parse(data[2, 2])
    B = parse(data[3, 2])
    blade_file = data[4, 2]

    return Rtip, Rhub, B, blade_file
end

function read_blade(blade_file::String; data_path=def_data_path)

    # Path to rotor files
    rotor_path = joinpath(data_path, "rotors")

    # Read blade
    files = CSV.read(joinpath(rotor_path, blade_file))
    chorddist = CSV.read(joinpath(rotor_path, files[1, 2]))
    pitchdist = CSV.read(joinpath(rotor_path, files[2, 2]))
    sweepdist = CSV.read(joinpath(rotor_path, files[3, 2]))
    heightdist = CSV.read(joinpath(rotor_path, files[4, 2]))
    airfoil_files = CSV.read(joinpath(rotor_path, files[5, 2]))
    spl_k = parse(files[6, 2])
    spl_s = parse(files[7, 2])

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
    Given a diameter-based Reynolds number \mathrm{Re}_D(r) at a certain
radial position r and an advance ratio J, return the rotational speed
"""
calc_n(ReD, J, r, D, nu) = nu/( D * sqrt( (2*pi*r)^2 + (J*D)^2 ) ) * ReD
calc_RPM(args...) = 60*calc_n(args...)

calc_Veff(D, RPM, J, r) = RPM/60 * sqrt((2*pi*r)^2 + (J*D)^2)

"Reynolds number at radial station r (m)"
calc_ReD(nu, D, args...) = D/nu * calc_Veff(D, args...)
