#=##############################################################################
# DESCRIPTION
    Tools for processing the aerodynamic solution to calculate forces.

# ABOUT
  * Created   : Apr 2022
  * License   : MIT
=###############################################################################

"""
    generate_calc_aerodynamicforce(; add_parasiticdrag=false,
                                          add_skinfriction=true,
                                          airfoilpolar="xf-n0012-il-500000-n5.csv",
                                          parasiticdrag_args=(),
                                          )

Default method for calculating aerodynamic forces.

Pass the output of this function to [`generate_monitor_wing`](@ref), or use this
as an example on how to define your own costumized force calculations.

This function stitches together the outputs of
[`generate_aerodynamicforce_kuttajoukowski`](@ref) and
[`generate_aerodynamicforce_parasiticdrag`](@ref), and
[`calc_aerodynamicforce_unsteady`](@ref).
See [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/),
Sec. 6.3.3.
"""
function generate_calc_aerodynamicforce(; add_parasiticdrag=false,
                                          add_skinfriction=true,
                                          airfoilpolar="xf-n0012-il-500000-n5.csv",
                                          parasiticdrag_args=(),
                                          )


    # Aerodynamic force from Kutta-Joukowski's theorem
    kuttajoukowski = generate_aerodynamicforce_kuttajoukowski("regular",
                                                                nothing, nothing,
                                                                false, nothing,
                                                                nothing, nothing)
    # Parasitic drag
    if add_parasiticdrag
        parasiticdrag = generate_aerodynamicforce_parasiticdrag(airfoilpolar;
                                                                add_skinfriction=add_skinfriction,
                                                                parasiticdrag_args...)
    end

    # Force due to unsteady circulation
    unsteady(args...; optargs...) = calc_aerodynamicforce_unsteady(args...; add_to_Ftot=false, optargs...)


    function calc_aerodynamicforce(vlm_system, args...; per_unit_span=false, optargs...)

        # Delete any previous force field
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            pop!(vlm_system.sol, fieldname)
        end

        # Calculate unsteady force
        Ftot = unsteady(vlm_system, args...; per_unit_span=per_unit_span, optargs...)

        # Calculate Kutta-Joukowski force
        Ftot = kuttajoukowski(vlm_system, args...; per_unit_span=per_unit_span, optargs...)

        # Calculate and add parasatic-drag force
        if add_parasiticdrag
            Ftot = parasiticdrag(vlm_system, args...;
                                    per_unit_span=per_unit_span, optargs...)
        end

        return Ftot

    end

    return calc_aerodynamicforce
end





# ------------ KUTTA-JOUKOWSKI FORCE -------------------------------------------
"""
    generate_aerodynamicforce_kuttajoukowski(KJforce_type::String,
                                sigma_vlm_surf, sigma_rotor_surf,
                                vlm_vortexsheet,
                                vlm_vortexsheet_overlap,
                                vlm_vortexsheet_distribution,
                                vlm_vortexsheet_sigma_tbv;
                                vehicle=nothing
                                )

Calculates the aerodynamic force at each element of a VLM system using its
current `Gamma` solution and the Kutta-Joukowski theorem. It saves the force
as the field `vlm_system.sol["Ftot"]`

This force calculated through the Kutta-Joukowski theorem uses the
freestream velocity, kinematic velocity, and wake-induced velocity on each bound
vortex. See [Alvarez' dissertation](https://scholarsarchive.byu.edu/etd/9589/),
Sec. 6.3.3.

# ARGUMENTS
* `vlm_vortexsheet::Bool`   : If true, the bound vorticity is approximated with
                                and actuator surface model through a vortex
                                sheet. If false, it is approximated with an
                                actuator line model with horseshoe vortex
                                filaments.
* `vlm_vortexsheet_overlap::Bool`       : Target core overlap between particles
                                representing the vortex sheet (if
                                `vlm_vortexsheet=true`).
* `vlm_vortexsheet_distribution::Function` : Vorticity distribution in vortex
                                sheet (see `g_uniform`, `g_linear`, and
                                `g_pressure`).
* `KJforce_type::String`    : If `vlm_vortexsheet=true`, it specifies how to
                                weight the force of each particle in the vortex
                                sheet. If `KJforce_type=="averaged"`, the KJ
                                force is a chordwise average of the force
                                experienced by the particles. If
                                `KJforce_type=="weighted"`, the KJ force
                                is chordwise weighted by the strength of each
                                particle. If `KJforce_type=="regular"`, the
                                vortex sheet is ignored, and the KJ force
                                is calculated from the velocity induced
                                at midpoint between the horseshoe filaments.
* `vehicle::VLMVehicle`     : If `vlm_vortexsheet=true`, it is expected that the
                                vehicle object is passed through this argument.
"""
function generate_aerodynamicforce_kuttajoukowski(KJforce_type::String,
                                sigma_vlm_surf, sigma_rotor_surf,
                                vlm_vortexsheet, vlm_vortexsheet_overlap,
                                vlm_vortexsheet_distribution,
                                vlm_vortexsheet_sigma_tbv;
                                vehicle=nothing
                                )

    function calc_aerodynamicforce_kuttajoukowski(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                    prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                    per_unit_span=false, spandir=[0, 1, 0],
                                    include_trailingboundvortex=false,
                                    Vout=nothing, lenout=nothing,
                                    lencrit=-1, debug=false)

        m = vlm.get_m(vlm_system)    # Number of horseshoes

        # Nodes of every horseshoe
        Ap = _get_Xs(vlm_system, "Ap")
        A = _get_Xs(vlm_system, "A")
        B = _get_Xs(vlm_system, "B")
        Bp = _get_Xs(vlm_system, "Bp")

        # Midpoints of bound vortices
        ApA = (Ap .+ A)/2
        AB = (A .+ B)/2
        BBp = (B .+ Bp)/2

        # Evaluate VPM on each midpoint
        Xs = vcat(ApA, AB, BBp)

        # Evaluate VLM on each midpoint
        Vvlm = vlm.Vind.(Ref(vlm_system), Xs; t=t, ign_col=true, ign_infvortex=true)

        # Evaluate Vinf on each midpoint
        Vinfs = Vinf.(Xs, t)

        # Evaluate kinematic velocity on each node
        Vtran = _Vkinematic(vlm_system, prev_vlm_system, dt; t=t,
                                                    targetX=["Ap", "A", "B", "Bp"])

        # Pre-allocate memory
        Ftot = [zeros(3) for i in 1:m]
        if debug
            Vvpms, Vvlms, Vtranmids = ([zeros(3) for i in 1:m] for j in 1:3)
        end

        # Bound vortices to include in force calculation
        vortices = include_trailingboundvortex ? (1:3) : (2:2)  # 1==ApA, 2==AB, 3==BBp



        if !vlm_vortexsheet || KJforce_type=="regular"

            ## NOTE: Instead of calling the VPM, we use what was calculated
            ## by `solve()`, which includes Rotor-on-VLM velocities
            # Vvpm = Vvpm_on_Xs(pfield, Xs; dt=dt)
            Vvpm = vcat(vlm_system.sol["Vvpm_ApA"], vlm_system.sol["Vvpm_AB"], vlm_system.sol["Vvpm_BBp"])

        elseif vehicle == nothing

            error("KJ force calculation based on vortex sheet has been
                    requested, but vehicle has not been provided.")

        else

            # ----- Estimate VPM velocity (including rotors) on vortex sheet

            # Generate function of static particles
            static_particles_function = generate_static_particle_fun(pfield, pfield,
                                            vehicle,
                                            sigma_vlm_surf, sigma_rotor_surf;
                                            vlm_vortexsheet=vlm_vortexsheet,
                                            vlm_vortexsheet_overlap=vlm_vortexsheet_overlap,
                                            vlm_vortexsheet_distribution=vlm_vortexsheet_distribution,
                                            vlm_vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv)

            # Pre-calculate direction of lifting bound vortices
            BVdir = [zeros(3) for i in 1:m]

            for i in 1:m
                BVdir[i] .= B[i]
                BVdir[i] .-= A[i]
                BVdir[i] ./= norm(BVdir[i])
            end

            # Add static particles representing the vortex sheet
            org_np = vpm.get_np(pfield)
            static_particles_function(pfield)

            # Nullify strength of vortex sheet particles to avoid double accounting for VLM velocity
            for P in vpm.iterate(pfield; include_static=true)

                if P.static[1] && 1 <= P.index[1] <= m

                    # Save strength for future reference
                    for i in 1:3; P.M[i] = P.Gamma[i]; end;

                    # Save index before it gets overwritten by the FMM
                    P.vol[1] = P.index[1]

                    # Zero it out
                    P.Gamma .*= 1e-14
                end
            end

            # Evaluate velocity in entire particle field
            vpm._reset_particles(pfield)
            pfield.UJ(pfield)

            # Collect velocities
            weights = zeros(m)
            Vvpm = [zeros(3) for i in 1:m]

            for P in vpm.iterate(pfield; include_static=true)

                # if P.static[1] && 1 < P.index[1] <= m
                if P.static[1] && 1 <= P.vol[1] <= m

                    # ind = P.index[1]
                    ind = Int(P.vol[1])

                    # Calculate weight of this static particle as projection to lifting BV
                    weight = KJforce_type=="weighted" ? abs(dot(view(P.M, 1:3), BVdir[ind])) :
                             KJforce_type=="averaged" ? 1.0 :
                             error("Invalid KJforce_type. Options are `\"regular\"`, `\"weighted\"`, or `\"averaged\"`; got $(KJforce_type).")

                    # Add weighted velocity
                    for i in 1:3
                        Vvpm[ind][i] += weight*P.U[i]
                    end
                    weights[ind] += weight

                end
            end

            # Divide by total weight to get the final weighted velocity
            Vvpm ./= weights

            Vvpm = vcat(Vvpm, Vvpm, Vvpm)
            # ------
        end


        # Calculate KJ force
        for i in 1:m                 # Iterate over horseshoes
            for j in vortices        # Iterate over bound vortices (1==ApA, 2==AB, 3==BBp)

                # Bound vortex' length vector
                if j==1
                    l = (A[i]-Ap[i])
                elseif j==2
                    l = (B[i]-A[i])
                else
                    l = (Bp[i]-B[i])
                end

                # Bound vortex' midpoint
                # X = Xs[i + m*(j-1)]

                # Kinematic velocity at midpoint
                Vtranmid = (Vtran[i + m*(j-1)] + Vtran[i + m*j])/2

                # Effective velocity at midpoint
                V = Vvpm[i + m*(j-1)] + Vvlm[i + m*(j-1)] + Vinfs[i + m*(j-1)] + Vtranmid
                if Vout != nothing
                    push!(Vout, [Vvpm[i + m*(j-1)], Vvlm[i + m*(j-1)], Vinfs[i + m*(j-1)], Vtranmid])
                end

                # Circulation
                Gamma = vlm_system.sol["Gamma"][i]

                # Normalization factor
                if per_unit_span
                    len = abs((B[i][1]-A[i][1])*spandir[1] + (B[i][2]-A[i][2])*spandir[2] + (B[i][3]-A[i][3])*spandir[3])
                else
                    len = 1
                end
                if lenout != nothing
                    push!(lenout, per_unit_span==false || len>lencrit ? len : -10*lencrit)
                end

                # Kutta–Joukowski theorem: F = rho * V × vecGamma
                if per_unit_span==false || len > lencrit # NOTE: Use lencrit to avoid dividing by zero
                    Ftot[i][1] += rho * Gamma * (V[2]*l[3] - V[3]*l[2]) / len
                    Ftot[i][2] += rho * Gamma * (V[3]*l[1] - V[1]*l[3]) / len
                    Ftot[i][3] += rho * Gamma * (V[1]*l[2] - V[2]*l[1]) / len

                    if debug && j==2
                        Vtranmids[i] .+= Vtranmid
                        Vvpms[i] .+= Vvpm[i + m*(j-1)]
                        Vvlms[i] .+= Vvlm[i + m*(j-1)]
                    end
                end

            end
        end


        if vlm_vortexsheet && KJforce_type!="regular"
            # Remove static particles
            for pi in vpm.get_np(pfield):-1:org_np+1
                vpm.remove_particle(pfield, pi)
            end
        end

        if debug
            # Save Kutta-Joukowski force as a solution field
            vlm._addsolution(vlm_system, (per_unit_span ? "f" : "F")*"rk-vector", deepcopy(Ftot); t=t)
            vlm._addsolution(vlm_system, "Vtranmid-vector", Vtranmids; t=t)
            vlm._addsolution(vlm_system, "Vvpm-vector", Vvpms; t=t)
            vlm._addsolution(vlm_system, "Vvlm-vector", Vvlms; t=t)
        end

        # Add Kutta-Joukowski force to any existing force calculation
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            Ftot .+= vlm_system.sol[fieldname]
        end

        # Save total force as a solution field
        vlm._addsolution(vlm_system, fieldname, Ftot; t=t)

        return Ftot
    end

    return calc_aerodynamicforce_kuttajoukowski
end





# ------------ UNSTEADY-CIRCULATION FORCE --------------------------------------
"""
    calc_aerodynamicforce_unsteady(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                per_unit_span=false, spandir=[0, 1, 0],
                                include_trailingboundvortex=false,
                                add_to_Ftot=false
                                )

Force from unsteady circulation.

This force tends to add a lot of numerical noise, which in most cases ends up
cancelling out when the loading is time-averaged. Hence, `add_to_Ftot=false`
will calculate the unsteady loading and save it under the field `Funs-vector`,
but it will not be added to the Ftot vector that is used to calculate the
wing's overall aerodynamic force.
"""
function calc_aerodynamicforce_unsteady(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                per_unit_span=false, spandir=[0, 1, 0],
                                include_trailingboundvortex=false,
                                Vout=nothing, lenout=nothing,
                                lencrit=-1, debug=false,
                                add_to_Ftot=false # Use false to calculate unsteady force but not add it
                                )

    m = vlm.get_m(vlm_system)    # Number of horseshoes

    # Nodes of every horseshoe
    Ap = _get_Xs(vlm_system, "Ap")
    A = _get_Xs(vlm_system, "A")
    B = _get_Xs(vlm_system, "B")
    Bp = _get_Xs(vlm_system, "Bp")

    Ftot = [zeros(3) for i in 1:m]
    area = zeros(3)


    if ("Gamma" in keys(prev_vlm_system.sol)) # Case that solution from previous step is available

        for i in 1:m                 # Iterate over horseshoes
            for j in 1:2             # Iterate over triangles of the horseshoe

                # Sides of the parallelogram
                if j==1
                    l1 = Ap[i] - A[i]
                    l2 = B[i]  - A[i]
                else
                    l1 = A[i]  - B[i]
                    l2 = Bp[i] - B[i]
                end

                # Calculate area-normal vector
                cross!(area, l1, l2)    # Calculate area of the parallelogram
                area ./= 2              # Convert to area of the triangle
                area ./= 1-vlm.pn       # Convert to area of panel instead of horseshoe

                # Circulation
                Gamma = vlm_system.sol["Gamma"][i]
                prev_Gamma = prev_vlm_system.sol["Gamma"][i]
                dGammadt = (Gamma-prev_Gamma)/dt

                # Normalization factor
                if per_unit_span
                    len = abs((B[i][1]-A[i][1])*spandir[1] + (B[i][2]-A[i][2])*spandir[2] + (B[i][3]-A[i][3])*spandir[3])
                else
                    len = 1
                end

                # Unsteady force: F = -rho dGamma/dt A normal
                if per_unit_span==false || len > lencrit # NOTE: Use lencrit to avoid dividing by zero
                    Ftot[i][1] -= rho * dGammadt * area[1] / len
                    Ftot[i][2] -= rho * dGammadt * area[2] / len
                    Ftot[i][3] -= rho * dGammadt * area[3] / len
                end

            end
        end

    end

    if debug || !add_to_Ftot
        # Save unsteady force as a solution field
        vlm._addsolution(vlm_system, (per_unit_span ? "f" : "F")*"uns-vector", deepcopy(Ftot); t=t)
    end


    if add_to_Ftot

        # Add unsteady force to any existing force calculation
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            Ftot .+= vlm_system.sol[fieldname]
        end

        # Save total force as a solution field
        vlm._addsolution(vlm_system, fieldname, Ftot; t=t)

        return Ftot

    else

        return [zeros(3) for i in 1:m]

    end
end



# ------------ PARASITIC-DRAG FORCE --------------------------------------------
"""
    generate_aerodynamicforce_parasiticdrag(polar_file::String;
                                                    read_path=FLOWUnsteady.default_database*"/airfoils",
                                                    calc_cd_from_cl=false,
                                                    add_skinfriction=true,
                                                    Mach=nothing)

Calculates the parasitic drag along the wing using a lookup airfoil table. It
adds this force to the field `vlm_system.sol["Ftot"]`.

The lookup table is read from the file `polar_file` under the directory
`read_path`. The parasitic drag includes form drag, skin friction, and
wave drag, assuming that each of these components are included in the lookup
polar. `polar_file` can be any polar file downloaded from
[airfoiltools.com](http://airfoiltools.com/).

To ignore skin friction drag, use `add_skinfriction=false`.

The drag will be calculated from the local effective angle of attack, unless
`calc_cd_from_cl=true` is used, which then will be calculated from the local
lift coefficient given by the circulation distribution. cd from cl tends to be
more accurate than from AOA, but the method might fail to correlate cd and cl
depending on how noise the polar data is.

If `Mach != nothing`, it will use a Prandtl-Glauert correction to pre-correct
the lookup cl. This will have no effects if `calc_cd_from_cl=false`.
"""
function generate_aerodynamicforce_parasiticdrag(polar_file::String;
                                                    read_path::String=joinpath(default_database, "airfoils"),
                                                    read_polar=vlm.ap.read_polar,
                                                    calc_cd_from_cl=false,
                                                    add_skinfriction=true,
                                                    Mach=nothing, CDmax=1.3, spl_k=3)

    # Read polar
    org_polar = read_polar(polar_file; path=read_path)

    alpha, cl = vlm.ap.get_cl(org_polar)

    if add_skinfriction

        cd = vlm.ap.get_cd(org_polar)[2]

    else

        header = ["Alpha", "Cl", "Cd", "Cdp", "Cm", "Top_Xtr", "Bot_Xtr"]
        data = CSV.read(joinpath(read_path, polar_file), DataFrames.DataFrame, skipto=12, header=header)

        cd = data.Cdp

    end

    # Mach correction (affects only lift)
    if Mach!=nothing
        polar = vlm.ap.Polar(vlm.ap.get_Re(org_polar), alpha, cl/sqrt(1-(Mach)^2),
                             cd, vlm.ap.get_cm(org_polar)[2]; vlm.ap._get_nonpypolar_args(org_polar)...)
     else
         # polar = org_polar
         polar = vlm.ap.Polar(vlm.ap.get_Re(org_polar), alpha, cl,
                              cd, vlm.ap.get_cm(org_polar)[2]; vlm.ap._get_nonpypolar_args(org_polar)...)
     end

    # Extrapolate to -180 and 180 deg
    polar = vlm.ap.extrapolate(polar, CDmax)

    # Makes sure the polar is injective to ease the spline
    polar = vlm.ap.injective(polar)

    # Extract corrected data
    alpha, cl = vlm.ap.get_cl(polar)
    _, cd = vlm.ap.get_cd(polar)

    # Spline through data (NOTE: AOA in radians!)
    k = min(length(alpha)-1, spl_k)
    spl_cl = Dierckx.Spline1D(alpha*pi/180.0, cl; k=k, s=0.1)
    spl_cd = Dierckx.Spline1D(alpha*pi/180.0, cd; k=k, s=0.001)

    if calc_cd_from_cl
        # Make alpha(cl) an injective function
        cl_mini = findmin(cl)[2]
        cl_maxi = findmax(cl)[2]
        cl_inj = cl[cl_mini:cl_maxi]
        cd_inj = cd[cl_mini:cl_maxi]

        # Spline cd as a function of cl
        spl_cdfromcl = Dierckx.Spline1D(cl_inj, cd_inj; k=k, s=0.1, bc="nearest")
    end

    # Create function for evaluation parasitic drag from the polar
    function calc_aerodynamicforce_parasiticdrag(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                                    prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                                    per_unit_span=false, spandir=[0, 1, 0],
                                                    include_trailingboundvortex=false,
                                                    Vout=nothing, lenout=nothing,
                                                    lencrit=-1, debug=false)

        m = vlm.get_m(vlm_system)    # Number of horseshoes

        # Nodes of every horseshoe
        Ap = _get_Xs(vlm_system, "Ap")
        A = _get_Xs(vlm_system, "A")
        B = _get_Xs(vlm_system, "B")
        Bp = _get_Xs(vlm_system, "Bp")

        # Midpoint of each lifting bound vortex segment
        AB = (A .+ B)/2


        # NOTE: Instead of calling the VPM, we use what was calculated
        # by `solve()`, which includes Rotor-on-VLM velocities
        Vvpm = vlm_system.sol["Vvpm_AB"]
        # Vvpm = [zeros(3) for i in 1:m]


        # Evaluate VLM on each midpoint
        # Vvlm = vlm.Vind.(Ref(vlm_system), AB; t=t, ign_col=true, ign_infvortex=true)
        # Vvlm = [zeros(3) for i in 1:m]

        # Evaluate Vinf on each midpoint
        Vinfs = Vinf.(AB, t)

        # Evaluate kinematic velocity on each node
        Vtran = _Vkinematic(vlm_system, prev_vlm_system, dt; t=t, targetX=["A", "B"])

        Ftot = [zeros(3) for i in 1:m]
        l, leff, Vtranmid, V = (zeros(3) for j in 1:4)
        that, shat, nhat, dhat = (zeros(3) for j in 1:4)
        lAAp, lBBp = (zeros(3) for j in 1:2)

        if debug
            AOAeffs, cds, cls = (zeros(m) for j in 1:3)
        end

        for i in 1:m                 # Iterate over horseshoes

            # Lifting bound vortex
            broadcast!(-, l, B[i], A[i])

            # Trailing bound vortices
            broadcast!(-, lAAp, Ap[i], A[i])
            broadcast!(-, lBBp, Bp[i], B[i])

            # Vector tangent to horseshoe
            broadcast!(+, that, lAAp, lBBp)
            that ./= norm(that)

            # Effective span direction
            broadcast!(*, leff, that, dot(l, that))
            broadcast!(-, leff, l, leff)
            broadcast!(*, shat, leff, 1/norm(leff))

            # Vector normal to horseshoe
            cross!(nhat, that, shat)

            # Chord length
            chord = 1/(1-vlm.pn) * (norm(lAAp) + norm(lBBp))/2

            # Kinematic velocity at midpoint of lifting BV
            broadcast!(+, Vtranmid, Vtran[i], Vtran[i + m])
            Vtranmid ./= 2

            # Effective velocity at midpoint of lifting BV
            # broadcast!(+, V, Vvpm[i], Vvlm[i], Vinfs[i], Vtranmid)
            broadcast!(+, V, Vvpm[i], Vinfs[i], Vtranmid)

            # Decompose velocity vector between component tangent and normal to horseshoe
            Vt = dot(V, that)
            Vn = dot(V, nhat)
            magV = sqrt(Vt^2 + Vn^2)

            # Effective AOA at this cross section (in radians)
            AOAeff = atan(Vn, Vt)

            # Drag coefficient from cl
            if calc_cd_from_cl

                Gamma = vlm_system.sol["Gamma"][i]
                this_cl = Gamma / (1/2*magV*chord)
                this_cd = spl_cdfromcl(this_cl)

                if debug
                    cls[i] = this_cl
                end

            # Drag coefficient from effective AOA
            else
                this_cd = spl_cd(AOAeff)
            end

            # Dimensional sectional drag
            d = this_cd * 0.5*rho*magV^2*chord

            # Drag of the horseshoe
            D = d * norm(leff)

            # Drag direction
            for j in 1:3; dhat[j] = Vt*that[j] + Vn*nhat[j]; end
            dhat ./= norm(dhat)

            # Normalization factor
            if per_unit_span
                len = abs(l[1]*spandir[1] + l[2]*spandir[2] + l[3]*spandir[3])
            else
                len = 1
            end
            if lenout != nothing
                push!(lenout, per_unit_span==false || len>lencrit ? len : -10*lencrit)
            end

            # Parasitic drag: F = cd * (0.5*rho*V^2*c) * l * Vhat
            if per_unit_span==false || len > lencrit # NOTE: Use lencrit to avoid dividing by zero
                Ftot[i] .= dhat
                Ftot[i] .*= D / len

                if debug
                    AOAeffs[i] = AOAeff
                    cds[i] = this_cd
                end
            end

        end

        if debug
            # Save parasitic-drag force as a solution field
            vlm._addsolution(vlm_system, (per_unit_span ? "f" : "F")*"pd-vector", deepcopy(Ftot); t=t)

            vlm._addsolution(vlm_system, "AOAeff-pd-scalar", AOAeffs; t=t)
            vlm._addsolution(vlm_system, "cl-pd-scalar", cls; t=t)
            vlm._addsolution(vlm_system, "cd-pd-scalar", cds; t=t)
        end

        # Add parasitic-drag force to any existing force calculation
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            Ftot .+= vlm_system.sol[fieldname]
        end

        # Save total force as a solution field
        vlm._addsolution(vlm_system, fieldname, Ftot; t=t)

        return Ftot
    end

    return calc_aerodynamicforce_parasiticdrag
end


# ------------ FREE-VORTEX FORCE -----------------------------------------------
"""
    Returns an aerodynamic-force function for calculating the free-vortex force.

# ARGUMENTS
* `maxboundparticles::Int`      :   Maximum number of bound vortices. This is
                                    used for memory pre-allocation.
* `include_TBVs::Bool`          :   Whether to include force on trailing bound
                                    vortices.
"""
function generate_calc_aerodynamicforce_freevortices(maxboundparticles::Int,
                                sigma_vlm_surf,
                                vlm_vortexsheet, vlm_vortexsheet_overlap,
                                vlm_vortexsheet_distribution,
                                vlm_vortexsheet_sigma_tbv;
                                                        Ffv::Function=Ffv_direct,
                                                        include_TBVs::Bool=false,
                                                        save_path=nothing
                                                        )


    pfield_FV = vpm.ParticleField(maxboundparticles)

    function calc_aerodynamicforce_freevortices(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                    prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                    per_unit_span=false, spandir=[0, 1, 0],
                                    include_trailingboundvortex=false,
                                    Vout=nothing, lenout=nothing,
                                    lencrit=-1, debug=false)

        m = vlm.get_m(vlm_system)    # Number of horseshoes

        # Nodes of every horseshoe
        A = _get_Xs(vlm_system, "A")
        B = _get_Xs(vlm_system, "B")

        # Pre-allocate memory
        Ftot = [zeros(3) for i in 1:m]
        Fbxf = [zeros(3) for i in 1:m]

        vortices = include_TBVs ? (1:3) : (1:1)  # 1==AB, 2==ApA, 3==BBp

        # Add particles representing the VLM surface
        _static_particles(pfield_FV, vlm_system, sigma_vlm_surf;
                                    vortexsheet=vlm_vortexsheet,
                                    vortexsheet_overlap=vlm_vortexsheet_overlap,
                                    vortexsheet_distribution=vlm_vortexsheet_distribution,
                                    vlm_vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv,
                                    vortices=vortices
                                    )

        # Evaluate F/rho = V × vecGamma
        Ffv(pfield_FV, pfield)

        # Save field for debugging
        if debug && save_path != nothing
            for P in vpm.iterate(pfield_FV)
                P.C .= P.U
            end
            vpm.save(pfield_FV, "pfield_FV"; num=pfield.nt, path=save_path)
        end

        # Accumulate F/rho = V × vecGamma from each section of the horseshoe
        for P in vpm.iterator(pfield_FV; include_static=true)
            for k in 1:3
                # NOTE: P.index won't work if we ever implement Ffv through the FMM
                Ftot[P.index][k] += P.M[k]
                Fbxf[P.index][k] += P.M[3+k]
            end
        end

        # Normalize the force to be per unit span
        if per_unit_span
            for i in 1:m
                # Spanwise length of this section
                len = abs((B[i][1]-A[i][1])*spandir[1] + (B[i][2]-A[i][2])*spandir[2] + (B[i][3]-A[i][3])*spandir[3])

                if len > lencrit # NOTE: Use lencrit to avoid dividing by zero
                    Ftot[i] ./= len
                    Fbxf[i] ./= len
                end
            end
        end

        # Multiply everything by density
        for i in 1:m

            # Ffv = ρ ∑Ub(xf) × Γf
            Ftot[i] .*= rho

            # Fb×f = ρ ∑ g_σ K(xb-xf) × (Γb×Γf)
            Fbxf[i] .*= rho

        end

        # Remove particles from FV auxiliary field
        for i in vpm.get_np(pfield_FV):-1:1
            vpm.remove_particle(pfield_FV, i)
        end

        # Save free-vortex force as a solution field
        if debug
            vlm._addsolution(vlm_system, (per_unit_span ? "f" : "F")*"fv-vector", deepcopy(Ftot); t=t)
            vlm._addsolution(vlm_system, (per_unit_span ? "f" : "F")*"bxf-vector", deepcopy(Fbxf); t=t)
        end

        # Add free-vortex force to any existing force calculation
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            Ftot .+= vlm_system.sol[fieldname]
        end

        # Save total force as a solution field
        vlm._addsolution(vlm_system, fieldname, Ftot; t=t)

        return Ftot
    end

    return calc_aerodynamicforce_freevortices

end


function _Ffv_direct(source::vpm.ParticleField, target::vpm.ParticleField)
  return _Ffv_direct( vpm.iterator(source; include_static=true),
                    vpm.iterator(target; include_static=true),
                    source.kernel)
end

function _Ffv_direct(sources, targets, kernel::vpm.Kernel)
 return _Ffv_direct(sources, targets, kernel.g_dgdr)
end

"""
Calculate free-vortex force exherted in particle field `targets` by each bound
vortex in `sources`.
It stores Ffv/rho under `P.M[1:3]` and Fbxp/rho under `P.M[4:6]`, which are the
reaction forces of the sources.

NOTE: This implementation is a direct P2P, hence it will take a long time when
`targets` is large.
"""
function _Ffv_direct(sources, targets, g_dgdr::Function)

    Threads.@threads for Pb in sources      # The bound vortex inducing velocity

        # Flush auxiliary memory
        Pb.M .= 0

        # Accumulate contributions from each free vortex
        for Pf in targets                   # The free vortex

            # Δx = xf-xb
            dX1 = Pf.X[1] - Pb.X[1]
            dX2 = Pf.X[2] - Pb.X[2]
            dX3 = Pf.X[3] - Pb.X[3]
            r = sqrt(dX1*dX1 + dX2*dX2 + dX3*dX3)

            if r!=0

                # Regularizing function and deriv
                g_sgm, dg_sgmdr = g_dgdr(r/Pb.sigma[1])

                # ---- Ffv/ρ = ∑Ub(xf) × Γf ------------------------------------

                # Ub(xf) = g_σ(xf-xb) * K(xf-xb) × Γb
                U1 = -g_sgm * 0.07957747154594767 / r^3 * ( dX2*Pb.Gamma[3] - dX3*Pb.Gamma[2] )
                U2 = -g_sgm * 0.07957747154594767 / r^3 * ( dX3*Pb.Gamma[1] - dX1*Pb.Gamma[3] )
                U3 = -g_sgm * 0.07957747154594767 / r^3 * ( dX1*Pb.Gamma[2] - dX2*Pb.Gamma[1] )

                # ∑ Ub × Γf
                Pb.M[1] += U2*Pf.Gamma[3] - U3*Pf.Gamma[2]
                Pb.M[2] += U3*Pf.Gamma[1] - U1*Pf.Gamma[3]
                Pb.M[3] += U1*Pf.Gamma[2] - U2*Pf.Gamma[1]

                # ---- Fb×f/ρ = ∑ g_σ K(xb-xf) × (Γb×Γf) -----------------------

                # Γb × Γf
                crss1 = Pb.Gamma[2]*Pf.Gamma[3] - Pb.Gamma[3]*Pf.Gamma[2]
                crss2 = Pb.Gamma[3]*Pf.Gamma[1] - Pb.Gamma[1]*Pf.Gamma[3]
                crss3 = Pb.Gamma[1]*Pf.Gamma[2] - Pb.Gamma[2]*Pf.Gamma[1]

                # ∑ g_σ K(xb-xf) × (Γb×Γf)   (-1 cancels flipping Δx = xb-xf)
                Pb.M[4] += g_sgm * 0.07957747154594767 / r^3 * ( dX2*crss3 - dX3*crss2 )
                Pb.M[5] += g_sgm * 0.07957747154594767 / r^3 * ( dX3*crss1 - dX1*crss3 )
                Pb.M[6] += g_sgm * 0.07957747154594767 / r^3 * ( dX1*crss2 - dX2*crss1 )

            end
        end
    end

    return nothing
end
