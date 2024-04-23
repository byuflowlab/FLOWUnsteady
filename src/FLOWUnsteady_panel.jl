#=##############################################################################
# DESCRIPTION
    Prototyping integration of FLOWPanel

# ABOUT
  * Created   : Apr 2023
  * License   : MIT


# TODO
* [ ] Validate solver with MultiBody
* [x] Verify that least-square solver in `panel_solver` also applies to open
        bodies or revisit
* [x] Avoid shedding at interface of contiguous bodies
* [ ] Spread the unsteady shedding into a sheet
* [ ] Verify correct implementation of `panel_solver` on a moving body
        (kinematic velocity)
=###############################################################################


"""
Generate and return a function that solves for the strength of the panels
imposing the no-flow-through condition.

The solver function must first be initialized by the user calling it as
`panel_solver(simulation, nothing, nothing, nothing; initialize=true)` before
being passed to the simulation as
`run_simulation(...; panel_solver=panel_solver, ...)`

NOTE: `panel_solver()` currently pre-computes the LU decomposition of the
        left-hand-side of the system of equations, which is valid as long as the
        position and orientation of all panels relative to each other is
        constant (meaning, it is ok if the paneled bodies are moving and
        rotating throughout the simulation as long as they all do so together).
"""
function generate_panel_solver(sigma_rotor, sigma_vlm, ref_magVinf, ref_rho;
                                sigmafactor_vpmonpanel=1.0,
                                calc_elprescribe=pnl.calc_elprescribe,
                                rlx=-1,
                                offset_U=0,                  # Offset CPs by this amount in U calc
                                save_path=nothing,
                                run_name=run_name,
                                clip_Cp=nothing,
                                userdefined_postprocessing=(args...; optargs...)->nothing
                                )

    normals, normalscorr = nothing, nothing
    controlpoints, prev_controlpoints, off_controlpoints = nothing, nothing, nothing
    Vkin, Vtot = nothing, nothing
    Vas, Vbs = nothing, nothing
    Das, Dbs = nothing, nothing
    Xsheddings, prev_Xsheddings, Xs = nothing, nothing, nothing
    allrotors = nothing
    Us, Ugradmus, Ugradmus_cell, Ugradmus_node, Fs = nothing, nothing, nothing, nothing, nothing
    areas, Cps = nothing, nothing

    solverprealloc = nothing
    elprescribe = nothing
    mbp, Glu, tGred = nothing, nothing, nothing
    prevGammals = nothing

    Vind = nothing
    Gpanelt, Gpaneln, Gpanelo = nothing, nothing, nothing
    Vpanelt, Vpaneln, Vpanelo = nothing, nothing, nothing
    tangents, obliques = nothing, nothing

    Xsheddingss, prev_Xsheddingss, oldstrengthss = [], [], []

    function panel_solver(sim, PFIELD, T, DT; initialize=false, optargs...)

        vhcl      = sim.vehicle

        # Fetch panel body, VLMs, and rotors
        panelbody = vhcl.panel_system
        allvlms   = vhcl.vlm_system

        ncells = panelbody.ncells
        nsheddings = panelbody.nsheddings

        if initialize
            allrotors = vlm.WingSystem()

            for (si, rotors) in enumerate(vhcl.rotor_systems)
                for (ri, rotor) in enumerate(rotors)
                    vlm.addwing(allrotors, "S$(si)R$(ri)", rotor; reset=false)
                end
            end

            # Precompute normals and control points
            if normals == nothing; normals = pnl.calc_normals(panelbody); end;
            if normalscorr == nothing; normalscorr = pnl.calc_normals(panelbody; flipbyCPoffset=true); end;
            if controlpoints == nothing; controlpoints = pnl.calc_controlpoints(panelbody, normals); end;

            # Precompute unit vectors and offset control points for U calc
            if tangents == nothing; tangents = pnl.calc_tangents(panelbody); end;
            if obliques == nothing; obliques = pnl.calc_obliques(panelbody); end;
            if off_controlpoints == nothing
                off_controlpoints = pnl.calc_controlpoints(panelbody, normals; off=offset_U)
            end

            # Initialize storage
            # if normals == nothing; normals = zeros(3, ncells); end;
            # if controlpoints == nothing; controlpoints = zeros(3, ncells); end;
            # if prev_controlpoints == nothing; prev_controlpoints = zeros(3, ncells); end;
            # if off_controlpoints == nothing; off_controlpoints = zeros(3, ncells); end;
            if prev_controlpoints == nothing; prev_controlpoints = similar(controlpoints); end;
            # if off_controlpoints == nothing; off_controlpoints = similar(controlpoints); end;
            if Vkin == nothing; Vkin = zeros(3, ncells); end;
            if Vtot == nothing; Vtot = zeros(3, ncells); end;
            if Vas == nothing; Vas = zeros(3, nsheddings); end;
            if Vbs == nothing; Vbs = zeros(3, nsheddings); end;
            if Das == nothing; Das = zeros(3, nsheddings); end;
            if Dbs == nothing; Dbs = zeros(3, nsheddings); end;
            if Xsheddings == nothing; Xsheddings = zeros(3, 2*nsheddings); end;
            if prev_Xsheddings == nothing; prev_Xsheddings = zeros(3, 2*nsheddings); end;
            if Xs == nothing; Xs = zeros(3, ncells + 2*nsheddings); end;
            if Us == nothing; Us = zeros(3, ncells); end;
            if Ugradmus == nothing; Ugradmus = zeros(3, ncells); end;
            if Ugradmus_cell == nothing; Ugradmus_cell = zeros(3, ncells); end;
            if Ugradmus_node == nothing; Ugradmus_node = zeros(3, ncells); end;
            if Fs == nothing; Fs = zeros(3, ncells); end;
            if areas == nothing; areas = zeros(ncells); end;
            if Cps == nothing; Cps = zeros(ncells); end;
            if Vind == nothing; Vind = [zeros(3) for i in 1:(ncells+2*nsheddings)]; end;
            if Gpanelt == nothing; Gpanelt = zeros(ncells, ncells); end;
            if Gpaneln == nothing; Gpaneln = zeros(ncells, ncells); end;
            if Gpanelo == nothing; Gpanelo = zeros(ncells, ncells); end;
            if Vpanelt == nothing; Vpanelt = zeros(ncells); end;
            if Vpaneln == nothing; Vpaneln = zeros(ncells); end;
            if Vpanelo == nothing; Vpanelo = zeros(ncells); end;

            # Add Xsheddings storage space to body
            function create_Xsheddings_field(body::pnl.RigidWakeBody)
                this_Xsheddings = zeros(3, body.nsheddings*2)
                this_prev_Xsheddings = zeros(3, body.nsheddings*2)

                pnl.add_field(body, "Xsheddings", "vector", this_Xsheddings,
                                                "system"; collectfield=false)
                pnl.add_field(body, "prev_Xsheddings", "vector", this_prev_Xsheddings,
                                                "system"; collectfield=false)

                # Allocate memory for storing TE circulation
                oldstrengths = fill(NaN, body.nsheddings)
                pnl.add_field(body, "oldstrengths", "vector", oldstrengths,
                                                "system"; collectfield=false)

                push!(Xsheddingss, this_Xsheddings)
                push!(prev_Xsheddingss, this_prev_Xsheddings)
                push!(oldstrengthss, oldstrengths)
            end
            applytobottom(create_Xsheddings_field, panelbody)

            # Pre-allocate solver memory
            elprescribe = calc_elprescribe(panelbody)
            solverprealloc = pnl.allocate_solver(panelbody, elprescribe, Float64)

            (; Gamma, Gammals, G, Gred, tGred, gpuGred, Gls, RHS, RHSls) = solverprealloc
            if mbp == nothing; mbp = similar(RHS); end;
            if prevGammals == nothing
                prevGammals = similar(Gammals)
                prevGammals .= NaN
            end

            print("Precomputing G matrix... ")
            t = @elapsed begin
                # Compute Gred, Gls=Gred'*Gred, and -bp
                Vtot .= 0         # Set Vinfs = 0 so then RHS becomes simply -bp

                pnl._G_U_RHS_leastsquares!(panelbody,
                                            G, Gred, tGred, gpuGred, Gls, RHS, RHSls,
                                            Vtot, controlpoints, normals,
                                            Das, Dbs,
                                            elprescribe;
                                            omit_wake=true
                                            )
                mbp .= RHS
                # tGred = transpose(Gred)
            end
            println("$(t) seconds")

            print("Precomputing LU decomposition... ")
            t = @elapsed begin

                # Precompute LU decomposition
                if length(elprescribe)==0
                    # Case of direct solver
                    Glu = pnl.calc_Alu!(G)
                    tGred = I
                    Gls .= G
                else
                    # Case of least-square solver
                    Glu = pnl.calc_Alu!(Gls)
                end

            end
            println("$(t) seconds")

            print("Precomputing self-induced velocity coeffs... ")
            t = @elapsed begin
                # Precompute coefficients of self-induced velocity
                pnl._G_Uvortexring!(panelbody, Gpanelt, off_controlpoints, tangents, Das, Dbs; omit_wake=true)
                pnl._G_Uvortexring!(panelbody, Gpaneln, off_controlpoints, normals,  Das, Dbs; omit_wake=true)
                pnl._G_Uvortexring!(panelbody, Gpanelo, off_controlpoints, obliques, Das, Dbs; omit_wake=true)
            end
            println("$(t) seconds")

            # Add body grids to the vehicle's array of grids for the vehicle
            # to translate and rotate the panel body along with it
            applytobottom(b -> push!(vhcl.grids, b.grid), panelbody)
            applytobottom(b -> push!(vhcl.grid_O, zeros(3)), panelbody)
            applytobottom(b -> push!(vhcl.grid_save, false), panelbody)

            return false
        end

        # Get panel control points
        prev_controlpoints .= controlpoints
        pnl.calc_normals!(panelbody, normals)
        pnl.calc_normals!(panelbody, normalscorr; flipbyCPoffset=true)
        pnl.calc_tangents!(panelbody, tangents)
        pnl.calc_obliques!(panelbody, obliques)
        pnl.calc_areas!(panelbody, areas)
        pnl.calc_controlpoints!(panelbody, controlpoints, normals)
        pnl.calc_controlpoints!(panelbody, off_controlpoints, normals; off=offset_U)

        # Get shedding points
        countsheddings = 0

        function calc_Xsheddings(body::pnl.RigidWakeBody)

            this_Xsheddings = pnl.get_field(body, "Xsheddings")["field_data"]

            grid = body.grid
            nodes = grid._nodes

            tricoor, quadcoor, lin, ndivscells, cin = gt.generate_getcellt_args(grid)

            # Iterate over sheddings
            for (ei, (pi, nia, nib, _, _, _)) in enumerate(eachcol(body.shedding))

                # Convert node indices from panel-local to global
                pia = gt.get_cell_t(tricoor, quadcoor, grid, pi, nia, lin, ndivscells, cin)
                pib = gt.get_cell_t(tricoor, quadcoor, grid, pi, nib, lin, ndivscells, cin)

                # Fetch shedding positions a and b
                for i in 1:3
                    Xsheddings[i, 2*countsheddings + 2*(ei-1) + 1] = nodes[i, pia]
                    Xsheddings[i, 2*countsheddings + 2*(ei-1) + 2] = nodes[i, pib]

                    this_Xsheddings[i, 2*(ei-1) + 1] = nodes[i, pia]
                    this_Xsheddings[i, 2*(ei-1) + 2] = nodes[i, pib]
                end

            end

            countsheddings += body.nsheddings

        end

        prev_Xsheddings .= Xsheddings

        applytobottom(calc_Xsheddings, panelbody)


        # Calculate kinematic velocity at each control point
        if sim.nt == 0
            Vkin .= 0
        else
            Vkin .= controlpoints
            Vkin .-= prev_controlpoints
            Vkin /= DT

            # Revert direction to reference frame moving with the control point
            Vkin *= -1
        end

        # Calculate velocity induced by VPM wake, rotors, and VLMs on control and shedding points
        function static_particles_fun(pfield, args...)

            # Rotor static particles
            _static_particles(pfield, allrotors, sigma_rotor)

            # VLM static particles
            _static_particles(pfield, allvlms, sigma_vlm)

        end

        Xs[:, 1:ncells] = controlpoints
        Xs[:, ncells+1:end] = Xsheddings

        for V in Vind; V .= 0; end;
        Vvpm_on_Xs!(Vind, PFIELD, eachcol(Xs); static_particles_fun=static_particles_fun,
                                                    dt=DT, fsgm=sigmafactor_vpmonpanel)

        Vindcp = view(Vind, 1:ncells)
        Vindsh = view(Vind, ncells+1:ncells+2*nsheddings)

        # Probe freestream velocity
        # TODO: Here I am using the Uinf(t) from the VPM. Use Vinf(X, t) from
        #       FLOWUnsteady instead when it is available
        for (X, V) in zip(eachcol(controlpoints), eachcol(Vtot))
            # V .= Vinf(X, T)
            V .= PFIELD.Uinf(T)
        end

        # Add all velocities together
        Vtot .+= Vkin

        for (V, Vi) in zip(eachcol(Vtot), Vindcp)
            V .+= Vi
        end

        # Unitary direction of semi-infinite vortex at points `a` and `b` of each
        # trailing edge panel
        for i in 1:nsheddings
            for (di, (Ds, Vs)) in enumerate( zip((Das, Dbs), (Vas, Vbs)) )

                # Kinematic velocity
                if sim.nt == 0
                    Vs[:, i] .= 0
                else
                    Vs[:, i] .= Xsheddings[:, 2*(i-1)+di]
                    Vs[:, i] .-= prev_Xsheddings[:, 2*(i-1)+di]
                    Vs[:, i] /= DT
                    Vs[:, i] *= -1
                end

                # Freestream
                Vs[:, i] += PFIELD.Uinf(T)

                # Induced velocity by VPM, wings, and rotors
                for j in 1:3
                    Vs[j, i] += Vindsh[2*(i-1)+di][j]
                end

                # Unitary direction
                for j in 1:3; Ds[j, i] = Vs[j, i]; end;
                Ds[:, i] /= norm(view(Ds, :, i))

            end
        end

        # Store velocity along trailing edge
        # add_field(self, "Va", "vector", collect(eachcol(Vas)), "system")
        # add_field(self, "Vb", "vector", collect(eachcol(Vbs)), "system")

        # ------------- Panel solver -------------------------------
        # solve(panelbody, Vtot, Das, Dbs; omit_wake=true)

        (; Gamma, Gammals, G, Gred, Gls, RHS, RHSls) = solverprealloc

        # Calculate normal velocity of freestream for boundary condition (RHS = -b)
        pnl.calc_bc_noflowthrough!(RHS, Vtot, normals)

        # Convert boundary condition into least-squares' boundary condition (RHS = -b-bp)
        RHS += mbp

        # Convert boundary condition into least-squares' RHS (RHSls = -G'*(b+bp))
        pnl.LA.mul!(RHSls, tGred, RHS)

        # Solve system of equations using precomputed LU decomposition
        pnl.solve_ludiv!(Gammals, Gls, RHSls; Alu=Glu)

        # Apply relaxation: Γ = α·Γnew + (1-α)·Γold      (omit in first step)
        if rlx > 0 && isnothing(findfirst(el->isnan(el), prevGammals))
            Gammals *= rlx
            prevGammals *= (1-rlx)
            Gammals += prevGammals
        end

        # Save solution
        pnl.set_solution(panelbody, Gamma, Gammals, elprescribe, Vtot, Das, Dbs)

        # Store current solution
        prevGammals .= Gammals

        # ----------------------------------------------------------

        # Calculate panel surface velocity
        Us .= 0
        # calcfield_U!(Us, panelbody, panelbody, off_controlpoints, Vtot; omit_wake=true)

        if !(panelbody isa pnl.MultiBody)

            # Stitch together the calculated Gammas and the prescribed ones
            prev_eli = 0
            for (i, (eli, elval)) in enumerate(elprescribe)

                Gamma[(prev_eli+1):(eli-1)] .= view(Gammals, (prev_eli+2-i):(eli-i))

                Gamma[eli] = elval

                if i==length(elprescribe) && eli!=length(Gamma)
                    Gamma[eli+1:end] .= view(Gammals, (eli-i+1):length(Gammals))
                end

                prev_eli = eli
            end

        else
            # println("LOGIC ERROR")
        end

        pnl.LA.mul!(Vpanelt, Gpanelt, Gamma)
        pnl.LA.mul!(Vpaneln, Gpaneln, Gamma)
        pnl.LA.mul!(Vpanelo, Gpanelo, Gamma)

        for (U, V, Vt, t, Vn, n, Vo, o) in zip(eachcol(Us), eachcol(Vtot),
                                            Vpanelt, eachcol(tangents),
                                            Vpaneln, eachcol(normals),
                                            Vpanelo, eachcol(obliques)
                                            )
            for i in 1:3
                U[i] += V[i] + Vt*t[i] + Vn*n[i] + Vo*o[i]
            end
        end

        pnl.add_field(panelbody, "U", "vector", eachcol(Us), "cell")


        # Calculate surface velocity U_∇μ due to the gradient of the doublet strength
        Ugradmus .= 0
        Ugradmus_cell .= 0
        Ugradmus_node .= 0
        pnl.calcfield_Ugradmu!(Ugradmus, Ugradmus_cell, Ugradmus_node,
                                panelbody, areas, normals, controlpoints; force_cellTE=false)

        # Add both velocities together
        pnl.addfields(panelbody, "Ugradmu", "U")

        # Calculate pressure coefficient (based on U + U_∇μ)
        Cps .= 0
        pnl.calcfield_Cp!(Cps, panelbody, Us, ref_magVinf; clip=clip_Cp)

        # Calculate the force of each panel (based on Cp)
        Fs .= 0
        pnl.calcfield_F!(Fs, panelbody, areas, normalscorr, Cps, ref_magVinf, ref_rho)

        # Restore fields erased by solve
        bodyi = 1
        function restore_Xsheddings_field(body::pnl.RigidWakeBody)

            pnl.add_field(body, "Xsheddings", "vector", Xsheddingss[bodyi],
                                                "system"; collectfield=false)
            pnl.add_field(body, "prev_Xsheddings", "vector", prev_Xsheddingss[bodyi],
                                                "system"; collectfield=false)
            pnl.add_field(body, "oldstrengths", "vector", oldstrengthss[bodyi],
                                                "system"; collectfield=false)

            bodyi += 1
        end
        applytobottom(restore_Xsheddings_field, panelbody)

        # Call user-defined postprocessing function
        userdefined_postprocessing(panelbody, areas, normals, controlpoints)

        # Output VTK files
        if save_path != nothing
            pnl.save(panelbody, run_name; path=save_path, num=sim.nt,
                                                out_wake=true,
                                                wake_panel=false,
                                                debug=false,
                                                suffix="_panel")
        end


        return false
    end

    return panel_solver
end





"""
Shed VPM wake from panel bodies

Use `omit_shedding` to omit shedding from specific TE panels in a body.
`omit_shedding[body][ei] = (bool_pa, bool_pb, bool_te)` indicates that the
body `body` should omit shedding from the `ei`-th TE panel. `bool_pa==true`
indicates to omit shedding the trailing vorticity from its pa point, `bool_pb`
from the pb point, and `bool_te` omits shedding the unsteady vorticity
transversal to the trailing.

TODO:
* [x] Implement `omit_shedding`
* [x] Implement starting vortex shedding
* [x] Implement unsteady shedding
* [ ] Spread the unsteady shedding into a sheet

"""
function shed_wake_panel(body::pnl.RigidWakeBody, Vinf::Function,
                            pfield, dt, nt;
                            t=0.0,
                            unsteady_shedcrit=-1.0,
                            shed_starting=false,
                            p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing,
                            # omit_shedding::Dict{<:pnl.AbstractBody, Dict{Int64, Tuple{Bool, Bool, Bool}}}=Dict(),
                            omit_shedding=Dict(),
                            tol=1e-6
                        )
    #=
    NOTE TO SELF: Even though the semi-infinite wake is defined to come
    in at pa, go from pa to pb, and go out at pb, lofts and bodies of
    revolution procedures were accidentally defined such that pb
    is the same than the previous pa if we go in sequential order
    that `sheddings` is defined, meaning that the semi-infinite
    horseshoe goes "from right to left", while the leading edge goes
    "from left to right". This shouldn't matter for the user, but it
    makes the implementation of this function quite confusing.

    NOTE TO SELF 2: The velocity used for shedding neglects all induced
    velocities (wake and surfaces), and uses only freestream and kinematic
    velocities. This is a gross simplification, but it saves one particle
    field evaluation. It is unclear how much inaccuracy this might be
    introducing.
    =#

    if nt==0 || body.nsheddings==0
        return
    end

    grid = body.grid
    nodes = grid._nodes
    npanels = body.ncells

    # Pre-allocate memory
    (tri_out, tricoor, quadcoor,
        quad_out, lin, ndivscells, cin) = gt.generate_getcellt_args!(grid)

    X, Gamma, V = zeros(3), zeros(3), zeros(3)
    TE = zeros(Int, 2)

    # Strength and pa position of previous shedding element
    prev_strength = 0.0
    prev_pa = zeros(3)

    # Vas = get_field(body, "Va")
    # Vbs = get_field(body, "Vb")
    sheddings = eachcol(body.shedding)

    # Position of shedding points in previous step
    prevstep_Xsheddings = pnl.get_field(body, "prev_Xsheddings")["field_data"]

    # Position of shedding points in this step
    Xsheddings = pnl.get_field(body, "Xsheddings")["field_data"]

    prevstep_Xsheddings .= Xsheddings    # Store previous Xsheddings

    # Trailing edge circulation from previous time step
    oldstrengths = pnl.get_field(body, "oldstrengths")["field_data"]

    noshedding = body in keys(omit_shedding) ? omit_shedding[body] : []

    # Fetch last shedding element to check closed geometry later on
    lasti = body.nsheddings
    last_strength = pnl._get_wakestrength_Gamma(body, lasti; stri=1)
    last_pi = body.shedding[1, lasti]
    last_nia = body.shedding[2, lasti]
    last_nib = body.shedding[3, lasti]
    last_panel = gt.get_cell_t!(tri_out, tricoor, quadcoor, quad_out,
                                        grid, last_pi, lin, ndivscells, cin)
    TE[1] = last_panel[last_nia]
    TE[2] = last_panel[last_nib]
    last_pa1, last_pa2, last_pa3 = nodes[1, TE[1]], nodes[2, TE[1]], nodes[3, TE[1]]
    last_pb1, last_pb2, last_pb3 = nodes[1, TE[2]], nodes[2, TE[2]], nodes[3, TE[2]]

    closed_geometry = false                  # Closed geometry flag

    # Iterate over wake-shedding panels
    for (ei, (pi, nia, nib, _, _, _)) in enumerate(sheddings)

        strength = pnl._get_wakestrength_Gamma(body, ei; stri=1)

        # Fetch nodes of upper wake panel
        panel = gt.get_cell_t!(tri_out, tricoor, quadcoor, quad_out,
                                            grid, pi, lin, ndivscells, cin)

        # Fetch nodes of the shedding edge
        TE[1] = panel[nia]
        TE[2] = panel[nib]
        pa1, pa2, pa3 = nodes[1, TE[1]], nodes[2, TE[1]], nodes[3, TE[1]]
        pb1, pb2, pb3 = nodes[1, TE[2]], nodes[2, TE[2]], nodes[3, TE[2]]

        # Length of shedding edge
        edgelength = sqrt((pb1-pa1)^2 + (pb2-pa2)^2 + (pb3-pa3)^2)

        # Store nodes of the shedding edge for kinematic velocity calc
        Xsheddings[1, 2*(ei-1) + 1] = pa1
        Xsheddings[2, 2*(ei-1) + 1] = pa2
        Xsheddings[3, 2*(ei-1) + 1] = pa3
        Xsheddings[1, 2*(ei-1) + 2] = pb1
        Xsheddings[2, 2*(ei-1) + 2] = pb2
        Xsheddings[3, 2*(ei-1) + 2] = pb3

        if nt==0; prevstep_Xsheddings .= Xsheddings; end; # No kinematic velocity on first step

        # Check for closed geometry
        if ei==1

            dist_firstlast = sqrt((last_pa1-pb1)^2 + (last_pa2-pb2)^2 + (last_pa3-pb3)^2)

            # Closed geometry  criterion
            closed_geometry = dist_firstlast/edgelength <= tol

            if closed_geometry
                # Make last pa the "previous" pa
                prev_pa[1] = last_pa1
                prev_pa[2] = last_pa2
                prev_pa[3] = last_pa3
                prev_strength = last_strength
            else
                # Inexistent previous pa if first element in open geometry
                prev_pa .= Inf
            end

        end

        dist_pbpreva = sqrt((prev_pa[1]-pb1)^2 + (prev_pa[2]-pb2)^2 + (prev_pa[3]-pb3)^2)

        if edgelength != 0

            strength_pb = strength

            middle = dist_pbpreva/edgelength <= tol

            # Case of middle panel: Shed from pb is strength-prev_strength
            if middle
                strength_pb -= prev_strength

            # Case that the previous panel was an end panel (discontinuous body): Shed from prev_pa
            elseif ei != 1

                # ----------------- Shed from prev_pa ----------------------------

                # V = Vas[ei-1]
                prev_pai = 2*(ei-1 - 1) + 1
                for i in 1:3
                    V[i] = -(Xsheddings[i, prev_pai] - prevstep_Xsheddings[i, prev_pai]) / dt
                end
                V += Vinf(view(Xsheddings, :, prev_pai), t)

                sigma = sqrt((V[1]*dt)^2 + (V[2]*dt)^2 + (V[3]*dt)^2) * sigmafactor

                # Filament end point
                prev_paend1 = prev_pa[1] + V[1]*dt
                prev_paend2 = prev_pa[2] + V[2]*dt
                prev_paend3 = prev_pa[3] + V[3]*dt

                # Shed previous incoming filament
                if unsteady_shedcrit<=0 && !(ei-1 in keys(noshedding) && noshedding[ei-1][1])
                    _add_filamentparticle!(pfield, X, Gamma,
                                            prev_strength,
                                            prev_paend1, prev_paend2, prev_paend3,
                                            prev_pa[1], prev_pa[2], prev_pa[3], sigma;
                                            p_per_step=p_per_step, overwrite_sigma=overwrite_sigma
                                            )
                end
            end

            # ----------------- Shed from pb -------------------------------------
            # V = Vbs[ei]
            pbi = 2*(ei-1) + 2
            for i in 1:3
                V[i] = -(Xsheddings[i, pbi] - prevstep_Xsheddings[i, pbi]) / dt
            end
            V += Vinf(view(Xsheddings, :, pbi), t)

            sigma = sqrt((V[1]*dt)^2 + (V[2]*dt)^2 + (V[3]*dt)^2) * sigmafactor

            # Filament end point
            pbend1 = pb1 + V[1]*dt
            pbend2 = pb2 + V[2]*dt
            pbend3 = pb3 + V[3]*dt


            # Shed outgoing filament
            if unsteady_shedcrit<=0 && !(ei in keys(noshedding) && noshedding[ei][2])
                _add_filamentparticle!(pfield, X, Gamma,
                                        -strength_pb,
                                        pbend1, pbend2, pbend3,
                                        pb1, pb2, pb3, sigma;
                                        p_per_step=p_per_step, overwrite_sigma=overwrite_sigma
                                        )
            end


            # Case of end panel in open geometry: Shed from pa
            if ei == length(sheddings) && !closed_geometry

                # ----------------- Shed from pa ----------------------------
                # V = Vas[ei]
                pai = 2*(ei-1) + 1
                for i in 1:3
                    V[i] = -(Xsheddings[i, pai] - prevstep_Xsheddings[i, pai]) / dt
                end
                V += Vinf(view(Xsheddings, :, pai), t)
                sigma = sqrt((V[1]*dt)^2 + (V[2]*dt)^2 + (V[3]*dt)^2) * sigmafactor

                # Filament end point
                paend1 = pa1 + V[1]*dt
                paend2 = pa2 + V[2]*dt
                paend3 = pa3 + V[3]*dt

                # Shed incoming filament
                if unsteady_shedcrit<=0 && !(ei in keys(noshedding) && noshedding[ei][1])
                    _add_filamentparticle!(pfield, X, Gamma,
                                            strength,
                                            paend1, paend2, paend3,
                                            pa1, pa2, pa3, sigma;
                                            p_per_step=p_per_step, overwrite_sigma=overwrite_sigma
                                            )
                end
            end


            # ----------------- Shed from TE ----------------------------
            # Case shedding unsteady vorticity at trailing edge
            if unsteady_shedcrit>0  && !(ei in keys(noshedding) && noshedding[ei][3])

                strength_old = oldstrengths[ei]

                # Case shedding starting vortex
                if isnan(strength_old) && shed_starting
                    strength_old = 0
                end

                if !isnan(strength_old)

                    # Unsteady circulation
                    strength_unsteady = -(strength - strength_old)

                    # Smoothing radius
                    sigma = edgelength * sigmafactor

                    # Effective velocity at pa
                    pai = 2*(ei-1) + 1
                    for i in 1:3
                        V[i] = -(Xsheddings[i, pai] - prevstep_Xsheddings[i, pai]) / dt
                    end
                    V += Vinf(view(Xsheddings, :, pai), t)

                    # Filament starting point: pa + dt*V(pa)
                    pstart1 = pa1 + V[1]*dt
                    pstart2 = pa2 + V[2]*dt
                    pstart3 = pa3 + V[3]*dt

                    # Effective velocity at pb
                    pbi = 2*(ei-1) + 2
                    for i in 1:3
                        V[i] = -(Xsheddings[i, pbi] - prevstep_Xsheddings[i, pbi]) / dt
                    end
                    V += Vinf(view(Xsheddings, :, pbi), t)

                    # Filament end point: pb + dt*V(pb)
                    pend1 = pb1 + V[1]*dt
                    pend2 = pb2 + V[2]*dt
                    pend3 = pb3 + V[3]*dt

                    # Shed vorticity only if strength is greater than certain threshold
                    if abs(strength_unsteady/strength_old) > unsteady_shedcrit

                        _add_filamentparticle!(pfield, X, Gamma,
                                                strength_unsteady,
                                                pstart1, pstart2, pstart3,
                                                pend1, pend2, pend3, sigma;
                                                p_per_step=p_per_step, overwrite_sigma=overwrite_sigma
                                                )

                    end
                end

                # Update old strengths for unsteady shedding
                oldstrengths[ei] = strength

            end

            prev_strength = strength
            prev_pa[1] = pa1
            prev_pa[2] = pa2
            prev_pa[3] = pa3

        end

    end

    return nothing
end


function shed_wake_panel(multibody::pnl.MultiBody, Vinf, pfield, dt, nt; optargs...)

    applytobottom(shed_wake_panel, multibody; args=[Vinf, pfield, dt, nt], optargs=optargs)

end


"""
Converts the filament of ends `pa` and `pb` and strength `strength` (going
from `pa` to `pb`) into a particle, and adds it to the particle field `pfield`
"""
function _add_filamentparticle!(pfield, X, Gamma,
                                strength, pa1, pa2, pa3, pb1, pb2, pb3, sigma;
                                p_per_step::Int=1, overwrite_sigma=nothing,
                                static=false)

    pps = p_per_step

    # Vectorial circulation
    Gamma[1] = pb1 - pa1
    Gamma[2] = pb2 - pa2
    Gamma[3] = pb3 - pa3
    Gamma *= strength/p_per_step

    # Smoothing radius
    if overwrite_sigma==nothing
        sigmap = sigma/p_per_step
    else
        sigmap = overwrite_sigma
    end

    # Particle position
    X[1] = pa1 - (pb1 - pa1)/pps/2
    X[2] = pa2 - (pb2 - pa2)/pps/2
    X[3] = pa3 - (pb3 - pa3)/pps/2

    for pi in 1:p_per_step

        X[1] += (pb1 - pa1)/pps
        X[2] += (pb2 - pa2)/pps
        X[3] += (pb3 - pa3)/pps

        # Add particle
        vpm.add_particle(pfield, X, Gamma, sigmap;
                            vol=0, circulation=abs(strength), static=static
                            )
    end
end




"""
Add a panel body to the computational domain of a particle field as
static vortex particles.
"""
function panel_static_particles(body::pnl.RigidWakeBody, pfield, sigma::Number;
                                                            p_per_filament=1)

    grid = body.grid
    nodes = grid._nodes
    npanels = body.ncells

    # Pre-allocate memory
    (tri_out, tricoor, quadcoor,
        quad_out, lin, ndivscells, cin) = gt.generate_getcellt_args!(grid)

    X, Gamma = zeros(3), zeros(3)
    TE = zeros(Int, 2)

    # Iterate over panels
    for pi in 1:npanels

        # Fetch panel
        panel = gt.get_cell_t!(tri_out, tricoor, quadcoor, quad_out,
                                        grid, pi, lin, ndivscells, cin)

        # NOTE: Here we hardcode the index of the vortex element
        strength = body.strength[pi, 1]

        ne = length(panel)                      # Number of edges


        # Iterate over edges
        for i in 1:ne

                # Fetch nodes of this edge
                ni, nj = panel[i], panel[i%ne + 1]
                pa1, pa2, pa3 = nodes[1, ni], nodes[2, ni], nodes[3, ni]
                pb1, pb2, pb3 = nodes[1, nj], nodes[2, nj], nodes[3, nj]

                _add_filamentparticle!(pfield, X, Gamma,
                                        strength, pa1, pa2, pa3, pb1, pb2, pb3,
                                        sigma;
                                        static=true,
                                        overwrite_sigma=sigma,
                                        p_per_step=p_per_filament)
        end

    end

    # Iterate over wake-shedding panels
    for (ei, (pi, nia, nib, pj, nja, njb)) in enumerate(eachcol(body.shedding))

        strengthi, strengthj = pnl._get_wakestrength_mu(body, ei; stri=1)

        # Fetch nodes of upper wake panel
        panel = gt.get_cell_t!(tri_out, tricoor, quadcoor, quad_out,
                                            grid, pi, lin, ndivscells, cin)

        # Fetch nodes of the shedding edge
        TE[1] = panel[nia]
        TE[2] = panel[nib]
        pa1, pa2, pa3 = nodes[1, TE[1]], nodes[2, TE[1]], nodes[3, TE[1]]
        pb1, pb2, pb3 = nodes[1, TE[2]], nodes[2, TE[2]], nodes[3, TE[2]]


        _add_filamentparticle!(pfield, X, Gamma,
                                strengthi, pa1, pa2, pa3, pb1, pb2, pb3, sigma,
                                static=true,
                                overwrite_sigma=sigma,
                                p_per_step=p_per_filament)

        if pj != -1

            # Fetch nodes of lower wake panel
            panel = gt.get_cell_t!(tri_out, tricoor, quadcoor, quad_out,
                                         grid, pj, lin, ndivscells, cin)

            # Indicate nodes in the lower shedding edge
            TE[1] = panel[nja]
            TE[2] = panel[njb]

            pa1, pa2, pa3 = nodes[1, TE[1]], nodes[2, TE[1]], nodes[3, TE[1]]
            pb1, pb2, pb3 = nodes[1, TE[2]], nodes[2, TE[2]], nodes[3, TE[2]]


            _add_filamentparticle!(pfield, X, Gamma,
                                    strengthj, pa1, pa2, pa3, pb1, pb2, pb3, sigma,
                                    static=true,
                                    overwrite_sigma=sigma,
                                    p_per_step=p_per_filament)

        end

    end

    return nothing
end


"""
Generate a function that adds static particles of panel bodies
"""
function generate_panel_static_particles_fun(body::pnl.AbstractBody,
                                                sigma::Number; p_per_filament=1)

    function panel_static_particles_fun(PFIELD, T, DT)

        applytobottom(panel_static_particles, body; args=(PFIELD, sigma),
                                                    optargs=(; p_per_filament))

    end

    return panel_static_particles_fun
end



"""
    applytobottom(f::Function, multibody::MultiBody)

Iterates over each body of `multibody` applying `f(component)` if the
body is not a `MultiBody`, or applying recursion otherwise.
"""
function applytobottom(f::Function, multibody::pnl.MultiBody; optargs...)
    for body in multibody.bodies
        applytobottom(f, body; optargs...)
    end
end

function applytobottom(f::Function, body::pnl.RigidWakeBody; args=[], optargs=[])
    return f(body, args...; optargs...)
end
