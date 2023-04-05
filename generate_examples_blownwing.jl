# ------------- BLOWN WING EXAMPLE ---------------------------------------------
output_name = "blownwing"
example_path = joinpath(uns.examples_path, "blownwing")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# -------- Aero Solution --------------------------------------------------------
open(joinpath(output_path, output_name*"-aero.md"), "w") do fout

    println(fout, """
    # [Aerodynamic Solution](@id blownwingaero)

    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
            src="https://www.youtube.com/embed/3REcIdIXrZA?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```

    """)

    println(fout, "```julia")

    open(joinpath(example_path, "blownwing.jl"), "r") do fin

        ignore = false

        for l in eachline(fin)
            if contains(l, "6) POSTPROCESSING")
                break
            end

            if l=="#=" || contains(l, "# Uncomment this")
                ignore=true
            end

            if !ignore && !contains(l, "save_code=@__FILE__")
                println(fout, l)
            end

            if l=="=#" || contains(l, "# paraview      = false")
                ignore=false
            end
        end
    end

    println(fout, "```")

    println(fout, """
    ```@raw html
    <span style="font-size: 0.9em; color:gray;"><i>
        Run time: ~60 minutes on a 16-core AMD EPYC 7302 processor. <br>
    </i></span>
    <br><br>
    ```

    ```@raw html
    <center>
        <img src="$(remote_url)/blownwing-example-rotors_convergence.png"
                                            alt="Pic here" style="width: 100%;"/>
        <br>
        <br>
        <img src="$(remote_url)/blownwing-example-wing_convergence.png"
                                            alt="Pic here" style="width: 100%;"/>
    </center>
    <br>
    ```

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```

    ```@raw html
    <center>
        <img src="$(remote_url)/blownwing-example-animation.gif"
                                            alt="Vid here" style="width: 100%;"/>
    </center>
    <br>
    ```

    !!! info "Unsteady Loading Animation"
        Check the full
        example under
        [examples/blownwing/](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/blownwing)
        to see how to postprocess the simulation and generate this animation.
    """)

end





# -------- Aeroacoustics -------------------------------------------------------
open(joinpath(output_path, output_name*"-acoustics.md"), "w") do fout

    println(fout, """
    # Acoustic Solution

    """)

    # println(fout, "```julia")
    #
    # open(joinpath(example_path, "rotorhover_aeroacoustics.jl"), "r") do fin
    #     for (li, l) in enumerate(eachline(fin))
    #
    #         if contains(l, "# Save figures")
    #             break
    #         end
    #
    #         if li > 5
    #
    #             if contains(l, "#=")
    #                 println(fout, "```")
    #
    #             elseif contains(l, "=#")
    #                 println(fout, "```julia")
    #
    #             else
    #
    #                 println(fout, l)
    #
    #             end
    #         end
    #
    #     end
    # end
    #
    # println(fout, "```")

end



# -------- Aeroacoustics -------------------------------------------------------
open(joinpath(output_path, output_name*"-asm.md"), "w") do fout

    println(fout, """
    # Actuator Surface Model

    The aerodynamic solution computed in [the first section](@ref blownwingaero)
    was intended to be a mid-low fidelity simulation, which modeled the wing
    using an actuator line model (ALM).
    The ALM places all the surface vorticity associated with lift at
    the quarter chord of the wing, while placing all the trailing bound
    vorticity at the three-quarter chord, as shown here:

    ```@raw html
    <center>
        <img src="$(remote_url)/alm00.png" alt="Pic here" style="width: 75%;"/>
    </center>
    <br>
    ```

    The ALM is very accurate for isolated wings and even cases with mild wake
    interactions.
    However, for cases with stronger wake interactions (e.g., a wake directly
    impinging on the wing surface), we have developed an actuator surface model
    (ASM) that introduces the surface vorticity into the LES domain that better
    represents the physics.
    This is done by spreading the surface vorticity following a pressure-like
    distribution, which ends up producing a velocity field at the wing surface
    that minimizes the flow that crosses the airfoil centerline, thus better
    representing a solid surface:

    ```@raw html
    <center>
        <img src="$(remote_url)/asm00.png" alt="Pic here" style="width: 100%;"/>
        <img src="$(remote_url)/asm01.png" alt="Pic here" style="width: 100%;"/>
    </center>
    <br>
    ```
    For an in-depth discussion of the actuator line and surface models
    implemented in FLOWUnsteady, see Chapter 6 in
    [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589).[^2]


    [^2]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and
        Meshless Large Eddy Simulation of Multirotor Aircraft," *Doctoral
        Dissertation, Brigham Young University*.
        [**[VIDEO]**](https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html)
        [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)

    In order to activate the actuator surface model, we define the following
    parameters:

    ```julia
    thickness_w     = 0.15                      # Wing airfoil thickness t/c

    # ---------- Vortex sheet parameters ---------------
    vlm_vortexsheet = true                      # Whether to spread the wing circulation as a vortex sheet
    vlm_vortexsheet_overlap = 2.125             # Overlap of the particles that make the vortex sheet
    vlm_vortexsheet_distribution = uns.g_pressure   # Distribution of the vortex sheet

    sigma_vlm_surf  = b/100         # Smoothing radius of lifting bound vorticity
    vlm_vortexsheet_sigma_tbv = thickness_w*(b/ar) / 100 # Smoothing radius of trailing bound vorticity

    vlm_vortexsheet_maxstaticparticle = 1000000 # How many particles to preallocate for the vortex sheet


    # ---------- Force calculation parameters ----------
    KJforce_type                = "regular"     # KJ force evaluated at middle of bound vortices
    # KJforce_type              = "averaged"  # KJ force evaluated at average vortex sheet
    # KJforce_type              = "weighted"  # KJ force evaluated at strength-weighted vortex sheet

    include_trailingboundvortex = false         # Include trailing bound vortices in force calculations

    include_freevortices        = false         # Include free vortices in force calculation
    include_freevortices_TBVs   = false         # Include trailing bound vortex in free-vortex force

    include_unsteadyforce       = true          # Include unsteady force
    add_unsteadyforce           = false         # Whether to add the unsteady force to Ftot or to simply output it

    include_parasiticdrag       = true          # Include parasitic-drag force
    add_skinfriction            = true          # If false, the parasitic drag is purely parasitic, meaning no skin friction
    calc_cd_from_cl             = false         # Whether to calculate cd from cl or effective AOA
    wing_polar_file             = "xf-rae101-il-1000000.csv"    # Airfoil polar for parasitic drag
    ```

    Then we use a custom-defined function for calculating aerodynamic forces
    that uses the vortex sheet:

    ```julia
    # ---------- Aerodynamic forces --------------

    forces = []

    # Calculate Kutta-Joukowski force
    kuttajoukowski = uns.generate_calc_aerodynamicforce_kuttajoukowski(KJforce_type,
                                    sigma_vlm_surf, sigma_rotor_surf,
                                    vlm_vortexsheet, vlm_vortexsheet_overlap,
                                    vlm_vortexsheet_distribution,
                                    vlm_vortexsheet_sigma_tbv;
                                    vehicle=vehicle)
    push!(forces, kuttajoukowski)

    # Free-vortex force
    if include_freevortices
        freevortices = uns.generate_calc_aerodynamicforce_freevortices(
                                                vlm_vortexsheet_maxstaticparticle,
                                                sigma_vlm_surf,
                                                vlm_vortexsheet,
                                                vlm_vortexsheet_overlap,
                                                vlm_vortexsheet_distribution,
                                                vlm_vortexsheet_sigma_tbv;
                                                Ffv=uns.Ffv_direct,
                                                include_TBVs=include_freevortices_TBVs
                                                )
        push!(forces, freevortices)
    end

    # Force due to unsteady circulation
    if include_unsteadyforce
        unsteady(args...; optargs...) = uns.calc_aerodynamicforce_unsteady(args...; add_to_Ftot=add_unsteadyforce, optargs...)

        push!(forces, unsteady)
    end

    # Parasatic-drag force (form drag and skin friction)
    if include_parasiticdrag
        parasiticdrag = uns.generate_aerodynamicforce_parasiticdrag(wing_polar_file;
                                                                read_path=joinpath(data_path, "airfoils"),
                                                                calc_cd_from_cl=calc_cd_from_cl,
                                                                add_skinfriction=add_skinfriction,
                                                                Mach=speedofsound!=nothing ? magVref/speedofsound : nothing)
        push!(forces, parasiticdrag)
    end


    # Stitch all the forces into one function
    function calc_aerodynamicforce_fun(vlm_system, args...; per_unit_span=false, optargs...)

        # Delete any previous force field
        fieldname = per_unit_span ? "ftot" : "Ftot"
        if fieldname in keys(vlm_system.sol)
            pop!(vlm_system.sol, fieldname)
        end

        Ftot = nothing

        for (fi, force) in enumerate(forces)
            Ftot = force(vlm_system, args...; per_unit_span=per_unit_span, optargs...)
        end

        return Ftot
    end
    ```

    This custom-defined force needs to be passed to
    [`uns.generate_monitor_wing`](@ref):
    ```julia
    monitor_wing = uns.generate_monitor_wing(   ...
                                                include_trailingboundvortex=include_trailingboundvortex,
                                                calc_aerodynamicforce_fun=calc_aerodynamicforce_fun,
                                                ...
                                                )
    ```

    Finally, pass the following keyword arguments to
    [`uns.run_simulation`](@ref):

    ```julia
    # ------------- 5) RUN SIMULATION ------------------------------------------
    uns.run_simulation( ...
                        vlm_vortexsheet=vlm_vortexsheet,
                        vlm_vortexsheet_overlap=vlm_vortexsheet_overlap,
                        vlm_vortexsheet_distribution=vlm_vortexsheet_distribution,
                        vlm_vortexsheet_sigma_tbv=vlm_vortexsheet_sigma_tbv,
                        max_static_particles=vlm_vortexsheet_maxstaticparticle
                        ...
                        )
    ```

    !!! info "ASM and High Fidelity"
        ASM uses a very high density of particles at the wing
        surface (~100k particles per wing) to accuratelly introduce the solid
        boundary into the LES.
        This increases the computational cost of the simulation considerably.
        Hence, we recommend using ASM only for high-fidelity simulations.
    """)

end
