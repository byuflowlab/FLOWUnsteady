# ------------- ROTOR EXAMPLE --------------------------------------------------

output_name = "rotorhover"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "rotorhover")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# -------- Low Fidelity --------------------------------------------------------
open(joinpath(output_path, output_name*"-aero.md"), "w") do fout

    println(fout, """
    # Rotor in Hover

    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:71%;"
            src="https://www.youtube.com/embed/u9SgYbYhPpU?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```

    While propeller simulations are numerically well behaved, a hover case
    can pose multiple numerical challenges.
    This is because, in the absence of a freestream velocity, the wake forms
    and evolves purely due to its own self induced velocity, and it becomes
    crucial to have a well resolved simulation.

    In this example we simulate a DJI rotor in hover, and we use this case to
    demonstrate the artillery of FLOWUnsteady that makes it robust and
    numerically stable, among other things:

    * Subfilter scale (SFS) model of turbulence related to vortex stretching
    * Defining a wake treatment procedure to suppress hub wake at begining of
        simulation to avoid fountain effects (due to impulsive start) and
        accelerate convergence
    * Defining hub and tip loss corrections
    * Viscous diffusion through core spreading
    * Monitor of global flow enstrophy to track numerical stability
    * Monitor of dynamic SFS model coefficient

    ```@raw html
    <br>
    ```
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "rotorhover.jl"), "r") do fin
        for l in eachline(fin)
            if contains(l, "6) POSTPROCESSING")
                break
            end

            println(fout, l)
        end
    end

    println(fout, "```")

    println(fout, """
    ```@raw html
    <span style="font-size: 0.9em; color:gray;"><i>
        Run time: ~2 minutes on a Dell Precision 7760 laptop.
    </i></span>
    <br><br>
    ```


    !!! info "Hub/Tip Loss Correction"
        In the rotor actuator line model, a hub and tip corrections can be
        applied to ``c_\\ell`` to account for the effects that bring the
        aerodynamic loading at hub and tip to zero.
        These correction factors, ``F_\\mathrm{tip}`` and ``F_\\mathrm{hub}``,
        are defined as modified Prandtl loss functions,
        ```math
        \\begin{align*}
            F_\\mathrm{tip}
        & =
            \\frac{2}{\\pi} \\cos^{-1} \\left( \\exp\\left( -f_\\mathrm{tip} \\right) \\right)
        , \\qquad
            f_\\mathrm{tip}
        =
            \\frac{B}{2}
            \\frac{
                \\left[  \\left( \\frac{R_\\mathrm{rotor}}{r} \\right)^{t_1} - 1  \\right]^{t_2}
            }{
                \\vert \\sin \\left( \\theta_\\mathrm{eff} \\right) \\vert^{t_3}
            }
        \\\\
            F_\\mathrm{hub}
        & =
            \\frac{2}{\\pi} \\cos^{-1} \\left( \\exp\\left( -f_\\mathrm{hub} \\right) \\right)
        , \\qquad
            f_\\mathrm{hub}
        =
            \\frac{B}{2}
            \\frac{
                \\left[  \\left( \\frac{r}{R_\\mathrm{hub}} \\right)^{h_1} - 1  \\right]^{h_2}
            }{
                \\vert \\sin \\left( \\theta_\\mathrm{eff} \\right) \\vert^{h_3}
            }
        ,\\end{align*}
        ```
        where ``R_\\mathrm{rotor}`` and ``R_\\mathrm{hub}`` are the rotor and hub radii, ``B``
        is the number of blades, ``r`` is the radial position of the blade element, and
        ``t_1``, ``t_2``, ``t_3``, ``h_1``, ``h_2``, and ``h_3`` are tunable parameters.
        The normal and tangential force coefficients, respectively ``c_n`` and ``c_t``, are then calculated as
        ```math
        \\begin{align*}
            c_n
        & =
            F_\\mathrm{tip} F_\\mathrm{hub} c_\\ell\\cos\\theta_\\mathrm{eff} + c_d\\sin\\theta_\\mathrm{eff}
        \\\\
            c_t
        & =
            F_\\mathrm{tip} F_\\mathrm{hub} c_\\ell\\sin\\theta_\\mathrm{eff} - c_d\\cos\\theta_\\mathrm{eff}
        .\\end{align*}
        ```

        The hub and tip corrections are passed to [`uns.run_simulation`](@ref)
        through the keyword argument
        `hubtiploss_correction = ((t1, t2, t3, tminangle), (h1, h2, h3, hminangle))`,
        where `tminangle` and `hminangle` are clipping thresholds for the minimum
        allowable value of ``\\vert\\theta_\\mathrm{eff}\\vert`` (in degs) that
        is used in tip and hub corrections.
        The following corrections are predefined in FLOWVLM for the user:

    ```@example
    import FLOWVLM as vlm

    # No corrections
    vlm.hubtiploss_nocorrection
    ```

    ```@example
    import FLOWUnsteady: vlm            # hide

    # Original Prandtl corrections
    vlm.hubtiploss_correction_prandtl
    ```

    ```@example
    import FLOWUnsteady: vlm            # hide

    # Modified Prandtl with a strong hub correction
    vlm.hubtiploss_correction_modprandtl
    ```
    """)

end



# -------- Aeroacoustics -------------------------------------------------------
open(joinpath(output_path, output_name*"-acoustics.md"), "w") do fout

    println(fout, """
    # Aeroacoustic Noise

    """)

    # println(fout, "```julia")
    #
    # open(joinpath(example_path, "rotorhover.jl"), "r") do fin
    #     for l in eachline(fin)
    #         if contains(l, "6) POSTPROCESSING")
    #             break
    #         end
    #
    #         println(fout, l)
    #     end
    # end
    #
    # println(fout, "```")

    # println(fout, """
    # ```@raw html
    # <span style="font-size: 0.9em; color:gray;"><i>
    #     Run time: ~2 minutes on a Dell Precision 7760 laptop.
    # </i></span>
    # <br><br>
    # ```
    # """)

end


# -------- Fluid Domain --------------------------------------------------------
open(joinpath(output_path, output_name*"-fdom.md"), "w") do fout

    println(fout, """
    # Fluid Domain

    """)

    # println(fout, "```julia")
    #
    # open(joinpath(example_path, "rotorhover.jl"), "r") do fin
    #     for l in eachline(fin)
    #         if contains(l, "6) POSTPROCESSING")
    #             break
    #         end
    #
    #         println(fout, l)
    #     end
    # end
    #
    # println(fout, "```")

    # println(fout, """
    # ```@raw html
    # <span style="font-size: 0.9em; color:gray;"><i>
    #     Run time: ~2 minutes on a Dell Precision 7760 laptop.
    # </i></span>
    # <br><br>
    # ```
    # """)

end
