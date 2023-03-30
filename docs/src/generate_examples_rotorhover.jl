# ------------- ROTOR EXAMPLE --------------------------------------------------

output_name = "rotorhover"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "rotorhover")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# -------- Low Fidelity --------------------------------------------------------
open(joinpath(output_path, output_name*"-aero.md"), "w") do fout

    println(fout, """
    # Variable Fidelity

    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
            src="https://www.youtube.com/embed/u9SgYbYhPpU?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```

    While propeller simulations tend to be numerically well behaved, hover cases
    can pose multiple numerical challenges.
    The rotation of blades in static air drives a strong axial flow that is
    solely caused by the shedding of tip vortices.
    This is challenging to simulate since, in the absence of a
    freestream, the wake quickly becomes fully turbulent and breaks down as tip
    vortices leapfrog and mix close to the rotor.
    Thus, a rotor in hover is a good engineering application to showcase the
    numerical stability and accuracy of FLOWUnsteady.

    In this example we simulate a DJI rotor in hover, and we use this case to
    demonstrate some of the advanced features of FLOWUnsteady that make it
    robust and accurate in resolving turbulent mixing:

    * [Subfilter scale (SFS) model](@ref sfsmodel) of turbulence related to vortex stretching
    * How to generate a monitor of dynamic SFS model coefficient
        [`uns.generate_monitor_Cd`](@ref)
    * How to generate a monitor of global flow enstrophy with
        [`uns.generate_monitor_enstrophy`](@ref) to track numerical stability
    * Defining a wake treatment procedure to suppress initial hub wake, avoiding
        hub fountain effects and accelerating convergence
    * Defining hub and tip loss corrections

    Also, in this example you can vary the fidelity of the simulation with the
    following parameters:

    | Parameter | Mid-low fidelity | Mid-high fidelity | High fidelity | Description |
    | :-------: | :--------------: | :---------------: | :-----------: | :---------- |
    | `n` | `20` | `50` | `50` | Number of blade elements per blade |
    | `nsteps_per_rev` | `36` | `72` | `360` | Time steps per revolution |
    | `p_per_step` | `4` | `2` | `2` | Particle sheds per time step |
    | `sigma_rotor_surf` | `R/10` | `R/10` | `R/80` | Rotor-on-VPM smoothing radius |
    | `sigmafactor_vpmonvlm` | `1.0` | `1.0` | `5.5` | Expand particles by this factor when calculating VPM-on-VLM/Rotor induced velocities |
    | `shed_starting` | `false` | `false` | `true` | Whether to shed starting vortex |
    | `suppress_fountain` | `true` | `true` | `false` | Whether to suppress hub fountain effect |
    | `vpm_integration` | `vpm.euler` | RK3``^\\star`` | RK3``^\\star`` | VPM time integration scheme |
    | `vpm_SFS` | None``^\\dag`` | None``^\\dag`` | Dynamic``^\\ddag`` | VPM LES subfilter-scale model |

    * ``^\\star``*RK3:* `vpm_integration = vpm.rungekutta3`
    * ``^\\dag``*None:* `vpm_SFS = vpm.SFS_none`
    * ``^\\ddag``*Dynamic:* `vpm_SFS = vpm.SFS_Cd_twolevel_nobackscatter`


    ```@raw html
    <br>

    <center><b>Spatial discretization</b></center>

    <table>
        <tr>
            <td>
                <img src="$(remote_url)/singlerotor-particlescomp-midlow-00.png" alt="Pic here" style="width:100%;"/>
            </td>
            <td>
                <img src="$(remote_url)/singlerotor-particlescomp-midhigh-00.png" alt="Pic here" style="width:100%;"/>
            </td>
            <td>
                <img src="$(remote_url)/singlerotor-particlescomp-high-03.png" alt="Pic here" style="width:100%;"/>
            </td>
        </tr>
            <tr>
                <th><center>Mid-Low</center></th>
                <th><center>Mid-High</center></th>
                <th><center>High</center></th>
            </tr>
    </table>
    ```

    ```@raw html
    <br>
    ```
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "rotorhover.jl"), "r") do fin

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
        Mid-low fidelity runtime: ~7 minutes on a 16-core AMD EPYC 7302 processor. <br>
        Mid-high fidelity runtime: ~60 minutes on a 16-core AMD EPYC 7302 processor. <br>
        High fidelity runtime: ~30 hours on a 16-core AMD EPYC 7302 processor.
    </i></span>
    <br><br>
    ```

    Rotor monitor in the high-fidelity case:
    ```@raw html
    <center>
        <img src="$(remote_url)/rotorhover-example-high02-singlerotor_convergence.png" alt="Pic here" style="width:100%;"/>
    </center>
    ```

    As the simulation runs, you will see the monitor shown below plotting the
    global enstrophy of the flow. The global enstrophy achieves a steady state
    once the rate of enstrophy produced by the rotor eventually balances out
    with the forward scatter of the SFS turbulence model, making the simulation
    indefinitely stable.

    ```@raw html
    <center>
        <img src="$(remote_url)/rotorhover-example-high02-singlerotorenstrophy.png" alt="Pic here" style="width:50%;"/>
    </center>
    ```

    The SFS model uses a [dynamic procedure](@ref sfsmodel) to compute its own
    model coefficient ``C_d`` as the simulation evolves. This model coefficient
    has a different value for each particle in space and time.
    The ``C_d``-monitor shown below plots the mean value from all the
    particle in the field that have a non-zero ``C_d`` (left), and also the ratio of the
    number of particles that got clipped to a zero ``C_d`` over the total number of
    particles (right).


    ```@raw html
    <center>
        <img src="$(remote_url)/rotorhover-example-high02-singlerotorChistory.png" alt="Pic here" style="width:100%;"/>
    </center>
    ```


    !!! info "Prescribing the Model Coefficient"
        The SFS model helps the simulation to more accurately capture
        the effects of turbulence from the scales that are not resolved,
        but it comes with a computational cost.
        The following table summarizes the cost of the rVPM, the SFS model,
        and the ``C_d`` dynamic procedure.
        ![pic]($(remote_url)/rvpmsfs-benchmark02.png)
        The dynamic procedure is the most costly operation, which increases the
        simulation runtime by about 35%.

        If you need to run a case multiple times with only slight changes
        (e.g., sweeping the AOA and/or RPM), you can first run the simulation
        with the dynamic procedure (`vpm_SFS = vpm.SFS_Cd_twolevel_nobackscatter`),
        take note of what the mean ``C_d`` shown in the monitor converges to,
        and then prescribe that value to subsequent simulations.
        Prescribing ``C_d`` ends up in a simulation that is only 8% slower than
        the classic VPM without any SFS model.

        ``C_d`` can then be prescribed as follows
        ```julia
        vpm_SFS = vpm.ConstantSFS(vpm.Estr_fmm; Cs=value, clippings=[vpm.clipping_backscatter])
        ```
        where `CS = value` is the value to prescribe for the model
        coefficient, and `clippings=[vpm.clipping_backscatter]` clips the
        backscatter of enstrophy (making it a purely diffusive model).
        As a reference, in this hover case, ``C_d`` converges to ``0.26`` in the
        high-fidelity simulation.


    In [examples/rotorhover/rotorhover_postprocess.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/rotorhover/rotorhover_postprocess.jl)
    we show how to postprocess the simulations to compare ``C_T`` and blade
    loading to experimental data by Zawodny & Boyd[^1] and a URANS simulation
    (STAR-CCM+) by Schenk[^2]:

    ```@raw html
    <center>
        <img src="$(remote_url)/dji9443-CTcomparison.png" alt="Pic here" style="width:75%;"/>
        <img src="$(remote_url)/dji9443-loadingcomparison.png" alt="Pic here" style="width:75%;"/>
    </center>
    ```

    |                           | ``C_T``   | Error |
    | ------------------------: | :-------: | :---: |
    | Experimental              | 0.072     | --    |
    | URANS                     | 0.071     | 1%    |
    | rVPM -- high fidelity     | 0.073     | 1%    |
    | rVPM -- mid-high fidelity | 0.066     | 8%    |
    | rVPM -- mid-low fidelity  | 0.064     | 11%   |
    | BEMT (quasi-steady)       | 0.073     | 2%    |

    [^1]: N. S. Zawodny, D. D. Boyd, Jr., and C. L. Burley, “Acoustic
        Characterization and Prediction of Representative, Small-scale
        Rotary-wing Unmanned Aircraft System Components,” in
        *72nd American Helicopter Society (AHS) Annual Forum* (2016).

    [^2]: A. R. Schenk, "Computational Investigation of the Effects of
        Rotor-on-Rotor Interactions on Thrust and Noise," Masters thesis,
        *Brigham Young University* (2020).


    !!! info "Hub/Tip Loss Correction"
        In the rotor actuator line model, hub and tip corrections can be
        applied to ``c_\\ell`` to account for the effects that bring the
        aerodynamic loading to zero at the hub and tips.
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
    !!! info "Paraview Visualization"
        The `.pvsm` file visualizing the simulation as shown at the
        top of this page is available here:
        [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/singlerotor-monitors-particles11.pvsm)
        (`right click → save as...`).
        
        To open in Paraview: `File → Load State → (select .pvsm file)` then
        select "Search files under specified directory" and point it to the
        folder where the simulation was saved.
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
