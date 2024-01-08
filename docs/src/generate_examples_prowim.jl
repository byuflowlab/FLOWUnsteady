# ------------- BLOWN WING EXAMPLE ---------------------------------------------
output_name = "prowim"
example_path = joinpath(uns.examples_path, "prowim")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# -------- Aero Solution --------------------------------------------------------
open(joinpath(output_path, output_name*"-aero.md"), "w") do fout

    println(fout, """
    # [Rotor-on-Wing Interactions](@id prowimaero)

    In this example we use the [actuator surface model](@ref asm) (ASM) to
    accurately predict the effects of props blowing on a wing.
    This case simulates the PROWIM experiment in
    [Leo Veldhuis' dissertation](https://repository.tudelft.nl/islandora/object/uuid%3A8ffbde9c-b483-40de-90e0-97095202fbe3)
    (2005), and reproduces the validation study published in
    [Alvarez & Ning (2023)](https://arc.aiaa.org/doi/10.2514/1.C037279).

    In this example you can vary the fidelity of the simulation setting the
    following parameters:

    | Parameter | Mid-low fidelity | Mid-high fidelity | High fidelity | Description |
    | :-------: | :--------------: | :---------------: | :-----------: | :---------- |
    | `n_wing` | `50` | `50` | `100` | Number of wing elements per semispan |
    | `n_rotor` | `12` | `20` | `50` | Number of blade elements per blade |
    | `nsteps_per_rev` | `36` | `36` | `72` | Time steps per revolution |
    | `p_per_step` | `2` | `5` | `5` | Particle sheds per time step |
    | `shed_starting` | `false` | `false` | `true` | Whether to shed starting vortex |
    | `shed_unsteady` | `false` | `false` | `true` | Whether to shed vorticity from unsteady loading |
    | `treat_wake` | `true` | `true` | `false` | Treat wake to avoid instabilities |
    | `vlm_vortexsheet_overlap` | `2.125/10` | `2.125/10` | `2.125` | Particle overlap in ASM vortex sheet |
    | `vpm_integration` | `vpm.euler` | RK3``^\\star`` | RK3``^\\star`` | VPM time integration scheme |
    | `vpm_SFS` | None``^\\dag`` | Dynamic``^\\ddag`` | Dynamic``^\\ddag`` | VPM LES subfilter-scale model |

    * ``^\\star``*RK3:* `vpm_integration = vpm.rungekutta3`
    * ``^\\dag``*None:* `vpm_SFS = vpm.SFS_none`
    * ``^\\ddag``*Dynamic:* `vpm_SFS = vpm.SFS_Cd_twolevel_nobackscatter`

    (Mid-low fidelity settings may be inadequate for capturing rotor-on-wing interactions, unless using `p_per_step=5`)


    """)

    println(fout, "```julia")

    open(joinpath(example_path, "prowim.jl"), "r") do fin

        ignore = false

        for l in eachline(fin)
            if contains(l, "6) POSTPROCESSING")
                break
            end

            # if l=="#=" || contains(l, "# Uncomment this")
            if l=="#="
                ignore=true
            end

            if !ignore && !contains(l, "save_code=")
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
        Mid-low fidelity run time: 13 minutes a Dell Precision 7760 laptop. <br>
        Mid-high fidelity run time: 70 minutes a Dell Precision 7760 laptop. <br>
        High fidelity runtime: ~2 days on a 16-core AMD EPYC 7302 processor.
    </i></span>
    <br><br>
    ```

    ```@raw html
    <center>
        <br><br>
        <b>Mid-High Fidelity</b>
        <br>
        <img src="$(remote_url)/prowimblown-compexp-midfi-composed.png" alt="Pic here" style="width: 100%;"/>
        <br><br><br><br>
        <b>High Fidelity</b>
        <br>
        <img src="$(remote_url)/prowimblown-compexp-hifi-composed.png" alt="Pic here" style="width: 100%;"/>
        <br><br><br>
    </center>
    ```

    The favorable comparison with the experiment at \$\\alpha=0^\\circ\$ and
    \$4^\\circ\$ confirms that ASM accurately predicts propeller-wing
    interactions up to a moderate angle of attack. At \$\\alpha=10^\\circ\$ we suspect
    that the wing is mildly stalled, leading to a larger discrepancy (further
    discussed in [Alvarez' Dissertation](https://scholarsarchive.byu.edu/etd/9589)[^1]
    and
    [Alvarez & Ning, 2023](https://arc.aiaa.org/doi/abs/10.2514/1.C037279)[^2]).

    !!! info "Source file"
        Full example available under
        [examples/prowim/](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/prowim).



    [^1]: E. J. Alvarez (2022), "Reformulated Vortex Particle Method and
        Meshless Large Eddy Simulation of Multirotor Aircraft," *Doctoral
        Dissertation, Brigham Young University*.
        [**[VIDEO]**](https://www.nas.nasa.gov/pubs/ams/2022/08-09-22.html)
        [**[PDF]**](https://scholarsarchive.byu.edu/etd/9589/)
    [^2]: E. J. Alvarez and A. Ning (2023), "Meshless Large-Eddy Simulation of
        Propeller–Wing Interactions with Reformulated Vortex Particle Method,"
        *Journal of Aircraft*.
        [**[DOI]**](https://arc.aiaa.org/doi/abs/10.2514/1.C037279)[**[PDF]**](https://scholarsarchive.byu.edu/facpub/6902/)
    """)

end
