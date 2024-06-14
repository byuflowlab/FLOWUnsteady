# ------------- PROPELLER EXAMPLE ----------------------------------------------

output_name = "propeller"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "propeller")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# -------- J=0.4 ---------------------------------------------------------------
open(joinpath(output_path, output_name*"-J040.md"), "w") do fout

    println(fout, """
    # Basics

    ```@raw html
    <center>
      <img src="$(remote_url)/apc10x7ccblongwake_J035_00cont03_img1_00.gif" alt="Vid here" style="width: 100%;"/>
    </center>
    ```

    ```@raw html
    <br>
    ```

    In this example we first simulate an
    [APC Thin-Electric 10x7 propeller](https://www.apcprop.com/product/10x7e/)
    operating in cruise conditions.
    Along the way, we demonstrate the basics of how to set up and run a rotor
    simulation.

    Rotors are generated through the function
    [`FLOWUnsteady.generate_rotor`](@ref), which can receive either
    a set of parameters that define the rotor geometry (like twist/chord/sweep
    distributions, etc), or it can read the rotor geometry from a file.
    FLOWunsteady provides a prepopulated database of airfoil and rotor geometries
    to automate the generation of rotors, which is found under
    [`database/`](https://github.com/byuflowlab/FLOWUnsteady/tree/master/database).
    This database can be accessed through the variable
    `FLOWUnsteady.default_database`.
    Alternatively, users can define their own database with custom rotors and
    airfoils.

    The following slides describe the structure of the database, using a DJI
    rotor as an example:

    ```@raw html
        <div style="position:relative;padding-top:50%;">
            <iframe style="position:absolute;left:0;top:0;height:100%;width:80%;" src="https://docs.google.com/presentation/d/e/2PACX-1vRsYbuuMFQdc05NRrQ3Db0RT4XKKoxEYDiUi0MpW58W6A-pp0sDHQI9mVqNFagPtQ/embed?start=true&loop=true&delayms=3000" frameborder="0" width="100%" height="100%" allowfullscreen="true" mozallowfullscreen="true" webkitallowfullscreen="true"></iframe>
        </div>
        <br><br>
    ```

    In this simulation we exemplify the following:

    * How to generate a rotor with [`uns.generate_rotor`](@ref)
    * How to generate a rotor monitor with [`uns.generate_monitor_rotors`](@ref)
    * How to set up and run a rotor simulation.

    ```@raw html
    <br>
    ```
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "propeller.jl"), "r") do fin
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

    As the simulation runs, you will see the monitor shown below plotting the
    blade loading along with thrust and torque coefficients and propulsive
    efficiency.

    ```@raw html
    <center>
        <img src="$(remote_url)/propeller-example-simmonitor.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```
    """)

end



# -------- J Sweep -------------------------------------------------------------
open(joinpath(output_path, output_name*"-jsweep.md"), "w") do fout

    println(fout, "# \$J\$ Sweep")

    println(fout, """
        \nUsing the same rotor from the previous section, we now run a sweep of
        the advance ratio \$J = \\frac{u_\\infty}{n d}\$ to characterize the
        performance of the propeller.
    """)

    println(fout, "```julia")

    input_name = "propeller_jsweep.jl"

    open(joinpath(example_path, input_name), "r") do fin
        for (li, l) in enumerate(eachline(fin))
            if contains(l, "POSTPROCESSING")
                break
            end

            println(fout, l)

        end
    end

    println(fout, "```")

    println(fout, """

    ```@raw html
    <span style="font-size: 0.9em; color:gray;"><i>
        Run time: ~12 minutes on a Dell Precision 7760 laptop.
    </i></span>
    <br><br>
    ```

    Check [examples/propeller/propeller_jsweep.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/propeller/propeller_jsweep.jl)
    to postprocess and plot the results as shown below.

    ```@raw html
    <center>
        <br>
        <img src="$(remote_url)/propeller-Jsweep-example.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```
    """)
end



# -------- Quasi-Steady Solver -------------------------------------------------
open(joinpath(output_path, output_name*"-quasisteady.md"), "w") do fout

    println(fout, "# Quasi-Steady Solver")

    println(fout, """
    \nWhile unsteady simulations are resolved using the reformulated VPM,
    FLOWUnsteady also provides a quasi-steady solver for low-fidelity
    simulations.
    The quasi-steady solver replaces the particle field with semi-infinite
    rigid wakes in wings and blade-element momentum theory in rotors.

    The quasi-steady solver is invoked by simply changing the line
    ```julia
    VehicleType     = uns.UVLMVehicle
    ```
    to
    ```julia
    VehicleType   = uns.QVLMVehicle
    ```
    in the previous sections.
    The results of the quasi-steady solver are shown below, predicted
    through blade-element momentum theory.

    ```@raw html
    <center>
        <img src="$(remote_url)/propeller-Jsweep-exampleBEMT.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```
    """)
end






# -------- Incidence Angle Sweep -------------------------------------------------
open(joinpath(output_path, output_name*"-incidence.md"), "w") do fout

    println(fout, "# Incidence Sweep")

    println(fout, """
    ```@raw html
    <center>
        <img src="$(remote_url)/prowim_isoprop_J100-AOA200-00_3.gif" alt="Vid here" style="width: 100%;"/>
    </center>
    ```

    In simple cases like a propeller in cruise, steady and quasi-steady methods
    like blade element momentum theory can be as accurate as a fully unsteady
    simulation, and even faster.
    However, in more complex cases, quasi-steady solvers are far from accurate
    and a fully unsteady solver is needed.
    We now highlight one of such cases: the case of a propeller at an incidence
    angle.

    A rotor operating at an incidence angle relative to the freestream
    experiences an unsteady loading due to the
    blade seeing a larger local velocity in the advancing side of the rotor and
    a smaller local velocity in the retreating side.
    This also causes a wake that is skewed.
    For this example we will run a sweep of simulations on a 4-bladed propeller
    operating at multiple incidence angles \$\\alpha\$ (where
    \$\\alpha=0^\\circ\$ is fully axial inflow, and \$\\alpha=90^\\circ\$
    is fully edgewise inflow).
    """)


    println(fout, "```julia")

    input_name = "propeller_incidence.jl"

    open(joinpath(example_path, input_name), "r") do fin
        for (li, l) in enumerate(eachline(fin))
            if contains(l, "POSTPROCESSING")
                break
            end

            println(fout, l)

        end
    end

    println(fout, "```")

    println(fout, """
    ```@raw html
    <span style="font-size: 0.9em; color:gray;"><i>
        Run time: ~15 minutes on a Dell Precision 7760 laptop.
    </i></span>
    <br><br>
    ```

    Check [examples/propeller/propeller_incidence.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/propeller/propeller_incidence.jl)
    to postprocess and plot the results as shown below.

    ```@raw html
    <center>
        <img src="$(remote_url)/propeller-incidencesweep-example.png" alt="Pic here" style="width: 50%;"/>
    </center>
    ```

    !!! info "Paraview visualization"
        The `.pvsm` file visualizing the simulation as shown at the
        top of this page is available here:
        [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/prowim-singlerotor-monitors01.pvsm)
        (`right click → save as...`).
        To open in Paraview: `File → Load State → (select .pvsm file)` then
        select "Search files under specified directory" and point it to the
        folder where the simulation was saved.
    """)
end
