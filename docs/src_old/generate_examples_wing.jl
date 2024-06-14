# ------------- SWEPT WING EXAMPLE ---------------------------------------------

output_name = "wing"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "wing")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"


# -------- 4.2deg AOA ----------------------------------------------------------
open(joinpath(output_path, output_name*"-4p2aoa.md"), "w") do fout

    println(fout, """
    # [Simple Wing](@id simple_wing)

    ```@raw html
    <center>
      <img src="$(remote_url)/weber-particles06.png" alt="Pic here" style="width: 49%;"/>
      <img src="$(remote_url)/weber-n100-00.png" alt="Pic here" style="width: 49%;"/>
    </center>
    ```

    ```@raw html
    <br>
    ```

    In this example we simulate a \$45^\\circ\$ swept-back wing at
    an angle of attack of \$4.2^\\circ\$.
    In the process we exemplify the basic structure of simulations, which is
    always the same, no matter how complex the simulation might be.
    The structure consists of six steps:

    >**[(1) Vehicle Definition](@ref vehicle_types):** Generate the geometry of the vehicle \
            and declare each vehicle subsystem in a \
            [`FLOWUnsteady.VLMVehicle`](@ref) object

    > **[(2) Maneuver Definition](@ref):** Generate functions that prescribe \
            the kinematics of the vehicle and specify the control inputs for \
            tilting and rotor subsystems in a \
            [`FLOWUnsteady.KinematicManeuver`](@ref) object

    > **[(3) Simulation Definition](@ref):** A \
            [`FLOWUnsteady.Simulation`](@ref) object is generated stating the \
            vehicle, maneuver, and total time and speed at which to perform \
            the maneuver

    >**[(4) Monitors Definitions](@ref):** Functions are generated for \
            calculating, monitoring, and outputting different metrics \
            throughout the simulation

    > **[(5) Run Simulation](@ref):** Call to \
            [`FLOWUnsteady.run_simulation`](@ref)

    > **[(6) Viz and Postprocessing](@ref fluid_domain):** The simulation is visualized in \
            [Paraview](https://www.paraview.org/) and results are postprocessed

    ```@raw html
    <br>
    ```

    While in this example we show the basic structure without much explanation,
    in subsequent examples we will dive into the details and options of each
    step (which are also listed in the [API Guide](@ref vehicle_types)).

    ```@raw html
    <br>
    ```

    """)

    println(fout, "```julia")

    open(joinpath(example_path, "wing.jl"), "r") do fin
        for l in eachline(fin)
            if contains(l, "COMPARISON TO EXPERIMENTAL DATA")
                break
            end

            println(fout, l)
        end
    end

    println(fout, "```")

    println(fout, """

    ```@raw html
    <span style="font-size: 0.9em; color:gray;"><i>
        Run time: ~3 minutes on a Dell Precision 7760 laptop.
        <br>
        Reduce resolution (n and steps) to speed up simulation without loss of accuracy.
    </i></span>
    <br><br>
    ```

    As the simulation runs, you will see the monitor (shown below) plotting the
    lift and drag coefficients over time along with the loading distribution.
    For comparison, here we have also added the experimental measurements
    reported by
    [Weber and Brebner, 1951](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=001efd2bf915ad8282a0c5df5e9335624cbde811).

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```

    ```@raw html
    <center>
        <img src="$(remote_url)/wing-example-simmonitor.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```
    """)

    open(joinpath(data_path, "wing-example-CLCD.md"), "r") do fin
        for l in eachline(fin)
            println(fout, l)
        end
    end


    println(fout, """
    !!! info "Paraview visualization"
        The `.pvsm` file visualizing the simulation as shown at the
        top of this page is available here:
        [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/weber_particle02.pvsm)
        (`right click → save as...`).
        To open in Paraview: `File → Load State → (select .pvsm file)` then
        select "Search files under specified directory" and point it to the
        folder where the simulation was saved.
    """)

end




# -------- AOA Sweep -----------------------------------------------------------
open(joinpath(output_path, output_name*"-aoasweep.md"), "w") do fout

    println(fout, "# AOA Sweep")

    println(fout, """
        \nUsing the same vehicle, maneuver, and simulation defined in the
        previous section, we now run a sweep of the angle of attack.
    """)

    println(fout, "```julia")

    input_name = "wing_aoasweep.jl"

    open(joinpath(example_path, input_name), "r") do fin
        for (li, l) in enumerate(eachline(fin))
            if contains(l, "COMPARISON TO EXPERIMENTAL DATA")
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
        <br>
        Reduce resolution (n and steps) to speed up simulation without loss of accuracy.
    </i></span>
    <br><br>
    ```

    Check [examples/wing/wing_aoasweep.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/wing/wing_aoasweep.jl)
    to see how to postprocess and plot the results as shown below.

    ```@raw html
    <center>
        <br><b>Spanwise loading distribution</b>
        <img src="$(remote_url)/wing-example-sweep-loading.png" alt="Pic here" style="width: 100%;"/>

        <br><br><b>Vehicle lift and drag</b>
        <img src="$(remote_url)/wing-example-sweep-CLCD.png" alt="Pic here" style="width: 100%;"/>

        <br><br><b>Pitching moment</b><br>
        <img src="$(remote_url)/wing-example-sweep-Cm.png" alt="Pic here" style="width: 50%;"/>
    </center>
    ```
    """)
end
