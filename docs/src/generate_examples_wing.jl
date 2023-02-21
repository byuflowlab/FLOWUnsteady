# ------------- SWEPT WING EXAMPLE ---------------------------------------------

output_name = "wing"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "wing")

remote_url = "http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"


# -------- 4.2deg AOA ----------------------------------------------------------
open(joinpath(output_path, output_name*"-4p2aoa.md"), "w") do fout

    println(fout, """
    # Simple Wing

    ```@raw html
    <center>
      <img src="$(remote_url)/weber-particles06.png" alt="Pic here" style="width: 49%;"/>
      <img src="$(remote_url)/weber-n100-00.png" alt="Pic here" style="width: 49%;"/>
    </center>
    ```

    In this example we solve the flow around a \$45^\\circ\$ swept-back wing at
    an angle of attack of \$4.2^\\circ\$.
    In the process we exemplify the basic structure of a simulation, which is
    always the same, no matter how complex the simulation might be.
    The structure consists of six steps:

    >**[1) Vehicle Definition](@ref):** Generate the geometry of the vehicle \
            and declare each vehicle subsystem in a \
            [`FLOWUnsteady.VLMVehicle`](@ref) object

    > **[2) Maneuver Definition](@ref):** Generate functions that prescribe \
            the kinematics of the vehicle and specify the control inputs for \
            tilting and rotor subsystems in a \
            [`FLOWUnsteady.KinematicManeuver`](@ref) object

    > **[3) Simulation Definition](@ref):** A \
            [`FLOWUnsteady.Simulation`](@ref) object is generated stating the \
            vehicle, maneuver, and total time and speed at which to perform \
            the maneuver

    >**[4) Monitors Definitions](@ref):** Functions are generated for \
            calculating, monitoring, and outputting different metrics \
            throughout the simulation

    > **[5) Run Simulation](@ref):** Call \
            [`FLOWUnsteady.run_simulation`](@ref) to run the simulation

    > **[6) Visualization and Postprocessing](@ref):** The simulation is visualized in \
            Paraview and results are postprocessed

    While in this example we show the basic structure without much explanation,
    in subsequent examples we will dive into the details and options of each
    step (which are also listed in the API Library).
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

    As the simulation runs, you will see the monitor shown below with the
    lift and drag coefficients over time along with the loading distribution.
    For comparison, here we have also added the experimental measurements
    reported in the literature.

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```

    ```@raw html
    <center>
        <img src="$(remote_url)/wingexample-simmonitor.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```
    """)

    open(joinpath(data_path, "wingexample-CLCD.md"), "r") do fin
        for l in eachline(fin)
            println(fout, l)
        end
    end

end




# -------- AOA Sweep -----------------------------------------------------------
open(joinpath(output_path, output_name*"-aoasweep.md"), "w") do fout

    println(fout, "# AOA Sweep")

    println(fout, """
        \nUsing the vehicle, maneuver, and simulation defined in the previous
        section, we now run a sweep of the angle of attack.
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
    (see the complete example under
    [examples/wing_aoasweep.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/wing/wing_aoasweep.jl)
    to see how to plot the results as shown here below)

    ```@raw html
    <center>
        <br><b>Spanwise loading distribution</b>
        <img src="$(remote_url)/wingexample-sweep-loading.png" alt="Pic here" style="width: 100%;"/>

        <br><br><b>Lift and induced drag</b>
        <img src="$(remote_url)/wingexample-sweep-CLCD.png" alt="Pic here" style="width: 100%;"/>

        <br><br><b>Pitching moment</b><br>
        <img src="$(remote_url)/wingexample-sweep-Cm.png" alt="Pic here" style="width: 50%;"/>
    </center>
    ```
    """)
end
