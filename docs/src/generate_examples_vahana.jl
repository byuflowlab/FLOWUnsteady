# ------------- BLOWN WING EXAMPLE ---------------------------------------------
output_name = "vahana"
example_path = joinpath(uns.examples_path, "vahana")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"

# ------------------------------------------------------------------------------
open(joinpath(output_path, output_name*"-vehicle.md"), "w") do fout

    println(fout, """
    # [Vehicle Definition](@id vahanavehicle)

    In this example we simulate the eVTOL transition maneuver of a tandem
    tilt-wing multirotor aircraft.
    The aircraft configuration resembles the Vahana eVTOL aircraft
    but with tilt and stacked rotors:

    ```@raw html
    <table>
        <tr>
            <td>
                <img src="$(remote_url)/vahana2-vehicle00-cropped-small.png" alt="Pic here" style="width:90%;"/>
                <br>
                <center><b>Takeoff and landing</b></center>
            </td>
            <td>
                <img src="$(remote_url)/vahana2-vehicle02-cropped-small.png" alt="Pic here" style="width:90%;"/>
                <br>
                <center><b>Cruise</b></center>
            </td>
        </tr>
    </table>
    ```

    Due to the complexity of this simulation, each step of the simulation setup
    is quite involved.
    Hence, we have broken down each step into a function that can be call
    when we setup the simulation.

    Below is the function that defines the vehicle for the simulation
    ([`uns.UVLMVehicle`](@ref)).
    Along with defining the vehicle geometry, it also defines two tilting
    systems (a tilting system is a set of components that tilt together) and
    three rotor systems (a rotor system is a set of rotors with a common RPM).
    Later in the next section we will define the control inputs for these
    tilting and rotor systems.

    ```@raw html
    <table>
        <tr>
            <td>
                <center>
                    <img src="$(remote_url)/vahana-tiltingsystems00.png" alt="Pic here" style="width:70%;"/>
                </center>
            </td>
            <td>
                <center>
                    <img src="$(remote_url)/vahana-rotorsystems00.png" alt="Pic here" style="width:70%;"/>
                </center>
            </td>
        </tr>
    </table>
    ```

    """)

    println(fout, "```julia")

    open(joinpath(example_path, "vahana_vehicle.jl"), "r") do fin

        ignore = false

        for (li, l) in enumerate(eachline(fin))
            if li >= 6

                println(fout, l)

                if l=="end"
                    break
                end

            end
        end

    end

    println(fout, "```")

    println(fout, """
    !!! info "Full example"
        The function that defines the fuselage is given in the full example
        under
        [examples/vahana/vahana_vehicle.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/vahana/vahana_vehicle.jl)
    """)

end






# ------------------------------------------------------------------------------
open(joinpath(output_path, output_name*"-maneuver.md"), "w") do fout

    println(fout, """
    # [Maneuver Definition](@id vahanamaneuver)

    In this section we define a function that generates the eVTOL transition
    maneuver (a [`uns.KinematicManeuver`](@ref) object).
    This maneuver object contains the non-dimensional velocity and attitude of
    the aircraft over a non-dimensional time (where ``t=0`` is the beginning
    of the maneuver and ``t=1`` is the end), which prescribes the kinematics of
    the vehicle.
    It also contains the control inputs for the aircraft over time: tilting
    angles for each tilting system and RPM for each rotor system shown below.

    ```@raw html
    <table>
        <tr>
            <td>
                <img src="$(remote_url)/vahana-tiltingsystems00.png" alt="Pic here" style="width:90%;"/>
            </td>
            <td>
                <img src="$(remote_url)/vahana-rotorsystems00.png" alt="Pic here" style="width:90%;"/>
            </td>
        </tr>
    </table>
    ```

    The maneuver here defined contains five stages:
    1. Takeoff, climb, hover
    2. Hover ``\\rightarrow`` cruise transition (powered lift to wing-borne flight)
    3. Cruise
    4. Cruise ``\\rightarrow`` hover transition (wing-borne flight to powered lift)
    5. Hover, descend, landing

    Since the vehicle kinematics and control inputs used in the maneuver
    definition are non-dimensional, the maneuver can be performed as fast or as
    slow as desired when we define the total time, reference velocity, and
    reference RPM of [`uns.Simulation`](@ref) later in the next section, without
    needing to change the maneuver definition shown here.
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "vahana_maneuver.jl"), "r") do fin

        ignore = false

        for (li, l) in enumerate(eachline(fin))
            if li >= 6

                println(fout, l)

                if l=="end"
                    break
                end

            end
        end

    end

    println(fout, "```")

    println(fout, """
    ```@raw html
    <center>
        <br><br>
        <img src="$(remote_url)/vahana-kinematics.png" alt="Pic here" style="width:100%;"/>
        <br><br><br><br>
        <img src="$(remote_url)/vahana-controls.png" alt="Pic here" style="width:75%;"/>
    </center>
    ```
    Notice that this plot shows four rotor systems instead of only three.
    This is because the system of stacked rotors was split into two (all upper
    stack rotors are grouped together, while all lower stack rotors are
    also grouped together).
    This way we could change the index angle of the stacked rotors throughout
    the simulation by modulating upper and lower RPMs independently.
    However, for simplicity, in this example we have kept upper and lower
    RPMs the same.
    """)

end




# ------------------------------------------------------------------------------
open(joinpath(output_path, output_name*"-monitor.md"), "w") do fout

    println(fout, """
    # [Monitors Definitions](@id vahanamonitor)

    Here we define the monitors of the simulation and concatenate them
    all.
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "vahana_monitor.jl"), "r") do fin

        ignore = false

        for (li, l) in enumerate(eachline(fin))
            if li >= 6

                println(fout, l)

                if l=="end"
                    break
                end

            end
        end

    end

    println(fout, "```")

end





# ------------------------------------------------------------------------------
open(joinpath(output_path, output_name*"-run.md"), "w") do fout

    println(fout, """
    # [Run Simulation](@id vahanarun)

    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
            src="https://www.youtube.com/embed/d__wNtRIBY8?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```

    """)

    println(fout, "```julia")

    open(joinpath(example_path, "vahana.jl"), "r") do fin

        for (li, l) in enumerate(eachline(fin))

            if contains(l, "# Uncomment") || contains(l, "# save_path") || contains(l, "# run_name")
                nothing
            else
                println(fout, l)
            end

        end

    end

    println(fout, "```")

end
