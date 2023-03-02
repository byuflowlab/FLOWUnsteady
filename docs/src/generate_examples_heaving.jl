# ------------- TETHERED WING EXAMPLE ------------------------------------------

output_name = "heavingwing"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "heavingwing")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"


open(joinpath(output_path, output_name*".md"), "w") do fout

    println(fout, """
    # Heaving Wing

    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:74%;"
            src="https://www.youtube.com/embed/Pch94bKpjrQ?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```

    In this example we extend the [simple wing case](@ref simple_wing) to create
    a case of interactional aerodynamics.
    We will place a straight wing flying in front of the original swept-back
    wing.
    The front wing will be moving in a heaving motion, shedding a wake that
    impinges on the back wing causing an unsteady loading.

    This simulation exemplify the following features:

    * Defining a [`uns.VLMVehicle`](@ref) with multiple surfaces, one of them \
    declared as a tilting surface
    * Defining the control inputs for a tilting system in \
    [`uns.KinematicManeuver`](@ref) (in this case, tilting the surface over \
    time in a heaving motion)

    ```@raw html
    <br>
    ```
    """)

    println(fout, "```julia")



    open(joinpath(example_path, "heavingwing.jl"), "r") do fin
        for l in eachline(fin)

            println(fout, l)

            if contains(l, "#       RPM are then defined in the simulation")
                println(fout, " ")
                break
            end
        end
    end

    println(fout, """
    ```

    At this point we can verify that we have correctly defined the control
    inputs of the maneuver calling [`FLOWUnsteady.plot_maneuver`](@ref) as follows:
    ```julia
    uns.plot_maneuver(maneuver)
    ```
    ```@raw html
    <center>
        <img src="$(remote_url)/heavingwing-example-maneuver-controls.png" alt="Pic here" style="width: 85%;"/>
    </center>
    <br>
    ```
    Here we confirm that the angle of the tilting surface along the \$y\$-axis of
    the vehicle (pitch) will change sinusoidally with amplitude \$5^\\circ\$, as
    expected.
    This function also plots the kinematics of the vehicle, which in this case
    are rather uneventful (straight line).

    Now we continue defining the simulation:

    ```julia \
    """)

    open(joinpath(example_path, "heavingwing.jl"), "r") do fin

        flag = false

        for l in eachline(fin)
            if flag==false && contains(l, "3) SIMULATION DEFINITION")
                flag = true
            end

            if contains(l, "6) POSTPROCESSING")
                break
            end

            if flag
                println(fout, l)
            end
        end
    end

    println(fout, "```")

    println(fout, """

    As the simulation runs, you will see the monitors (shown below) plotting the
    lift and drag coefficients over time along with the loading distribution.

    ```@raw html
    <center>
        <img src="$(remote_url)/heavingwing-example-frontwing_convergence.png"
                                            alt="Pic here" style="width: 100%;"/>
        <br>
        <br>
        <img src="$(remote_url)/heavingwing-example-backwing_convergence.png"
                                            alt="Pic here" style="width: 100%;"/>
    </center>
    <br>
    ```

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```

    In these monitors, we clearly see the fluctuation of \$C_L\$ and \$C_D\$
    over time due to the heaving motion (front wing) and the wake
    impingement (back wing).
    The plots of loading distribution seem very convoluted since the loading
    fluctuates over time, and all the time steps are super imposed in the
    monitor.

    To more clearly see what the loading distribution is doing, it is insightful
    to plot the loading as an animation as shown below (check the full
    example under
    [examples/heavingwing/](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/heavingwing)
    to see how to postprocess the simulation and generate this animation).

    ```@raw html
    <center>
        <img src="$(remote_url)/heavingwing-example-animation.gif"
                                            alt="Vid here" style="width: 100%;"/>
    </center>
    <br>
    ```

    """)

end
