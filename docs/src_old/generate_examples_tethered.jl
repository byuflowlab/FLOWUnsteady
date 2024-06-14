# ------------- TETHERED WING EXAMPLE ------------------------------------------

output_name = "tetheredwing"
data_path = joinpath(module_path, "..", "resources", "data")
# example_path = joinpath(uns.examples_path, "tetheredwing")
example_path = uns.examples_path

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"


open(joinpath(output_path, output_name*".md"), "w") do fout

    println(fout, """
    # Tethered Wing

    ```@raw html
    <center>
      <img src="$(remote_url)/tetheredwing-example-00small.gif" alt="Vid here" style="width: 80%;"/>
    </center>
    ```

    ```@raw html
    <br>
    ```

    In this example we extend the [simple wing case](@ref simple_wing) to fly
    a circular path.
    This simulates a kite tethered to the ground with a crosswind that causes it
    to fly in circles.
    The kite effectively acts as a turbine extracting energy from the wind to sustain
    flight, which is the basis for the field of
    [airborne wind energy](https://en.wikipedia.org/wiki/Airborne_wind_turbine).

    This simulation exemplify the following features:

    * Translating and re-orienting the initial position of the vehicle using
        [`FLOWVLM.setcoordsystem`](@ref)
    * Defining a [`uns.KinematicManeuver`](@ref) object that prescribes the
        kinematics of a circular path
    * Customizing the wing monitor ([`uns.generate_monitor_wing`](@ref)) to
        compute and plot forces decomposed in arbitrary directions (in this
        case, following the orientation of the vehicle along the circular path)
    * Monitoring the vehicle state variables with
        [`uns.generate_monitor_statevariables`](@ref)

    ```@raw html
    <br>
    ```
    """)

    println(fout, "```julia")

    open(joinpath(example_path, "tetheredwing.jl"), "r") do fin
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
        Run time: ~20 minutes on a Dell Precision 7760 laptop.
        <br>
        Reduce resolution (n and steps) to speed up simulation without loss of accuracy.
    </i></span>
    <br><br>
    ```

    As the simulation runs, you will see the monitor shown below plotting the
    state variables of the vehicle. The components of both velocity
    and position follow a sinusoidal function, which is consistent with the
    circular path, while the angular velocity is constant.


    ```@raw html
    <center>
        <img src="$(remote_url)/tetheredwing-example-statemonitor.png" alt="Pic here" style="width: 100%;"/>
    </center>
    <br>
    ```

    The force monitor (shown below) plots the
    force components that are normal and tangential to the plane of rotation.
    Notice that the tangential force is negative, which is equivalent to
    having a negative drag, meaning that the vehicle is actually being propelled
    by the crosswind.
    Also notice that the forces are perturbed every time it completes a
    revolution.
    This is due to encountering the starting point of the wake that is slowly
    traveling downstream.
    It takes a couple revolutions for the forces to converge once the wake
    has been fully deployed.

    ```@raw html
    (<span style="color:red;">red</span> = beginning,
    <span style="color:blue;">blue</span> = end)
    ```

    ```@raw html
    <center>
        <img src="$(remote_url)/tetheredwing-example-wingmonitor.png" alt="Pic here" style="width: 100%;"/>
    </center>
    ```

    !!! info "Disable monitors to speed up simulation"
        For unknown reasons, the simulation oftentimes halts in between time
        steps while plotting the monitors.
        This slows down the overall simulation time, sometimes taking up to 2x
        longer.
        If you are not particularly interested in the plots of the monitors,
        you can disable the plots passing the keyword `disp_plot=false` to each
        monitor generator.


    !!! info "ParaView visualization"
        The `.pvsm` file visualizing the simulation as shown at the
        top of this page is available here:
        [LINK](https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/tetheredwing00.pvsm)
        (`right click → save as...`).

        To open in ParaView: `File → Load State → (select .pvsm file)` then
        select "Search files under specified directory" and point it to the
        folder where the simulation was saved.
    """)

end
