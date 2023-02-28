# ------------- SWEPT WING EXAMPLE ---------------------------------------------

output_name = "tetheredwing"
data_path = joinpath(module_path, "..", "resources", "data")
example_path = joinpath(uns.examples_path, "tetheredwing")

remote_url = "https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/"


# -------- R = 3b --------------------------------------------------------------
open(joinpath(output_path, output_name*"-R3b.md"), "w") do fout

    println(fout, """
    # [Tethered Wing](@id tethered_wing)

    ```@raw html
    <center>
      <img src="$(remote_url)/weber-particles06.png" alt="Pic here" style="width: 49%;"/>
      <img src="$(remote_url)/weber-n100-00.png" alt="Pic here" style="width: 49%;"/>
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
            if contains(l, "COMPARISON TO EXPERIMENTAL DATA")
                break
            end

            println(fout, l)
        end
    end

    println(fout, "```")

    # println(fout, """
    #
    # As the simulation runs, you will see the monitor (shown below) plotting the
    # lift and drag coefficients over time along with the loading distribution.
    # For comparison, here we have also added the experimental measurements
    # reported by
    # [Weber and Brebner, 1951](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=001efd2bf915ad8282a0c5df5e9335624cbde811).
    #
    # ```@raw html
    # (<span style="color:red;">red</span> = beginning,
    # <span style="color:blue;">blue</span> = end)
    # ```
    #
    # ```@raw html
    # <center>
    #     <img src="$(remote_url)/wingexample-simmonitor.png" alt="Pic here" style="width: 100%;"/>
    # </center>
    # ```
    # """)
    #
    # open(joinpath(data_path, "wingexample-CLCD.md"), "r") do fin
    #     for l in eachline(fin)
    #         println(fout, l)
    #     end
    # end

end




# # -------- AOA Sweep -----------------------------------------------------------
# open(joinpath(output_path, output_name*"-aoasweep.md"), "w") do fout
#
#     println(fout, "# AOA Sweep")
#
#     println(fout, """
#         \nUsing the same vehicle, maneuver, and simulation defined in the
#         previous section, we now run a sweep of the angle of attack.
#     """)
#
#     println(fout, "```julia")
#
#     input_name = "wing_aoasweep.jl"
#
#     open(joinpath(example_path, input_name), "r") do fin
#         for (li, l) in enumerate(eachline(fin))
#             if contains(l, "COMPARISON TO EXPERIMENTAL DATA")
#                 break
#             end
#
#             println(fout, l)
#
#         end
#     end
#
#     println(fout, "```")
#
#     println(fout, """
#     (Check [examples/wing_aoasweep.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/wing/wing_aoasweep.jl)
#     to see how to postprocess and plot the results as shown below)
#
#     ```@raw html
#     <center>
#         <br><b>Spanwise loading distribution</b>
#         <img src="$(remote_url)/wingexample-sweep-loading.png" alt="Pic here" style="width: 100%;"/>
#
#         <br><br><b>Vehicle lift and drag</b>
#         <img src="$(remote_url)/wingexample-sweep-CLCD.png" alt="Pic here" style="width: 100%;"/>
#
#         <br><br><b>Pitching moment</b><br>
#         <img src="$(remote_url)/wingexample-sweep-Cm.png" alt="Pic here" style="width: 50%;"/>
#     </center>
#     ```
#     """)
# end
