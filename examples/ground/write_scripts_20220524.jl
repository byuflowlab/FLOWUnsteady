# prepares scripts to be run on the BYU supercomputer
# run this script on the supercomputer
import Dates

TODAY = replace(string(Dates.today()), '-' => "")
save_path = joinpath(ENV["HOME"], "compute", TODAY)
if !isdir(save_path); mkdir(save_path); end

import FLOWUnsteady
uns = FLOWUnsteady

function write_script_20220524(name="healy_rotors";
        script_path = joinpath(save_path, name*".jl"),
        example_file = joinpath("ground", "rotors.jl"),
        include_no_ground=true,
        FLOWExaFMM_dir=joinpath(ENV["HOME"], "julia", "dev", "FLOWExaFMM")
    )

    RPM = 1600.0
    hd_list = ["0.5", "1.0", "0.5", "1.0"]
    dx_list = ["3.0", "3.0", "6.0", "6.0"]
    nx_list = ["10", "10", "20", "20"]
    npr_list = ["72", "72", "72", "72"]
    call_example_functions = get_function_strings(RPM, hd_list, dx_list, nx_list; include_no_ground)
    @show call_example_functions
    uns.write_script_example(script_path, example_file, call_example_functions...)
    uns.write_batch(script_path[1:end-2]*"sh", [script_path], FLOWExaFMM_dir;
        test=true, email_notifications=false, email_address="",
        time="01:00:00", ntasks=24, mem_per_cpu="1024M"
    )
    return nothing
end

# Healy rotor, single rotor, no ground
rotor_function_no_ground(RPM, nsteps_per_rev) =
"""
run_singlerotor_hover(; xfoil=false, RPM=$RPM, rotor_file="Healy_rotor.csv", prompt=false, save_path=joinpath("$save_path","single_rotor_hover_ground_none"), nrevs=14, nsteps_per_rev=$nsteps_per_rev)
"""

rotor_function_panel_ground(tag, h_over_d, dx, nx, RPM, nsteps_per_rev) =
"""
run_singlerotor_hover_ground_panel(; run_name="healy_rotor_panel"*$tag, xfoil=false, prompt=false, disp_conv=true,
        ground_point = [5.5*0.3048 * $h_over_d,0,0], ground_axes = [0.0 0.0 1.0; 0.0 1.0 0.0; -1.0 0 0.0],
        Delta_x = $dx, Delta_y = $dx, nx = $nx, ny = $nx,
        kernel = PS.Source(), panel_shape = PS.Quad(),
        RPM = $RPM,
        nrevs = 14, nsteps_per_rev = $(nsteps_per_rev),
        save_path=joinpath("$save_path","single_rotor_hover_ground_panel"),
    )
"""

rotor_function_images_ground(tag, h_over_d, RPM, nsteps_per_rev) =
"""
run_singlerotor_hover_ground_mirror(; run_name="rotor_mirror"*$tag, xfoil=false, prompt=false, disp_conv=false,
        ground_point = [5.5*0.3048 * $h_over_d,0,0], ground_normal = [-1.0, 0,0], RPM = $RPM,
        nrevs = 14, nsteps_per_rev = $(nsteps_per_rev),
        save_path=joinpath("$save_path","single_rotor_hover_ground_images"),
    )
"""

function get_function_strings(RPM, hd_list, dx_list, nx_list, npr_list; include_no_ground=true)
    ex_funs = String[]
    if include_no_ground; push!(ex_funs, rotor_function_no_ground(RPM)); end
    for i in 1:length(hd_list)
        hd = hd_list[i]
        dx = dx_list[i]
        nx = nx_list[i]
        npr = npr_list[i]
        push!(ex_funs, rotor_function_panel_ground("_hd$(hd)_dx$(dx)_nx$(nx)_npr$npr", parse(Float64,hd), parse(Float64,dx), parse(Int64,nx), RPM, parse(Float64,npr)))
    end
    return ex_funs
end
