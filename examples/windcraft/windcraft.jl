#=##############################################################################
# DESCRIPTION
Testing models, dynamics, and control of a wind-harvesting aircraft (a.k.a.,
windcraft). This vehicle is a multirotor aircraft tethered to the ground in
similitud to the Makani M600 Project, whose rotors operate both as propellers
and turbines along sections of a circular path in the presence of cross wind.

This example is a snippet of the study published under J. Mehr, E. Alvarez, A.
Cardoza, and A. Ning, "Mixed-fidelity Aerodynamic Analysis of Wind Harvesting
Aircraft," AIAA AVIATION Forum, 2020. See the paper for further details on
geometry, rotor design, and analysis.

# AUTHORSHIP
  * Author    : Judd Mehr and Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

# ------------ MODULES ---------------------------------------------------------
# Load simulation engine
# import FLOWUnsteady
reload("FLOWUnsteady")
uns = FLOWUnsteady
vlm = uns.vlm
gt = uns.gt

using PyPlot

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata7/"
# extdrive_path = "temps/"


# ------------ HEADERS ---------------------------------------------------------
for header_name in ["geometry", "kinematics"]
    include("windcraft_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------



"""
    Visualize kinematic maneuver of the windcraft: saves it as vtk files, and
    calls Paraview visualizing the vehicle path.
"""
function visualize_maneuver_windcraft_kinematic(; save_path=extdrive_path*"windcraft_maneuver08/",
                                                    prompt=true,
                                                    run_name="windcraft",
                                                    verbose=true, v_lvl=0,
                                                    paraview=true)

    # Geometry parameters
    includewing     = true
    includetail     = false
    includepylons   = false
    includerotors   = false
    numrotors       = 8
    inlinerotors    = false
    includecontrols = false

    # Maneuver parameters
    R               = 135/2             # (m) radius of circle
    t_per_rev       = 3.0               # (s) time of one full revolution
    nrevs           = 1.00              # Revolutions to simulate
    RPMref          = 4.0*40.0*30/pi    # Reference RPM: 40m/s with tip speed ratio of 4

    Vmean           = 2*pi*R/t_per_rev/nrevs  # (m/s) mean velocity along a full circle
    ttot            = nrevs*t_per_rev   # (s) total time to perform maneuver
    nsteps          = 60                # Time steps

    tinit           = 0.00              # (s) initial time (sim time by end will be tinit+ttot)

    # Circular path parameters
    theta0          = 0.0
    thetan          = 360.0
    omegan          = [4.0/9.0, 20.0/27.0, 4.0/9.0]
    tn              = [0.0, 0.5, 1.0]

    # Generate maneuver
    gt.verbalize("MANEUVER GENERATION", v_lvl, verbose)

    maneuver = generate_maneuver_windcraft_kinematic(nrevs;
                                                       disp_plot        = false,
                                                       includetail      = includetail,
                                                       includewing      = includewing,
                                                       includecontrols  = includecontrols,
                                                       includerotors    = includerotors,
                                                       theta0           = theta0,
                                                       thetan           = thetan,
                                                       omegan           = omegan,
                                                       tn               = tn)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; tstages=[])


    # Generate geometry
    gt.verbalize("VEHICLE GENERATION", v_lvl, verbose)

    (system, vlm_system, rotors,
     tilting_systems, rotor_systems,
        wake_system) = generate_geometry_windcraft(;
                                                    includewing     = includewing,
                                                    includetail     = includetail,
                                                    includepylons   = includepylons,
                                                    includerotors   = includerotors,
                                                    numrotors       = numrotors,
                                                    inlinerotors    = inlinerotors,
                                                    includecontrols = includecontrols,
                                                    xfoil           = false,
                                                    data_path       = uns.def_data_path,
                                                    run_name        = run_name,
                                                    verbose         = verbose,
                                                    v_lvl           = v_lvl+1)


    vehicle = uns.VLMVehicle(   system;
                                tilting_systems = tilting_systems,
                                rotor_systems   = rotor_systems,
                                vlm_system      = vlm_system,
                                wake_system     = wake_system,
                             )

    # Simulation setup
    gt.verbalize("SIMULATION GENERATION", v_lvl, verbose)

    Vref = Vmean                            # (m/s) reference velocity
    Vinit = Vref*maneuver.Vvehicle(tinit)   # (m/s) initial vehicle velocity
                                            # (rad/s) initial vehicle angular velocity
    angle1 = maneuver.anglevehicle(tinit/ttot)
    angle2 = maneuver.anglevehicle(tinit/ttot + 1e-12)
    Winit = pi/180 * (angle2-angle1)/(ttot*1e-12)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                            Vinit=Vinit, Winit=Winit, t=tinit)

    save_vtk_optsargs = [(:save_horseshoes, false)]

    # Visualize maneuver
    gt.verbalize("STEPPING THROUGH MANEUVER", v_lvl, verbose)

    strn = uns.visualize_kinematics(simulation, nsteps, save_path;
                                    run_name=run_name,
                                    save_vtk_optsargs=save_vtk_optsargs,
                                    prompt=prompt, verbose=verbose, v_lvl=v_lvl,
                                    paraview=false
                                    )

    # # Move landing pad to landing area
    # vlm.vtk.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])
    #
    # # Save ground
    # for (i, ground) in enumerate(grounds)
    #     gt.save(ground, run_name*"_Ground$i"; path=save_path)
    #     strn *= run_name*"_Ground$i.vtk;"
    # end

    # Call paraview
    if paraview
        run(`paraview --data="$save_path/$strn"`)
    end

    return strn
end

"""
    Generates geometry of the windcraft, saves it as vtk files, and calls
    Paraview visualizing the VTK geometry.
"""
function visualize_geometry_windcraft(; save_path   = extdrive_path*"windcraft_geometry00/",
                                        run_name    = "windcraft",
                                        prompt      = true,
                                        verbose     = true,
                                        v_lvl       = 0,
                                        paraview    = true,
                                        xfoil       = false,
                                        optargs...)

    gt.verbalize("GEOMETRY VISUALIZATION", v_lvl, verbose)

    # Generate the Geometry in order to visualize
    (system, vlm_system, rotors,
     tilting_systems, rotor_systems,
        wake_system) = generate_geometry_windcraft(; xfoil=xfoil,
                                                  data_path=uns.def_data_path,
                                                  run_name=run_name,
                                                  v_lvl=v_lvl+1,
                                                  verbose=verbose, optargs...)

    # Set up dummy values of RPM and freestream
    Vinf(x,t) = [1,0,0]
    vlm.setVinf(system, Vinf)
    for rotor in rotors
        vlm.setRPM(rotor, 6000)
    end

    # Save visualization of geometry
    gt.create_path(save_path, prompt)
    strn = vlm.save(system, run_name; save_horseshoes=false, path=save_path)

    # Call Paraview
    if paraview
        run(`paraview --data="$save_path/$strn"`)
    end

end
