#=
    Temporary functions for debugging
=#



function visualize_maneuver_vahana(; save_path=extdrive_path*"vahanamaneuver110/",
                                        prompt=true,
                                        run_name="vahana",
                                        verbose=true, v_lvl=1)

    # Maneuver to perform
    maneuver = maneuver_vahana1
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    RPMh_w = 400                            # RPM of main wing rotors in hover
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = 300                           # Time steps
    dt = telapsed/nsteps

    # # Maneuver to perform
    # maneuver = maneuver_vahana1
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200                            # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # # Maneuver to perform
    # maneuver = maneuver_vahana1
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200                            # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 27000                           # Time steps
    # dt = telapsed/nsteps

    # # Maneuver to perform
    # maneuver = maneuver_vahana1
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600                            # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # Generate geometry
    (system, rotors, tilting_systems, rotors_tilting_systems,
        fuselage, grounds, strn) = generategeometry_vahana(; n_factor=1,
                                                     xfoil=false,
                                                     data_path=data_path,
                                                     run_name=run_name)

    # Visualize maneuver
    visualize_kinematic(maneuver, system, rotors, tilting_systems,
                         rotors_tilting_systems, fuselage;
                         # SIMULATION OPTIONS
                         Vcruise=Vcruise,
                         RPMh_w=RPMh_w,
                         telapsed=telapsed,
                         nsteps=nsteps,
                         # OUTPUT OPTIONS
                         save_path=save_path,
                         run_name=run_name,
                         prompt=prompt,
                         verbose=verbose, v_lvl=v_lvl+1)

    # Move landing pad to landing area
    vlm.vtk.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])

    # Save ground
    strn *= run_name*"_FuselageGrid.vtk;"
    strn = replace(strn, ".", "...")

    for (i, ground) in enumerate(grounds)
      vlm.vtk.save(ground, run_name*"_Ground$i"; path=save_path)
      strn *= run_name*"_Ground$i.vtk;"
    end
    # println(strn)

    # Call paraview
    run(`paraview --data="$save_path/$strn"`)
end



function visualize_geometry_vahana(; save_path=extdrive_path*"vahanageometry03/",
                                prompt=true, run_name="vahana")

    (system, rotors, tilting_systems, rotors_tilting_systems,
        fuselage, grounds, strn) = generategeometry_vahana(; n_factor=1,
                                                     xfoil=false,
                                                     data_path=data_path,
                                                     run_name=run_name)


    if save_path != nothing
        Vinf(x,t) = [1,0,0]
        vlm.setVinf(system, Vinf)
        for rotor in rotors; vlm.setRPM(rotor, 6000); end;

        vlm.vtk.create_path(save_path, prompt)
        vlm.save(system, run_name; save_horseshoes=false, path=save_path)
        vlm.vtk.save(fuselage, run_name*"_FuselageGrid"; path=save_path)
        for (i, ground) in enumerate(grounds)
            vlm.vtk.save(ground, run_name*"_Ground$i"; path=save_path)
            strn *= run_name*"_Ground$i.vtk;"
        end

        strn *= run_name*"_FuselageGrid.vtk;"

        run(`paraview --data="$save_path/$strn"`)
    end
end
