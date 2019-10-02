# Load simulation engine
include(joinpath(splitdir(@__FILE__)[1], "../src/fvs.jl"))

#=
    Temporary functions for debugging
=#



function run_simulation_vahana(;    # save_path="temps/vahanasimulation01",
                                    save_path=extdrive_path*"vahanasimulation02",
                                    prompt=true,
                                    run_name="vahana",
                                    verbose=true, v_lvl=1)

    # # Maneuver to perform
    # maneuver = maneuver_vahana1
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600                            # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # Geometry options
    n_factor = 5                              # Refinement factor

    # Maneuver to perform
    maneuver = maneuver_vahana1
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation will crash
    Vinf(x,t) = 1.0*[0,0,-1]               # (m/s) freestream velocity, if 0 the simulation will crash
    # RPMh_w = 200                            # RPM of main wing rotors in hover
    RPMh_w = 20
    telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 1500                           # Time steps
    nsteps = 300
    dt = telapsed/nsteps

    # Solver options
    R = 0.75                                # (m) blade radius as a reference
    lambda = 4.0                            # Target minimum core overlap
    p_per_step = 4                          # Particle sheds per time step
    overwrite_sigma = lambda * (2*pi*RPMh_w/60*R + Vcruise)*dt / p_per_step
    # vlm_sigma = R/25                        # VLM regularization
    vlm_sigma = R                        # VLM regularization

    # Generate geometry
    (system, rotors,
            tilting_systems, rotors_systems,
            vlm_system, wake_system,
            fuselage, grounds, strn) = generategeometry_vahana(; n_factor=n_factor,
                                                             xfoil=false,
                                                             data_path=data_path,
                                                             run_name=run_name)

     run_simulation(maneuver, system, rotors,
                                  tilting_systems, rotors_systems,
                                  wake_system, vlm_system,
                                  fuselage;
                                  # SIMULATION OPTIONS
                                  Vcruise=Vcruise,
                                  RPMh_w=RPMh_w,
                                  telapsed=telapsed,
                                  nsteps=nsteps,
                                  Vinf=Vinf,
                                  # SOLVERS OPTIONS
                                  p_per_step=p_per_step,
                                  overwrite_sigma=overwrite_sigma,
                                  vlm_sigma=vlm_sigma,
                                  # OUTPUT OPTIONS
                                  save_path=save_path,
                                  run_name=run_name,
                                  prompt=prompt,
                                  verbose=verbose, v_lvl=v_lvl,
                                  # paraview=false
                                  )


      # Move landing pad to landing area
      gt.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])

      # Save ground
      strn *= run_name*"_FuselageGrid.vtk;"
      strn = replace(strn, ".", "...")

      for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
        strn *= run_name*"_Ground$i.vtk;"
      end
      # println(strn)

      # Call paraview
      run(`paraview --data="$save_path/$strn"`)
end


function visualize_maneuver_vahana(; save_path=extdrive_path*"vahanamaneuver000/",
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
    (system, rotors,
            tilting_systems, rotors_systems,
            vlm_system, wake_system,
            fuselage, grounds, strn) = generategeometry_vahana(; n_factor=1,
                                                     xfoil=false,
                                                     data_path=data_path,
                                                     run_name=run_name)

    # Visualize maneuver
    visualize_kinematic(maneuver, system, rotors, tilting_systems,
                         rotors_systems, fuselage;
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



function visualize_geometry_vahana(; save_path=extdrive_path*"vahanageometry00/",
                                     prompt=true, run_name="vahana")

    (system, rotors,
            tilting_systems, rotors_systems,
            vlm_system, wake_system,
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
