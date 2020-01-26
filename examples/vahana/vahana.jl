#=##############################################################################
# DESCRIPTION
    Unsteady simulation of eVTOL transition maneuver on tilt-wing, tandem,
    distributed propulsion aircraft.

    REFERENCES
    * Vahana geometry: Droandi, G., Syal, M., and Bower, G., “Tiltwing Multi-Rotor
    Aerodynamic Modeling in Hover, Transition and Cruise Flight Conditions,” AHS
    International 74th Annual Forum & Technology Display, 2018, p. 2018.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


# ------------ MODULES ---------------------------------------------------------
# Load simulation engine
# import FLOWUnsteady
reload("FLOWUnsteady")
uns = FLOWUnsteady
vlm = uns.vlm

import GeometricTools
gt = GeometricTools

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"

# Default data path where to find rotor and airfoil data
data_path = joinpath(splitdir(@__FILE__)[1], "../../data/")

# ------------ HEADERS ---------------------------------------------------------
for header_name in ["geometry", "kinematics"]
    include("vahana_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------

function run_simulation_vahana(;    # save_path="temps/vahanasimulation01",
                                    save_path=extdrive_path*"vahanasimulation10",
                                    prompt=true,
                                    run_name="vahana",
                                    verbose=true, v_lvl=1)

    # # Maneuver to perform
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600                            # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # Geometry options
    n_factor = 5                              # Refinement factor

    # Maneuver to perform
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation will crash
    Vinf(x,t) = 1.0*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation will crash
    # RPMh_w = 200                            # RPM of main wing rotors in hover
    RPMh_w = 20
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = 1500                           # Time steps
    # nsteps = 100
    dt = telapsed/nsteps

    # Solver options
    R = 0.75                                # (m) blade radius as a reference
    lambda = 4.0                            # Target minimum core overlap
    p_per_step = 4                          # Particle sheds per time step
    overwrite_sigma = lambda * (2*pi*RPMh_w/60*R + Vcruise)*dt / p_per_step
    # vlm_sigma = R/25                        # VLM regularization
    vlm_sigma = R                        # VLM regularization
    surf_sigma = vlm_sigma                  # Surface regularization
    shed_unsteady = false                   # Shed unsteady-loading particles

    # Generate maneuver
    maneuver = generate_maneuver_vahana1()

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; tstages=[0.2, 0.3, 0.5, 0.6])


    # Generate geometry
    (vehicle, grounds) = generate_geometry_vahana(; n_factor=n_factor,
                                                    xfoil=false,
                                                    data_path=data_path,
                                                    run_name=run_name)

    # Simulation setup
    Vref = Vcruise
    RPMref = RPMh_w
    ttot = telapsed
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot)


    # Run simulation
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      surf_sigma=surf_sigma,
                                      max_particles=max_particles,
                                      shed_unsteady=shed_unsteady,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl,
                                      )


    # Move landing pad to landing area
    gt.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])

    # Save ground
    for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
    end

    return simulation, pfield
end


function visualize_maneuver_vahana(; save_path=extdrive_path*"vahanamaneuver100/",
                                        prompt=true,
                                        run_name="vahana",
                                        verbose=true, v_lvl=0,
                                        paraview=true)

    # Maneuver to perform
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    RPMh_w = 400.0                          # RPM of main wing rotors in hover
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = 100                           # Time steps
    dt = telapsed/nsteps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200.0                         # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200.0                         # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 27000                           # Time steps
    # dt = telapsed/nsteps

    # # Maneuver to perform
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600.0                          # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # Generate maneuver
    maneuver = generate_maneuver_vahana1()

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; tstages=[0.2, 0.3, 0.5, 0.6])


    # Generate geometry
    (vehicle, grounds) = generate_geometry_vahana(; n_factor=1,
                                                    xfoil=false,
                                                    data_path=data_path,
                                                    run_name=run_name)

    # Simulation setup
    Vref = Vcruise
    RPMref = RPMh_w
    ttot = telapsed
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot)

    save_vtk_optsargs = [(:save_horseshoes, false)]

    # Visualize maneuver
    strn = uns.visualize_kinematics(simulation, nsteps, save_path;
                                    run_name=run_name,
                                    save_vtk_optsargs=save_vtk_optsargs,
                                    prompt=prompt, verbose=verbose, v_lvl=v_lvl,
                                    paraview=false
                                    )

    # Move landing pad to landing area
    vlm.vtk.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])

    # Save ground
    for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
        strn *= run_name*"_Ground$i.vtk;"
    end

    # Call paraview
    if paraview
        run(`paraview --data="$save_path/$strn"`)
    end

    return strn
end


"""
    Generates geometry of Vahana aircraft, saves it as vtk files, and calls
    Paraview visualizing the VTK geometry.
"""
function visualize_geometry_vahana(; save_path=extdrive_path*"vahanageometry00/",
                                     prompt=true, run_name="vahana")

    (vehicle, grounds) = generate_geometry_vahana(; n_factor=1,
                                                     xfoil=false,
                                                     data_path=data_path,
                                                     run_name=run_name)


    # Setup dummy freestream and RPMs for horseshoe visualization
    Vinf(x,t) = [1,0,0]
    vlm.setVinf(vehicle.system, Vinf)

    for rotor_system in vehicle.rotor_systems
        for rotor in rotor_system
            vlm.setRPM(rotor, 6000)
        end
    end

    # Create save path
    gt.create_path(save_path, prompt)

    # Save vehicle
    strn = uns.save_vtk(vehicle, run_name; path=save_path, save_horseshoes=false)

    # Save ground
    for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
        strn *= run_name*"_Ground$i.vtk;"
    end

    # Call Paraview
    run(`paraview --data="$save_path/$strn"`)
end
