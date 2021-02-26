#=##############################################################################
# DESCRIPTION
    Unsteady simulation of eVTOL transition maneuver on tilt-wing, tandem,
    distributed propulsion aircraft. Geometry of a modified Vahana aircraft with
    tilt rotors.

    REFERENCES
    * Vahana geometry: Droandi, G., Syal, M., and Bower, G., “Tiltwing Multi-Rotor
    Aerodynamic Modeling in Hover, Transition and Cruise Flight Conditions,” AHS
    International 74th Annual Forum & Technology Display, 2018, p. 2018.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Feb 2021
  * License   : MIT
=###############################################################################


# ------------ MODULES ---------------------------------------------------------
using Revise
import LinearAlgebra: I
import FLOWUnsteady

uns = FLOWUnsteady
vpm = uns.vpm
vlm = uns.vlm
gt = uns.gt

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/Samsung_T5/simulationdata202102/"

# Default data path where to find rotor and airfoil data
data_path = uns.def_data_path

# ------------ HEADERS ---------------------------------------------------------
for header_name in ["geometry", "kinematics", "monitor", "misc"]
    include("vahana2_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------

function run_simulation_vahana(;    save_path=extdrive_path*"vahana2_sim13",
                                    prompt=true,
                                    run_name="vahana2",
                                    verbose=true, v_lvl=1)

    # ----------------- PARAMETERS ---------------------------------------------

    # Geometry options
    n_factor = 1                            # Refinement factor
    add_rotors = true                       # Whether to include rotors

    ## 72 steps per rev settings
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation might crash
    RPMh_w = 600.0                          # RPM of main wing rotors in hover
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = 4*5400                         # Time steps for complete maneuver
    lambda = 2.125                          # Target minimum core overlap
    p_per_step = 2                          # Particle sheds per time step
    vlm_rlx = 0.2                           # VLM relaxation (deactivated with -1)

    # # Maneuver to perform
    # ## 18 steps per rev settings
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation might crash
    # RPMh_w = 600.0                          # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 5400                           # Time steps
    # lambda = 2.125                          # Target minimum core overlap
    # p_per_step = 4                          # Particle sheds per time step
    # vlm_rlx = 0.75                          # VLM relaxation (deactivated with -1)

    # # Maneuver to perform
    # Vcruise = 0.125 * 125*0.44704         # Cruise speed
    # RPMh_w = 600                          # RPM of main wing rotors in hover
    # telapsed = 60.0                       # Total time to perform maneuver
    # nsteps = 9000                         # Time steps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation might crash
    # # RPMh_w = 200                          # RPM of main wing rotors in hover
    # # RPMh_w = 20
    # RPMh_w = 100
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 1500                           # Time steps
    # # nsteps = 100

    dt = telapsed/nsteps

    # Simulation options
    rho = 1.225                             # (kg/m^3) air density
    mu = 1.81e-5                            # Air dynamic viscosity

    # Solver options
    R = 0.75                                # (m) Reference blade radius
    # lambda = 2.125                          # Target minimum core overlap
    # p_per_step = 4                          # Particle sheds per time step
    overwrite_sigma = lambda * (2*pi*RPMh_w/60*R + Vcruise)*dt / p_per_step
    surf_sigma = R/10                       # Surface regularization
    # vlm_sigma = R/25                      # VLM regularization
    # vlm_sigma = R
    vlm_sigma = -1
    # vlm_sigma = surf_sigma # MAKE THIS -1 if unstable
    # vlm_rlx = 0.75                          # VLM relaxation (deactivated with -1)
    # shed_unsteady = false                   # Shed unsteady-loading particles
    shed_unsteady = true
    # VehicleType = uns.QVLMVehicle           # Type of vehicle to generate
    VehicleType = uns.UVLMVehicle           # Type of vehicle to generate


    vpm_formulation = vpm.formulation_sphere # VPM formulation
    # vpm_formulation = vpm.formulation_classic
    # vpm_sgsmodel    = vpm.sgs_stretching1_fmm
    vpm_sgsmodel    = vpm.generate_sgs_directionfiltered(vpm.generate_sgs_lowfiltered(vpm.sgs_stretching1_fmm))
    vpm_sgsscaling(args...) = 0.1
    # vpm_relaxation  = vpm.norelaxation
    # vpm_relaxation  = vpm.pedrizzetti
    vpm_relaxation  = vpm.correctedpedrizzetti



    # ----------------- MANEUVER DEFINITION ------------------------------------
    # Generate maneuver
    maneuver = generate_maneuver_vahana(; add_rotors=add_rotors)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; tstages=[0.2, 0.3, 0.5, 0.6])


    # ----------------- GEOMETRY GENERATION ------------------------------------
    # Generate geometry
    (vehicle, grounds) = generate_geometry_vahana(; n_factor=n_factor,
                                                    xfoil=false,
                                                    data_path=data_path,
                                                    run_name=run_name,
                                                    add_rotors=add_rotors,
                                                    VehicleType=VehicleType)

    # Move landing pad to landing area
    gt.lintransform!(grounds[2], Array(1.0I, 3, 3), Vcruise*telapsed*[-0.25, 0, -0.0025])

    # Save ground
    gt.create_path(save_path, prompt)
    for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
    end

    # ----------------- SIMULATION SETUP ---------------------------------------
    Vref = Vcruise
    RPMref = RPMh_w
    ttot = telapsed
    # max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)
    max_particles = 200000

    Vinit = Vref*maneuver.Vvehicle(0)       # (m/s) initial vehicle velocity
                                            # (rad/s) initial vehicle angular velocity
    Winit = pi/180 * (maneuver.anglevehicle(0+1e-12)-
                                          maneuver.anglevehicle(0))/(ttot*1e-12)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit)

    # ----------------- SIMULATION MONITOR -------------------------------------
    monitor_vahana = generate_monitor_vahana(vehicle, rho, RPMref, nsteps, save_path, Vinf)
    monitor_vpm = uns.generate_monitor_enstrophy(; save_path=save_path)

    monitor(args...; optargs...) = monitor_vahana(args...; optargs...) || monitor_vpm(args...; optargs...)


    # ----------------- WAKE TREATMENT FUNCTION --------------------------------
    remove_particles(args...; optargs...) = (remove_particles_lowstrength(args...; optargs...)
                                          || remove_particles_sphere(args...; optargs...))

    # ----------------- RUNTIME FUNCTION ---------------------------------------
    runtime_function(args...; optargs...) = remove_particles(args...; optargs...) || monitor(args...; optargs...)

    # ----------------- RUN SIMULATION -----------------------------------------
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      rho=rho,
                                      mu=mu,
                                      # SOLVERS OPTIONS
                                      vpm_formulation=vpm_formulation,
                                      vpm_sgsmodel=vpm_sgsmodel,
                                      vpm_sgsscaling=vpm_sgsscaling,
                                      vpm_relaxation=vpm_relaxation,
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      vlm_rlx=vlm_rlx,
                                      surf_sigma=surf_sigma,
                                      max_particles=max_particles,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=runtime_function,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      create_savepath=false,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl,
                                      save_code=splitdir(@__FILE__)[1]
                                      )

    return simulation, pfield
end


function visualize_maneuver_vahana(; save_path=extdrive_path*"vahana2_maneuver04/",
                                        prompt=true,
                                        run_name="vahana2",
                                        verbose=true, v_lvl=0,
                                        paraview=true,
                                        optargs...)

    # Maneuver to perform
    # ## 72 steps per rev settings
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 600.0                          # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 21600                          # Time steps

    ## 4 steps per rev settings
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    RPMh_w = 600.0                          # RPM of main wing rotors in hover
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = Int(21600/18)                  # Time steps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 400.0                          # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 100                            # Time steps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200.0                         # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps

    # # Maneuver to perform
    # Vcruise = 0.25 * 125*0.44704            # Cruise speed
    # RPMh_w = 1200.0                         # RPM of main wing rotors in hover
    # telapsed = 30.0                         # Total time to perform maneuver
    # nsteps = 27000                           # Time steps

    # # Maneuver to perform
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600.0                          # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps

    dt = telapsed/nsteps

    # Generate maneuver
    maneuver = generate_maneuver_vahana(; optargs...)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; tstages=[0.2, 0.3, 0.5, 0.6])


    # Generate geometry
    (vehicle, grounds) = generate_geometry_vahana(; n_factor=1,
                                                    xfoil=false,
                                                    data_path=data_path,
                                                    run_name=run_name,
                                                    optargs...)

    # Simulation setup
    Vref = Vcruise
    RPMref = RPMh_w
    ttot = telapsed

    Vinit = Vref*maneuver.Vvehicle(0)       # (m/s) initial vehicle velocity
                                            # (rad/s) initial vehicle angular velocity
    Winit = pi/180 * (maneuver.anglevehicle(0+1e-12)-
                                          maneuver.anglevehicle(0))/(ttot*1e-12)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit)

    save_vtk_optsargs = [(:save_horseshoes, false)]

    # Visualize maneuver
    strn = uns.visualize_kinematics(simulation, nsteps, save_path;
                                    run_name=run_name,
                                    save_vtk_optsargs=save_vtk_optsargs,
                                    prompt=prompt, verbose=verbose, v_lvl=v_lvl,
                                    paraview=false
                                    )

    # Move landing pad to landing area
    gt.lintransform!(grounds[2], Array(1.0I, 3, 3), Vcruise*telapsed*[-0.25, 0, -0.0025])

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
function visualize_geometry_vahana(; save_path=extdrive_path*"vahana2_geometry01/",
                                     prompt=true, run_name="vahana2", optargs...)

    (vehicle, grounds) = generate_geometry_vahana(; n_factor=1,
                                                     xfoil=false,
                                                     data_path=data_path,
                                                     run_name=run_name,
                                                     optargs...)


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
