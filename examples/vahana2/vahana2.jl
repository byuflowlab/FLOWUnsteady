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
import QuadGK
import LinearAlgebra: I
import FLOWVPM
import FLOWUnsteady

uns = FLOWUnsteady
vpm = FLOWVPM
vlm = uns.vlm
gt = uns.gt

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
# extdrive_path = "/media/edoalvar/Samsung_T51/simulationdata202202/"
# extdrive_path = "/media/flowlab/Storage/ealvarez/simulations/"
extdrive_path = "/fslhome/edoalvar/simulationdataFSLG1/"

# Default data path where to find rotor and airfoil data
data_path = uns.def_data_path

# ------------ HEADERS ---------------------------------------------------------
for header_name in ["geometry", "kinematics", "monitor", "postprocessing"]
    include("vahana2_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------

function run_simulation_vahana(;    save_path=extdrive_path*"vahana2_sim22",
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
    # p_per_step = 2                          # Particle sheds per time step
    vlm_rlx = 0.2                           # VLM relaxation (deactivated with -1)

    p_per_step = 2*2                          # Particle sheds per time step

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

    ## Segment of the maneuver to perform
    # # Beginning to end
    # tstart = 0
    # tquit = Inf

    # # Hover segment
    # tstart = 0.07*telapsed
    # tquit = 0.14*telapsed

    # Hover-cruise transition segment
    tstart = 0.20*telapsed
    tquit = 0.35*telapsed

    start_kinmaneuver = false               # If true, it starts the maneuver with the
                                            # velocity and angles of tstart.
                                            # If false, starts with velocity=0
                                            # and angles as initiated by the geometry


    dt = telapsed/nsteps

    # Simulation options
    rho = 1.225                             # (kg/m^3) air density
    mu = 1.81e-5                            # Air dynamic viscosity

    # Solver options
    R = 0.75                                # (m) Reference blade radius
    # lambda = 2.125                          # Target minimum core overlap
    # p_per_step = 4                          # Particle sheds per time step
    sigma_vpm_overwrite = lambda * (2*pi*RPMh_w/60*R + Vcruise)*dt / p_per_step


    sigma_vlm_surf  = R/50                 # Size of embedded particles representing VLM surfaces (for VLM-on-VPM and VLM-on-Rotor)
    sigma_rotor_surf= R/80                 # Size of embedded particles representing rotor blade surfaces (for Rotor-on-VPM, Rotor-on-VLM, and Rotor-on-Rotor)
    sigma_vlm_solver= -1                   # Regularization of VLM solver (internal)
    sigmafactor_vpmonvlm   = 1.0           # Shrinks the particles by this factor when calculated VPM-on-VLM/Rotor induced velocities
    # sigmafactor_vpmonvlm   = 1.0 * nsteps_per_rev/72
    # vlm_rlx         = 0.5                # VLM relaxation
    # vlm_rlx         = 0.3


    # shed_unsteady = false                   # Shed unsteady-loading particles
    shed_unsteady = true
    # unsteady_shedcrit      = 0.0001      # Shed unsteady loading whenever circulation fluctuates more than this ratio
    unsteady_shedcrit      = 0.001
    shed_starting   = true                 # Shed starting vortex
    # shed_starting   = false

    # VehicleType = uns.QVLMVehicle           # Type of vehicle to generate
    VehicleType = uns.UVLMVehicle           # Type of vehicle to generate

    hubtiploss_correction = vlm.hubtiploss_correction_prandtl   # Hub and tip correction


    vpm_formulation = vpm.formulation_rVPM # VPM formulation
    # vpm_formulation = vpm.formulation_cVPM
    # vpm_SFS = vpm.SFS_none
    # vpm_SFS = vpm.SFS_Cs_nobackscatter
    vpm_SFS = vpm.SFS_Cd_twolevel_nobackscatter
    # vpm_SFS = vpm.DynamicSFS(vpm.Estr_fmm, vpm.pseudo3level_positive; alpha=0.999,
    #                             clippings=[vpm.clipping_backscatter],
    #                             controls=[vpm.control_directional, vpm.control_magnitude])
    # vpm_relaxation  = vpm.norelaxation
    vpm_relaxation  = vpm.pedrizzetti
    # vpm_relaxation  = vpm.correctedpedrizzetti



    # ----------------- MANEUVER DEFINITION ------------------------------------
    # Generate maneuver
    maneuver = generate_maneuver_vahana(; add_rotors=add_rotors)

    # Plot maneuver path and controls
    gt.create_path(save_path, prompt)
    uns.plot_maneuver(maneuver; tstages=[0.2, 0.3, 0.5, 0.6], save_path=save_path)


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
    for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
    end

    # ----------------- SIMULATION SETUP ---------------------------------------
    Vref = Vcruise
    RPMref = RPMh_w
    ttot = telapsed
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)
    if tquit != Inf
        max_particles = ceil(Int, max_particles*(tquit-tstart)/ttot)
    end

    max_particles = minimum(Int(10e6), max_particles)

    t0 = tstart/ttot*start_kinmaneuver

    Vinit = Vref*maneuver.Vvehicle(t0)       # (m/s) initial vehicle velocity
                                            # (rad/s) initial vehicle angular velocity
    Winit = pi/180 * (maneuver.anglevehicle(t0+1e-12)- maneuver.anglevehicle(t0))/(ttot*1e-12)

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit,
                                                    t=tstart)

    # ----------------- SIMULATION MONITOR -------------------------------------
    monitor_vahana = generate_monitor_vahana(vehicle, rho, RPMref, nsteps, save_path, Vinf)

    monitors_vpm = []
    if VehicleType == uns.UVLMVehicle

        monitor_enstrophy = uns.generate_monitor_enstrophy(; save_path=save_path)
        push!(monitors_vpm, monitor_enstrophy)

        if vpm.isSFSenabled(vpm_SFS)
            monitor_Chistory = uns.generate_monitor_Cd(; save_path=save_path)
            push!(monitors_vpm, monitor_Chistory)
        end
    end

    monitor_vpm(args...; optargs...) = length(monitors_vpm)==0 ? false : !prod(!f(args...; optargs...) for f in monitors_vpm)

    monitor(args...; optargs...) = monitor_vahana(args...; optargs...) || monitor_vpm(args...; optargs...)


    # ----------------- WAKE TREATMENT FUNCTION --------------------------------
    wake_treatment_lowstrength = uns.remove_particles_lowstrength( (0.00001^2 * 2*2/p_per_step * dt/(30/(4*5400)) )^2, 1)
    wake_treatment_sphere = uns.remove_particles_sphere((1.25*5.86)^2, 1; Xoff=[4.0, 0, 0])
    remove_particles(args...; optargs...) = (wake_treatment_lowstrength(args...; optargs...) || wake_treatment_sphere(args...; optargs...))

    # ----------------- RUNTIME FUNCTION ---------------------------------------
    runtime_function(args...; optargs...) = remove_particles(args...; optargs...) || monitor(args...; optargs...)

    # ----------------- RUN SIMULATION -----------------------------------------
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      rho=rho,
                                      mu=mu,
                                      tquit=tquit,
                                      # SOLVERS OPTIONS
                                      max_particles=max_particles,
                                      p_per_step=p_per_step,
                                      vpm_formulation=vpm_formulation,
                                      vpm_SFS=vpm_SFS,
                                      vpm_relaxation=vpm_relaxation,
                                      vlm_rlx=vlm_rlx,
                                      sigma_vpm_overwrite=sigma_vpm_overwrite,
                                      sigma_vlm_solver=sigma_vlm_solver,
                                      sigma_vlm_surf=sigma_vlm_surf,
                                      sigma_rotor_surf=sigma_rotor_surf,
                                      sigmafactor_vpmonvlm=sigmafactor_vpmonvlm,
                                      hubtiploss_correction=hubtiploss_correction,
                                      shed_unsteady=shed_unsteady,
                                      unsteady_shedcrit=unsteady_shedcrit,
                                      shed_starting=shed_starting,
                                      extra_runtime_function=runtime_function,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      create_savepath=false,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl,
                                      save_horseshoes=VehicleType==uns.QVLMVehicle && false,
                                      save_code=splitdir(@__FILE__)[1]
                                      )

    return simulation, pfield
end


function visualize_maneuver_vahana(; save_path=extdrive_path*"vahana2_maneuver00/",
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
function visualize_geometry_vahana(; save_path=extdrive_path*"vahana2_geometry00/",
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

function vahana2_postprocess(; ite=1)

    range = collect(4:4:6480)                   # Time steps to read
    nchunks = 4                                 # Number of chunks
    chunk = ceil(Int, length(range)/nchunks)    # Chunk length
    # ite = 1                                     # This chunk to iterate over

    nums = range[chunk*(ite-1) + 1 : (chunk*ite > length(range) ? length(range) : chunk*ite)]

    read_path = extdrive_path*"vahana2_sim16/"
    save_path = extdrive_path*"vahana2_sim16-fdom00-$(ite)/"


    postprocessing_fluiddomain(read_path, save_path, nums;
                                        # PROCESSING OPTIONS
                                        fdx=1/80,                # Scaling of cell length
                                        static_particles=false,  # Add static particles
                                        # OUTPUT OPTIONS
                                        run_name="vahana2",
                                        prompt=true, paraview=false)
end
