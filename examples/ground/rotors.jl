import FLOWUnsteady
uns = FLOWUnsteady
vpm = uns.vpm
vlm = uns.vlm
PS = uns.PanelSolver
import PyPlot
plt = PyPlot

import Dates
const TODAY = replace(string(Dates.today()), '-' => "")

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = joinpath(uns.module_path,"..","data",TODAY)
if !isdir(extdrive_path); mkpath(extdrive_path); end



# ------------ DRIVERS ---------------------------------------------------------
function run_singlerotor_hover_ground_panel(; xfoil=true, prompt=true, disp_conv=false,
        ground_point = [0.0,0,0], ground_axes = [1.0 0 0; 0 1 0; 0 0 1],
        Delta_x = 4.0, Delta_y = 4.0, nx = 10, ny = 10,
        kernel = PS.Source(), panel_shape = PS.Quad()
    )

    J = 0.00                # Advance ratio Vinf/(nD)
    angle = 0.0             # (deg) angle of freestream (0 == climb, 90==forward flight)

    ground_type = uns.Panel(ground_point, ground_axes, Delta_x, Delta_y, nx, ny, kernel, panel_shape)

    singlerotor(;   xfoil=xfoil,
                    VehicleType=uns.VLMVehicle,
                    J=J,
                    DVinf=[cos(pi/180*angle), sin(pi/180*angle), 0],
                    save_path=joinpath(extdrive_path,"singlerotor_hover_test00"),
                    prompt=prompt, disp_conv,
                    ground_type=ground_type)
end

function run_singlerotor_hover_ground_mirror(; xfoil=true, prompt=true, disp_conv=false,
        ground_point = [0.0,0,0], ground_axes = [1.0 0 0; 0 1 0; 0 0 1]
    )

    J = 0.00                # Advance ratio Vinf/(nD)
    angle = 0.0             # (deg) angle of freestream (0 == climb, 90==forward flight)

    ground_type = uns.Mirror(ground_point, ground_axes)

    singlerotor(;   xfoil=xfoil,
                    VehicleType=uns.VLMVehicle,
                    J=J,
                    DVinf=[cos(pi/180*angle), sin(pi/180*angle), 0],
                    save_path=joinpath(extdrive_path,"singlerotor_hover_test00"),
                    prompt=prompt, disp_conv, ground_type=ground_type)
end

function run_singlerotor_forwardflight(; xfoil=true, prompt=true)

    J = 0.15                # Advance ratio Vinf/(nD)
    angle = 60.0            # (deg) angle of freestream (0 == climb, ~90==forward flight)

    singlerotor(;   xfoil=xfoil,
                    VehicleType=uns.VLMVehicle,
                    J=J,
                    DVinf=[cos(pi/180*angle), sin(pi/180*angle), 0],
                    nrevs=2,
                    nsteps_per_rev=120,
                    save_path=joinpath(extdrive_path,"singlerotor_fflight_test01/"),
                    prompt=prompt)
end


# ------------------------------------------------------------------------------

function singlerotor(;  xfoil       = true,             # Whether to run XFOIL
                        VehicleType = uns.VLMVehicle,   # Vehicle type
                        J           = 0.0,              # Advance ratio
                        DVinf       = [1.0, 0, 0],      # Freestream direction
                        nrevs       = 6,                # Number of revolutions
                        nsteps_per_rev = 72,            # Time steps per revolution
                        shed_unsteady = true,
                        lambda      = 2.125,
                        n           = 10,
                        overwrite_overwrite_sigma = nothing,
                        # OUTPUT OPTIONS
                        save_path   = nothing,
                        run_name    = "singlerotor",
                        prompt      = true,
                        verbose     = true,
                        v_lvl       = 0,
                        rotor_file = "DJI-II.csv",           # Rotor geometry
                        disp_conv = true,
                        optargs...)

    # TODO: Wake removal ?

    # ------------ PARAMETERS --------------------------------------------------

    # Rotor geometry
    data_path = uns.def_data_path       # Path to rotor database
    pitch = 0.0                         # (deg) collective pitch of blades
    # n = 50                              # Number of blade elements
    # n = 10
    CW = false                          # Clock-wise rotation
    # xfoil = false                     # Whether to run XFOIL

    # Read radius of this rotor and number of blades
    R, B = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]

    # Simulation parameters
    RPM = 81*60                         # RPM
    # J = 0.00001                       # Advance ratio Vinf/(nD)
    rho = 1.225                         # (kg/m^3) air density
    mu = 1.81e-5                        # (kg/ms) air dynamic viscosity
    ReD = 2*pi*RPM/60*R * rho/mu * 2*R  # Diameter-based Reynolds number

    magVinf = J*RPM/60*(2*R)
    Vinf(X,t) = magVinf*DVinf           # (m/s) freestream velocity

    # Solver parameters
    # nrevs = 6                         # Number of revolutions in simulation
    # nsteps_per_rev = 72                 # Time steps per revolution
    p_per_step = 2                      # Sheds per time step
    ttot = nrevs/(RPM/60)               # (s) total simulation time
    nsteps = nrevs*nsteps_per_rev       # Number of time steps
    # lambda = 2.125                      # Core overlap
    overwrite_sigma = overwrite_overwrite_sigma != nothing ? overwrite_overwrite_sigma :
                                        lambda * 2*pi*R/(nsteps_per_rev*p_per_step) # Smoothing core size

    surf_sigma = R/10                   # Smoothing radius of lifting surface
    vlm_sigma = surf_sigma              # Smoothing radius of VLM
    # shed_unsteady = true                # Shed particles from unsteady loading

    max_particles = ((2*n+1)*B)*nrevs*nsteps_per_rev*p_per_step # Max particles for memory pre-allocation
    plot_disc = true                    # Plot blade discretization for debugging


    # ------------ SIMULATION SETUP --------------------------------------------
    # Generate rotor
    rotor = uns.generate_rotor(rotor_file; pitch=pitch,
                                            n=n, CW=CW, ReD=ReD,
                                            verbose=verbose, xfoil=xfoil,
                                            data_path=data_path,
                                            plot_disc=plot_disc)
    # ----- VEHICLE DEFINITION
    # System of all FLOWVLM objects
    system = vlm.WingSystem()
    vlm.addwing(system, run_name, rotor)

    # Systems of rotors
    rotors = vlm.Rotor[rotor]   # Defining this rotor as its own system
    rotor_systems = (rotors,)

    # Wake-shedding system (doesn't include the rotor if quasi-steady vehicle)
    wake_system = vlm.WingSystem()

    if VehicleType != uns.QVLMVehicle
        vlm.addwing(wake_system, run_name, rotor)
    else
        # Mute colinear warnings. This is needed since the quasi-steady solver
        #   will probe induced velocities at the lifting line of the blade
        uns.vlm.VLMSolver._mute_warning(true)
    end

    # FVS's Vehicle object
    vehicle = VehicleType(   system;
                                rotor_systems=rotor_systems,
                                wake_system=wake_system
                             )

    # ----- MANEUVER DEFINITION
    RPM_fun(t) = 1.0                # RPM (normalized by reference RPM) as a
                                    # function of normalized time

    angle = ()                      # Angle of each tilting system (none in this case)
    sysRPM = (RPM_fun, )              # RPM of each rotor system
    Vvehicle(t) = zeros(3)          # Translational velocity of vehicle over Vcruise
    anglevehicle(t) = zeros(3)      # (deg) angle of the vehicle

    # FVS's Maneuver object
    maneuver = uns.KinematicManeuver(angle, sysRPM, Vvehicle, anglevehicle)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; vis_nsteps=nsteps)


    # ----- SIMULATION DEFINITION
    RPMref = RPM
    Vref = 0.0
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot)

    monitor = generate_monitor(J, rho, RPM, nsteps; save_path=save_path,
                                                            run_name=run_name, disp_conv)


    # ------------ RUN SIMULATION ----------------------------------------------
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
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl,
                                      optargs...
                                      )
    return pfield, rotor
end


"""
Generate monitor for rotor performance parameters
"""
function generate_monitor(J, rho, RPM, nsteps; save_path=nothing,
                            run_name="singlerotor",
                            figname="monitor", disp_conv=true,
                            nsteps_savefig=10)

    fcalls = 0                  # Number of function calls

    colors="rgbcmy"^100
    stls = "o^*.px"^100

    # Name of convergence file
    if save_path!=nothing
        fname = joinpath(save_path, run_name*"_convergence.csv")
    end

    # Function for run_vpm! to call on each iteration
    function extra_runtime_function(sim::uns.Simulation{V, M, R},
                                    PFIELD::uns.vpm.ParticleField,
                                    T::Real, DT::Real
                                   ) where{V<:uns.AbstractVLMVehicle, M, R}

        rotors = vcat(sim.vehicle.rotor_systems...)
        angle = T*360*RPM/60


        # Call figure
        if disp_conv; fig = plt.figure(figname, figsize=(7*3,5*2)); end;

        if fcalls==0
            # Format subplots
            if disp_conv
                plt.subplot(231)
                plt.title("Circulation Distribution")
                plt.xlabel("Element index")
                plt.ylabel(L"Circulation $\Gamma$ (m$^2$/s)")
                plt.grid(true, color="0.8", linestyle="--")
                plt.subplot(232)
                plt.title("Plane-of-rotation Normal Force")
                plt.xlabel("Element index")
                plt.ylabel(L"Normal Force $N_p$ (N)")
                plt.grid(true, color="0.8", linestyle="--")
                plt.subplot(233)
                plt.title("Plane-of-rotation Tangential Force")
                plt.xlabel("Element index")
                plt.ylabel(L"Tangential Force $T_p$ (N)")
                plt.grid(true, color="0.8", linestyle="--")
                plt.subplot(234)
                plt.xlabel(L"Age $\psi$ ($^\circ$)")
                plt.ylabel(L"Thrust $C_T$")
                plt.grid(true, color="0.8", linestyle="--")
                plt.subplot(235)
                plt.xlabel(L"Age $\psi$ ($^\circ$)")
                plt.ylabel(L"Torque $C_Q$")
                plt.grid(true, color="0.8", linestyle="--")
                plt.subplot(236)
                plt.xlabel(L"Age $\psi$ ($^\circ$)")
                plt.ylabel(L"Propulsive efficiency $\eta$")
                plt.grid(true, color="0.8", linestyle="--")
            end

            # Convergence file header
            if save_path!=nothing
                f = open(fname, "w")
                print(f, "age (deg),T,DT")
                for (i, rotor) in enumerate(rotors)
                    print(f, ",RPM_$i,CT_$i,CQ_$i,eta_$i")
                end
                print(f, "\n")
                close(f)
            end
        end

        # Write rotor position and time on convergence file
        if save_path!=nothing
            f = open(fname, "a")
            print(f, angle, ",", T, ",", DT)
        end


        # Plot circulation and loads distributions
        if disp_conv

            cratio = PFIELD.nt/nsteps
            cratio = cratio > 1 ? 1 : cratio
            clr = fcalls==0 && false ? (0,0,0) : (1-cratio, 0, cratio)
            stl = fcalls==0 && false ? "o" : "-"
            alpha = fcalls==0 && false ? 1 : 0.5

            # Circulation distribution
            plt.subplot(231)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, [vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B]...)
            end
            plt.plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Np distribution
            plt.subplot(232)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Np"]["field_data"]...)
            end
            plt.plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)

            # Tp distribution
            plt.subplot(233)
            this_sol = []
            for rotor in rotors
                this_sol = vcat(this_sol, rotor.sol["Tp"]["field_data"]...)
            end
            plt.plot(1:size(this_sol,1), this_sol, stl, alpha=alpha, color=clr)
        end

        # Plot performance parameters
        for (i,rotor) in enumerate(rotors)
            CT, CQ = vlm.calc_thrust_torque_coeffs(rotor, rho)
            eta = J*CT/(2*pi*CQ)

            if disp_conv
                plt.subplot(234)
                plt.plot([angle], [CT], "$(stls[i])", alpha=alpha, color=clr)
                plt.subplot(235)
                plt.plot([angle], [CQ], "$(stls[i])", alpha=alpha, color=clr)
                plt.subplot(236)
                plt.plot([angle], [eta], "$(stls[i])", alpha=alpha, color=clr)
            end

            if save_path!=nothing
                print(f, ",", rotor.RPM, ",", CT, ",", CQ, ",", eta)
            end
        end

        if disp_conv
            # Save figure
            if fcalls%nsteps_savefig==0 && fcalls!=0 && save_path!=nothing
                plt.savefig(joinpath(save_path, run_name*"_convergence.png"),
                                                            transparent=true)
            end
        end

        # Close convergence file
        if save_path!=nothing
            print(f, "\n")
            close(f)
        end

        fcalls += 1

        return false
    end

    return extra_runtime_function
end
