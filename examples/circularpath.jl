#=##############################################################################
# DESCRIPTION
Wing in a circular path with cross wind.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


# ------------ MODULES ---------------------------------------------------------
# Load simulation engine
# import FLOWFVS
reload("FLOWFVS")
fvs = FLOWFVS
vlm = fvs.vlm

import GeometricTools
gt = GeometricTools

using PyPlot

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"



# ------------ DRIVERS ---------------------------------------------------------
function run_circularpath()
    # circularpath(; nsteps=800, p_per_step=4, vlm_rlx=0.75,
    #                 save_path=extdrive_path*"circularpath10/",
    #                 verbose=true, disp_plot=true)
    circularpath(; nsteps=400, p_per_step=1, vlm_rlx=0.75,
                    save_path=extdrive_path*"circularpath10/",
                    verbose=true, disp_plot=true)
end


# ------------------------------------------------------------------------------

"""
    Test FLOWVLM solver on kinematics a wint in circular path with cross wind.
"""
function circularpath(;   # TEST OPTIONS
                        tol=0.025,
                        wake_coupled=true,
                        nsteps=150,
                        vlm_fsgm=-1,
                        p_per_step = 1,
                        vlm_rlx = -1,
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="bertins",
                        prompt=true,
                        verbose=true, verbose2=true, v_lvl=1,
                        disp_plot=true, figsize_factor=5/6
                        )

    if verbose; println("\t"^(v_lvl)*"Running Bertin's wing test..."); end;

    # ------------- GENERATE BERTIN'S WING -------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;
    # Experimental conditions
    magVinf = 12.0                  # (m/s) cross wind
    Rcirc = 40.0                    # (m) circling radius
    Mach = 0.2                      # Target Mach number
    speedofsound = 343              # (m/s) speed of sound
    wcirc = Mach*speedofsound/Rcirc # (rad/s) angular velocity of circular motion
    rhoinf = 9.093/10^1             # (kg/m^3) air density
    alpha = -4.2                     # (deg) angle of attack
    qinf = 0.5*rhoinf*magVinf^2     # (Pa) static pressure

    if verbose
        println("\t"^(v_lvl+1)*"RPM:\t\t\t$(round(60*wcirc/(2*pi), 1))")
        println("\t"^(v_lvl+1)*"Inflow AOA:\t\t$(round(180/pi*atan2(magVinf, wcirc*Rcirc), 1)) deg")
        println("\t"^(v_lvl+1)*"Effective AOA:\t\t$(round(180/pi*atan2(magVinf, wcirc*Rcirc)+alpha, 1)) deg")
    end

    # Geometry
    twist = 0.0                     # (deg) root twist
    lambda = 45.0                   # (deg) sweep
    gamma = 0.0                     # (deg) Dihedral
    b = 26.0/2                      # (m) span
    ar = 5.0                        # Aspect ratio
    tr = 1.0                        # Taper ratio

    # Discretization
    n = 4*2^4                       # Number of horseshoes
    r = 12.0                        # Geometric expansion
    central = false                 # Central expansion

    # Freestream function
    Vinf(X, t) = magVinf*[1,0,0]    # Cross wind in positive x-direction

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist, lambda, gamma;
                                                    n=n, r=r, central=central)

    # Pitch wing to corresponding angle of attack
    O = zeros(3)                    # Coordinate system origin
    Oaxis = gt.rotation_matrix2(0.0, -alpha, 0.0) # Coordinate system axes
    vlm.setcoordsystem(wing, O, Oaxis)


    # ------------- SIMULATION SETUP -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    revs = 3                    # Revolutions to solve
    lambda_vpm = 2.0            # target core overlap of vpm wake

    # Simulation options
    telapsed = revs/(wcirc/(2*pi)) # (s) total time to perform maneuver
    # nsteps = 2000             # Number of time steps
    Vcruise = wcirc*Rcirc       # (m/s) aircraft velocity during cruise
    RPMh_w = 0.0                # Rotor RPM during hover (dummy)

    # Solver options
    # p_per_step = 1              # Number of particle sheds per time steps
    overwrite_sigma = lambda_vpm * norm([magVinf, wcirc*Rcirc]) * (telapsed/nsteps)/p_per_step # Smoothing core size
    # vlm_sigma = -1            # VLM regularization core size (deactivated with -1)
    vlm_sigma = vlm_fsgm*b
    # wake_coupled = true       # Coupled VPM wake with VLM solution
    shed_unsteady = true        # Whether to shed unsteady-loading wake
    # shed_unsteady = false
    # vlm_rlx = -1                # VLM relaxation (deactivated with -1)

    # System definitions
    system = vlm.WingSystem()   # System of all FLOWVLM objects
    vlm.addwing(system, "BertinsWing", wing)

    # Aligns the wing with positive z-axis and perpendicular to cross wind
    O = zeros(3)
    Oaxis = [0 0 -1; 0 1 0; 1 0 0]
    vlm.setcoordsystem(system, O, Oaxis)

    vlm_system = system         # System solved through VLM solver
    wake_system = system        # System that will shed a VPM wake

    # Vehicle definition
    vehicle = fvs.VLMVehicle(   system;
                                vlm_system=vlm_system,
                                wake_system=wake_system
                             )

    if verbose
        println("\t"^(v_lvl+1)*"Core overlap:\t\t$(lambda_vpm)")
        println("\t"^(v_lvl+1)*"Core size:\t\t$(round(overwrite_sigma/b, 3))*b")
        println("\t"^(v_lvl+1)*"Time step translation:\t$(round(magVinf * (telapsed/nsteps)/b, 3))*b")
    end


    # ------------- MANEUVER DEFINITION  ---------------------------------------
    ncycles = revs              # Number of cycles in maneuver
    k = 2*pi*ncycles            # Frequency

    # Translational velocity of system over Vcruise
    function Vaircraft(t)
        # Counter-clockwise rotation
        Vx = 0
        Vy = sin(k*t)
        Vz = cos(k*t)
        return [Vx, Vy, Vz]
    end
    # Angle of the vehicle
    function angle_wing(t)
        return [0, 0, -360*k/(2*pi) * t]
    end
    angle = ()                  # Angle of each tilting system
    RPM = ()                    # RPM of each rotor system
    Vvehicle = Vaircraft        # Velocity of the vehicle
    anglevehicle = angle_wing   # Angle of the vehicle

    maneuver = fvs.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)
    # ------------- SIMULATION MONITOR -----------------------------------------
    y2b = 2*wing._ym/b

    # Weber's lift distribution data (Table 3)
    web_2yb = [0.0, 0.041, 0.082, 0.163, 0.245, 0.367, 0.510, 0.653, 0.898, 0.949]
    web_Cl = [0.235, 0.241, 0.248, 0.253, 0.251, 0.251, 0.251, 0.246, 0.192, 0.171]
    web_CL = 0.238
    web_ClCL = web_Cl/web_CL

    # Weber's drag distribution data (Table 3)
    web_Cd = [0.059, 0.025, 0.016, 0.009, 0.007, 0.006, 0.006, 0.004, -0.002, -0.007]
    web_CD = 0.005
    web_CdCD = web_Cd/web_CD

    prev_wing = nothing

    function monitor(sim, PFIELD, T, DT; figname="monitor_$(save_path)", nsteps_plot=1)

        aux = PFIELD.nt/nsteps
        clr = (1-aux, 0, aux)

        if PFIELD.nt==0 && disp_plot
            figure(figname, figsize=[7*2, 5*2]*figsize_factor)
            subplot(221)
            xlim([0,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cl}{CL}$")
            title("Spanwise lift distribution")

            subplot(222)
            xlim([0,1])
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"$\frac{Cd}{CD}$")
            title("Spanwise drag distribution")

            subplot(223)
            xlabel("Simulation time (s)")
            ylabel(L"Lift Coefficient $C_L$")

            subplot(224)
            xlabel("Simulation time (s)")
            ylabel(L"Drag Coefficient $C_D$")

            figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
            subplot(121)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Circulation $\Gamma$")
            subplot(122)
            xlabel(L"$\frac{2y}{b}$")
            ylabel(L"Effective velocity $V_\infty$")
        end

        if PFIELD.nt!=0 && PFIELD.nt%nsteps_plot==0 && disp_plot
            figure(figname)


            # Force at each VLM element
            Ftot = fvs.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                                            rhoinf; t=PFIELD.t)
            L, D, S = fvs.decompose(Ftot, [0,0,1], [-1,0,0])
            vlm._addsolution(wing, "L", L)
            vlm._addsolution(wing, "D", D)
            vlm._addsolution(wing, "S", S)

            # Force per unit span at each VLM element
            ftot = fvs.calc_aerodynamicforce(wing, prev_wing, PFIELD, Vinf, DT,
                                        rhoinf; t=PFIELD.t, per_unit_span=true)
            l, d, s = fvs.decompose(ftot, [0,0,1], [-1,0,0])

            # Lift of the wing
            Lwing = norm(sum(L))
            CLwing = Lwing/(qinf*b^2/ar)
            ClCL = norm.(l) / (Lwing/b)

            # Drag of the wing
            Dwing = norm(sum(D))
            CDwing = Dwing/(qinf*b^2/ar)
            CdCD = [sign(dot(this_d, [1,0,0])) for this_d in d].*norm.(d) / (Dwing/b) # Preserves the sign of drag

            vlm._addsolution(wing, "Cl/CL", ClCL)
            vlm._addsolution(wing, "Cd/CD", CdCD)

            subplot(221)
            plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
            plot(y2b, ClCL, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(222)
            plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
            plot(y2b, CdCD, "-", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(223)
            plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CLwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            subplot(224)
            plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
            plot([T], [CDwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

            figure(figname*"_2")
            subplot(121)
            plot(y2b, wing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
            if wake_coupled && PFIELD.nt!=0
                subplot(122)
                plot(y2b, norm.(wing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                plot(y2b, norm.(wing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                plot(y2b, [norm(Vinf(vlm.getControlPoint(wing, i), T)) for i in 1:vlm.get_m(wing)],
                                                            "-k", label="FLOWVLM", alpha=0.5)
            end
        end

        prev_wing = deepcopy(wing)

        return false
    end


    # ------------- RUN SIMULATION ---------------------------------------------
    # Simulation setup
    Vref = Vcruise                  # Reference velocity
    RPMref = RPMh_w                 # Reference RPM
    ttot = telapsed                 # Total time to perform maneuver
    Vinit = Vref*Vaircraft(0)       # Initial vehicle velocity
    Winit = pi/180*(angle_wing(1e-6) - angle_wing(0))/(1e-6*ttot)  # Initial angular velocity
                                    # Maximum number of particles
    max_particles = ceil(Int, (nsteps+2)*(2*vlm.get_m(vehicle.vlm_system)+1)*p_per_step)

    simulation = fvs.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                        Vinit=Vinit, Winit=Winit)

    if verbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;
    pfield = fvs.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      vlm_rlx=vlm_rlx,
                                      max_particles=max_particles,
                                      wake_coupled=wake_coupled,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose2, v_lvl=v_lvl+1,
                                      save_horseshoes=!wake_coupled
                                      )

    return simulation, pfield
end
