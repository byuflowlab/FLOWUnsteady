#=##############################################################################
# DESCRIPTION
    Vehicle and components performance monitor.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Feb 2021
  * License   : MIT
=###############################################################################


function generate_monitor_vahana(vehicle, rho, RPMref, nsteps, save_path, Vinf)

    # Reference parameters for calculating coefficients
    # NOTE: make b, ar, and qinf equals to 1.0 to obtain dimensional force
    b_ref, ar_ref = 1.0, 1.0
    qinf = 1.0
    Jref = 1.0

    # Force axis labels
    CL_lbl = "Lift (N)"
    CD_lbl = "Induced drag (N)"

    # Directions of force components
    L_dir = [0, 0, 1]
    D_dir = [-1, 0, 0]


    # Rotor monitors
    rotors_monitors = []

    for (si, rotors) in enumerate(vehicle.rotor_systems)
        monitor_name = "rotorsys$(si)"
        rotors_monitor = uns.generate_monitor_rotors(rotors, Jref, rho, RPMref,
                                                        nsteps;
                                                        save_path=save_path,
                                                        run_name=monitor_name,
                                                        figname=monitor_name)
        push!(rotors_monitors, rotors_monitor)
    end

    # Main wing monitor
    monitor_name = "wing_main"
    mainwing_system = vlm.get_wing(vehicle.vlm_system, "MWing")
    mainwing_monitor = uns.generate_monitor_wing(mainwing_system, Vinf, b_ref, ar_ref,
                                                    rho, qinf, nsteps;
                                                    save_path=save_path,
                                                    run_name=monitor_name,
                                                    figname=monitor_name,
                                                    CL_lbl=CL_lbl,
                                                    CD_lbl=CD_lbl,
                                                    L_dir=L_dir,
                                                    D_dir=D_dir)

    # Tandem wing monitor
    monitor_name = "wing_tandem"
    tandemwing_system = vlm.get_wing(vehicle.vlm_system, "TWing")
    tandemwing_monitor = uns.generate_monitor_wing(tandemwing_system, Vinf, b_ref, ar_ref,
                                                    rho, qinf, nsteps;
                                                    save_path=save_path,
                                                    run_name=monitor_name,
                                                    figname=monitor_name,
                                                    CL_lbl=CL_lbl,
                                                    CD_lbl=CD_lbl,
                                                    L_dir=L_dir,
                                                    D_dir=D_dir)

    # State-variable monitor
    statevariable_monitor = uns.generate_monitor_statevariables(; save_path=save_path)

    # Stitch the simulation monitor together
    function monitor(args...)

        for rotors_monitor in rotors_monitors
            rotors_monitor(args...)
        end

        mainwing_monitor(args...)
        tandemwing_monitor(args...)
        statevariable_monitor(args...)

        return false
    end

    return monitor
end
