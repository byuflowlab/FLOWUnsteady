include("rotorground_20240803_sc.jl")

#--- rotor in hover validation for DJI 4350 ---#

RPM = 6000.0
J = 0.0001
#for xR in (0.5, 1.0, 1.5, 2.0, 4.0, 7.0)
#end

    xR = 7.0 
    x_ground = xR
    run_rotorground(RPM,J;
            nrevs = 20,
            nsteps_per_rev = 144,
            run_name = "rotorground-20240803-isolated-ige-validation-NR7-xR$(xR)",      # Name of this simulation
            mirror          = true,
            x_ground        = x_ground,
            use_actuator_line = true,
            no_tip_correction = false,
            NR_truncate = 7,
            save_dir="/home/cibin/scratch"
        )
