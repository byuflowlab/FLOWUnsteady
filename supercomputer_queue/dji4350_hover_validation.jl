include("rotorground_20240803_sc.jl")

#--- rotor in hover validation for DJI 4350 ---#

J = 0.0001
for RPM in range(3000, stop=7000, length=5)
    run_rotorground(RPM,J;
            nrevs = 10,
            nsteps_per_rev = 144,
            run_name = "rotorground-20240803-hover-validation-rpm$(Int(round(RPM)))",      # Name of this simulation
            mirror          = false,
            x_ground        = 1,
            use_actuator_line = true,
            no_tip_correction = false,
            save_dir="~/scratch"
        )
end
