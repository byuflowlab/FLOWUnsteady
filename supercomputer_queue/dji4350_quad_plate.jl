include("quadrotor_20240803_sc.jl")

J = 0.0001
RPM = 6000.0

#--- add plate to prevent fountain flow ---#
# l/R = 2.7, h/R = 1.5
run_quadrotor(RPM,J;  # Cibin: Changed name
        nrevs = 30,
        nsteps_per_rev = 144,
        run_name = "quadrotor-20240803-hover-validation-lR27_hR15-plate",      # Name of this simulation
        mirror          = true,
        x_ground        = 1.5,
        lR = 2.1,
        use_actuator_line = true,
        no_tip_correction = false,
        plate = true
    )

#--- finish rotor in hover validation for DJI 4350 ---#
# l/R = 2.1, h/R = 1.5
run_quadrotor(RPM,J;  # Cibin: Changed name
        nrevs = 30,
        nsteps_per_rev = 144,
        run_name = "quadrotor-20240803-hover-validation-lR21_hR15",      # Name of this simulation
        mirror          = true,
        x_ground        = 1.5,
        lR = 2.1,
        use_actuator_line = true,
        no_tip_correction = false
    )
