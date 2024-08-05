include("quadrotor_20240803_sc.jl")

#--- rotor in hover validation for DJI 4350 ---#

J = 0.0001
RPM = 6000.0

# l/R = 2.7, h/R = 1.5
run_quadrotor(RPM,J;  # Cibin: Changed name
        nrevs = 25,
        nsteps_per_rev = 144,
        run_name = "quadrotor-20240803-hover-validation-nrevs25_NR7_lR27_hR15", # Name of this simulation
        mirror          = true,
        x_ground        = 1.5,
        lR = 2.7,
        use_actuator_line = true,
        no_tip_correction = false,
        NR_truncate = 7,
        save_dir="~/scratch"
    )
