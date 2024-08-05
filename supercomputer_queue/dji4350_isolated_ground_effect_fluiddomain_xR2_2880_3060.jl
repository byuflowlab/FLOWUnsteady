include("rotorground_fluiddomain_20240805.jl")

read_path = "~/scratch/rotorground-20240803-isolated-ige-validation-NR7-xR2.0"
file_pref = "rotorground-20240803-isolated-ige-validation-NR7-xR2.0_pfield"
static_pfield_pref = "rotorground-20240803-isolated-ige-validation-NR7-xR2.0_staticpfield"
nums = 2880:3060
ground_xR = 2
save_path = joinpath(read_path, "fluid_domain")

calculate_fluid_domain(read_path, file_pref, static_pfield_pref, nums, ground_xR, save_path;
    n=40, B=2,
    nrevs = 25,
    nsteps_per_rev = 144,
    p_per_step = 1,
)
