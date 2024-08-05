include("rotorground_fluiddomain_20240805.jl")

read_path = "~/scratch/rotorground-20240803-hover-validation-rpm6000"
file_pref = "rotorground-20240803-hover-validation-rpm6000_pfield" 
static_pfield_pref = "rotorground-20240803-hover-validation-rpm6000_staticpfield"
nums = 1081:1440
ground_xR = 
save_path = joinpath(read_path, "fluid_domain")

calculate_fluid_domain(read_path, file_pref, static_pfield_pref, nums, ground_xR, save_path;
    n=40, B=2,
    nrevs = 10,
    nsteps_per_rev = 144,
    p_per_step = 1,
)
