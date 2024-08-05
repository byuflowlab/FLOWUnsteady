using FLOWUnsteady

const vpm = FLOWUnsteady.vpm
gt = FLOWUnsteady.gt

vpm_path = normpath(joinpath(splitdir(pathof(vpm))[1], ".."))
include(joinpath(vpm_path, "examples", "utilities", "utilities_fluiddomain.jl"))

function calculate_fluid_domain(read_path, file_pref, static_pfield_pref, nums, ground_xR, save_path;
        n=20, B=2,
        nrevs = 10,
        nsteps_per_rev = 144,
        p_per_step = 1,
    )
    # size of fluid domain grid
    R = 0.12
    dx = R / 40
    x_max = ground_xR * R

    P_min = [-0.05,-0.15,-0.15]
    P_max = [x_max,0.15,0.15]
    NDIVS = Int.(ceil.((P_max .- P_min) ./ dx))
    
    # define origin function
    origin = (pfield, num) -> [0.0,0,0]
    
    # get max particles
    nsteps = nrevs * nsteps_per_rev
    max_particles = ((2*n+1)*B)*nsteps*p_per_step + 1
    max_particles += prod(NDIVS)

    # preprocess
    maxsigma = R/10
    maxmagGamma = Inf
    preprocessing_pfield = FLOWUnsteady.generate_preprocessing_fluiddomain_pfield(maxsigma, maxmagGamma;
                                                                         verbose=true, v_lvl=2)
    f_sigma = 0.5
    
    # compute fluid domain
    if !isdir(save_path)
        mkdir(save_path)
    end
    computefluiddomain(P_min, P_max, NDIVS, max_particles, nums, read_path, file_pref;
                       origin, verbose=true, save_path, userfunction_pfield=preprocessing_pfield, f_sigma,
                       other_file_prefs = [static_pfield_pref], other_read_paths = [read_path])
end
