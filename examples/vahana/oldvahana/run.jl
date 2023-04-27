include("vahana2.jl")



############## Run simulation ##################################################
# run_simulation_vahana(; prompt=true)
################################################################################



this_folder_path = splitdir(splitdir(@__FILE__)[1])[1]
this_folder_name = splitdir(splitdir(@__FILE__)[1])[2]

this_file_path = splitdir(@__FILE__)[1]
this_file_name = splitext(splitdir(@__FILE__)[2])[1]

# Default path where to save data
extdrive_path  = this_folder_path*"/"

println("Number of threads: $(Threads.nthreads())")

# ------------ INPUTS AND OUTPUTS ----------------------------------------------
sim_name        = "vahana2-transition-101"      # Name of simulation to process
# sim_name        = "vahana2-transition-202"
# sim_name        = "vahana2-transition-105-test"


sim_name        = "run-"*sim_name
save_path       = extdrive_path*sim_name*"-postprocess100"

# nums            = 0:1600                          # Time steps to proces
# nums            = collect(0:30) .+ 830
# nums            = 500:899
# nums            = 275:780
# nums            = 3:36
# nums            = [1480, 1080, 700, 504]
# nums            = [1, 400, 800, 1020]
# nums            = [1, 400, 800, 1200, 1600, 2060]

nums            = collect(1000:4:2060)

reverse!(nums)

Rref            = 0.75                          # (m) reference blade radius


# OUTPUT OPTIONS
prompt          = true                          # Whether to promp the user
debug           = true                          # Debug mode
verbose         = true                          # Enable verbose
v_lvl           = 0                             # Verbose indentation level

# ------------ PROCESS SIMULATION ----------------------------------------------
read_path = joinpath(extdrive_path, sim_name)

# Calculate forces and moments
# postprocessing_forceandmoment(read_path, save_path*"-forcesandmoments00", nums;
#                                 prompt=prompt, debug=debug,
#                                 verbose=verbose, v_lvl=v_lvl)


# Calculate statistics of forces and moments
# postprocessing_statistics(read_path, save_path*"-statistics09", nums;
#                                 prompt=prompt, debug=debug,
#                                 verbose=verbose, v_lvl=v_lvl)





# Compute fluid domain
nthreads = 1
nthread = 1
dnum = floor(Int, length(nums)/nthreads)
threaded_nums = [view(nums, dnum*i+1:(i<nthreads-1 ? dnum*(i+1) : length(nums))) for i in 0:nthreads-1]

for these_nums in threaded_nums[nthread:nthread]

    this_save_path = save_path*"-fdom01"*(nthreads!=1 ? "-$nthread" : "")

    println("Starting postprocessing ($(this_save_path))...")

    postprocessing_fluiddomain(read_path, this_save_path, these_nums;
                                    # PROCESSING OPTIONS
                                    # fdx=1/160,                # Scaling of cell length
                                    # maxparticles=Int(30e6),
                                    fdx=1/140,
                                    maxparticles=Int(24e6),
                                    # fdx=1/120,
                                    # maxparticles=Int(20e6),
                                    # maxparticles=Int(24e6),
                                    # fdx=1/80,
                                    # maxparticles=Int(8e6),
                                    # maxparticles=Int(12e6),
                                    static_particles=true,
                                    # fmm=vpm.FMM(; p=4, ncrit=50, theta=0.4, phi=0.5), # FMM parameters
                                    fmm=vpm.FMM(; p=4, ncrit=50, theta=0.4, phi=0.3),
                                    f_sigma=0.5,                # Smoothing radii of node particles as sigma = f_sigma*meansigma
                                    # f_sigma=1.0,
                                    maxmagGamma=Inf,            # Any vortex strengths larger than this get clipped to this value
                                    # maxsigma=Inf,             # Any particles larger than this get shrunk to this value
                                    maxsigma=Rref/4,
                                    # SIMULATION INFORMATION
                                    ttot=30.0,
                                    tstart=0.20*30.0,           # Starting time of simulation
                                    nsteps=2*4*5400,
                                    start_kinmaneuver=false,
                                    # Vref=30.0,
                                    Vref=0.25 * 125*0.44704,
                                    # OUTPUT OPTIONS
                                    prompt=true)

    println("Fluid domain saved under $(this_save_path)")
end
