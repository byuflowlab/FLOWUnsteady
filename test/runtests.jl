# Load simulation engine
# import FLOWFVS
reload("FLOWFVS")
fvs = FLOWFVS
vlm = fvs.vlm
gt = vlm.vtk

# ------------ GENERIC MODULES -------------------------------------------------
using Base.Test
using PyPlot
using JLD

# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["bertinswing"]
    include("test_"*module_name*".jl")
end

# ------------ TESTS -----------------------------------------------------------

# Test VLM solver: Isolated wing
@test bertin_VLM(; wake_coupled=false, nsteps=0, verbose=true, disp_plot=true)

# Test VLM regularization: Isolated wing
@test bertin_VLM(; wake_coupled=false, vlm_fsgm=0.00125, nsteps=0, verbose=true, disp_plot=true)

# Test VPM+VLM coupling: Isolated wing
@test bertin_VLM(; wake_coupled=true, nsteps=150, verbose=true, disp_plot=true)

# Test VPM+VLM solver on kinematic velocity: Isolated wing
@test bertin_kinematic(; nsteps=150, verbose=true, disp_plot=true)

# Test simulation stability: Isolated wing with a very fine time stepping and shedding
@test bertin_kinematic(; nsteps=300, p_per_step=4, vlm_rlx=0.75, verbose=true, disp_plot=true)


# @test bertin_kinematic(; nsteps=150, p_per_step=4, vlm_rlx=0.75, save_path="temps/bertinswing11/", verbose=true, disp_plot=true)
