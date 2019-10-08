# Load simulation engine
include(joinpath(splitdir(@__FILE__)[1], "../src/fvs.jl"))

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
# @test bertin_VLM(; wake_coupled=false, nsteps=1, verbose=true)
# @test bertin_VLM(; wake_coupled=true, nsteps=200, verbose=true)
@test bertin_VLM(; wake_coupled=false, nsteps=200, save_path="temps/bertinswing/", verbose=true)
# @test bertin_VLM(; wake_coupled=true, nsteps=200, save_path="temps/bertinswing01/", verbose=true)



# TODO
# * Test VLM regularization
# * Test solution on kinematic velocity
# *
