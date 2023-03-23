using Test

using PyPlot
using Printf

import FLOWUnsteady

# Alises
const uns = FLOWUnsteady
const vlm = uns.vlm
const gt  = uns.gt

# Headers
for header_name in ["sweptwing"]
    include("test_"*header_name*".jl")
end


# ------------ TESTS -----------------------------------------------------------
@testset verbose=true "Isolated wing" begin

    name = "Test VLM solver — Isolated wing"
    @test run_sweptwing(name; wake_coupled=false, nsteps=3)

    name = "Test VLM regularization — Isolated wing"
    @test run_sweptwing(name; wake_coupled=false, fsgm_vlm=0.00125, nsteps=3)

    name = "Test VPM+VLM coupling — Isolated wing"
    @test run_sweptwing(name)

    name = "Test kinematic velocity — Isolated wing"
    @test run_sweptwing(name; kinematic=true)

    # name = "Test simulation stability — Isolated wing with fine time stepping and shedding"
    # @test run_sweptwing(name; nsteps=300, p_per_step=4, vlm_rlx=0.75,
    #                     simverbose=true, disp_plot=true, save_path="sweptwing00")

    name = "Test quasi-steady solver — Isolated wing"
    @test run_sweptwing(name; VehicleType=uns.QVLMVehicle, nsteps=10)

end
