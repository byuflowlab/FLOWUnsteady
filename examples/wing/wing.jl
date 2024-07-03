#=##############################################################################
# DESCRIPTION
    45° swept-back wing at an angle of attack of 4.2°. This wing has an aspect
    ratio of 5.0, a RAE 101 airfoil section with 12% thickness, and no dihedral,
    twist, nor taper. This test case matches the experimental setup of Weber,
    J., and Brebner, G., “Low-Speed Tests on 45-deg Swept-Back Wings, Part I,”
    Tech. rep., 1951. The same case is used in a VLM calculation in Bertin's
    Aerodynamics for Engineers, Example 7.2, pp. 343.

# AUTHORSHIP
  * Author          : Eduardo J. Alvarez (edoalvarez.com)
  * Email           : Edo.AlvarezR@gmail.com
  * Created         : Feb 2023
  * Last updated    : Feb 2023
  * License         : MIT

Rewritten by Benjamin Varela in June 2024 for integration into version 5.0.0 of FLOWUnsteady
using weber validaiton
=###############################################################################

using Pkg
this_dir = @__DIR__
FLOWUnsteady_dir = normpath(joinpath(this_dir,"../.."))
Pkg.activate(FLOWUnsteady_dir)

using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm

using Plots

# construct VLM system (see VortexLattice.jl for info on VLM systems)
function construct_vlm_system(; ns=5, nc=5)
    c0 = 20*0.0254
    halfspan = 44.0/0.9 * 0.0254
    yle = [0.0, halfspan]
    xle = [0.0, halfspan]
    zle = [0.0, 0.0]
    chord = [c0, c0]
    theta = [0.0, 0.0]
    phi = [0.0, 0.0]

    grid, surface = vlm.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
            mirror=true, spacing_s=vlm.Sine(), spacing_c=vlm.Uniform(), fcore=(c,Δs) -> 0.001
        )

    surfaces = [surface]
    ref = vlm.Reference(1.0, 1.0, 1.0, SVector{3}(halfspan/2+c0/2, 0.0, 0.0), 1.0)
    fs = vlm.Freestream(0.0, 0.0, 0.0, SVector{3}(0.0,0.0,0.0), 1.0)

    system = vlm.steady_analysis(surfaces, ref, fs; symmetric=false)
    system.trailing_vortices .= false

    return system
end

function construct_simulation(time_range)
    alpha = deg2rad(4.2) # angle of attack
    ρ = 1.0 # air density
    vinf = 163 * 0.3048 # magnitude of freestream velocity
    velocity = zero(SVector{3,Float64}) # vehicle velocity
    save_steps=collect(range(0,length(time_range)-1)) # which steps to save info on
    dynamic = false # Do not use forces to determine states or motion

    # simulation parameters
    freestream = SimpleFreestream(vinf * SVector{3}(-cos(alpha),0,-sin(alpha)), ρ) # Freestream velocity and density

    # construct vehicle
    vlm_system = construct_vlm_system()
    model = VortexLatticeModel(vlm_system; max_timesteps=length(time_range), max_particles_override=50000, FMM=false) # at the time of this writing, FMM is not working but the default is true
    vehicle = RigidBodyVehicle(model; dynamic, velocity, model_coordinates=Aerodynamics(), vehicle_coordinates=FlightDynamics())
    controller = PrescribedKinematics()

    # postprocessor
    history = History(vehicle, controller, save_steps)
    paraview = ParaviewOutput(save_steps)
    postprocessor = MultiPostprocessor((history, paraview))

    # create simulation object
    sim = Simulation(vehicle, time_range;
        freestream, postprocessor)

    return sim
end

function run_simulation(time_range, run_name, path)
    sim = construct_simulation(time_range)
    simulate!(sim, time_range; run_name, path)
    return sim
end

function get_lift(history, alpha;
        rho = 1.0, vinf = 163 * 0.3048, c=20 * 0.0254, b=98*0.0254
    )
    salpha, calpha = sincos(alpha)
    R = FLOWUnsteady.StaticArrays.SMatrix{3,3,Float64,9}(calpha,0,-salpha,0,1,0,salpha,0,calpha)

    CL = zeros(size(history.history[:vehicle_force],2))
    CD = zeros(size(history.history[:vehicle_force],2))
    for i in eachindex(CL)
        F = R * history.history[:vehicle_force][:,i]
        F = F .* SVector{3}(-1,1,-1)
        CF = F ./ (0.5*rho*vinf^2 * c * b)
        CL[i] = CF[3]
        CD[i] = CF[1]
    end

    # overall forces
    # F = R * history.history[:vehicle_force][:,end]
    # F = F .* SVector{3}(-1,1,-1)
    # CF = F ./ (0.5*rho*vinf^2 * c * b)

    return CL, CD
end

save_path = normpath(joinpath(this_dir,"results"))
!isdir(save_path) && mkdir(save_path)
# sim = run_simulation(range(0.0,stop=0.1,length=2), "wing", save_path);
sim = run_simulation(range(0.0,step=0.001,length=151), "wing", save_path);

CL, CD = get_lift(sim.postprocessor.postprocessors[1], deg2rad(4.2))

plot(CL)

# # Where to save output data (default to re-generating files used in the docs)
# outdata_path = joinpath(this_dir, "../..", "docs", "resources", "data")

# # Post-process monitor plots
# include(joinpath(uns.examples_path, "wing", "wing_postprocessing.jl"))


# # --------- Save simulation outputs
# if save_outputs
#     figs[1].savefig(joinpath(fig_path, run_name*"-simmonitor.png"),
#                                                 dpi=300, transparent=true)

#     str = """
#     |           | Experimental  | FLOWUnsteady              | Error |
#     | --------: | :-----------: | :-----------------------: | :---- |
#     | \$C_L\$   | 0.238         | $(round(CL, digits=5))    | $(round(CLerr*100, digits=3))% |
#     | \$C_D\$   | 0.005         | $(round(CD, digits=5))    | $(round(CDerr*100, digits=3))% |
#     """

#     open(joinpath(outdata_path, run_name*"-CLCD.md"), "w") do f
#         println(f, str)
#     end
# end


# # ----------------- ANGLE OF ATTACK SWEEP --------------------------------------
# sweep_aoa = !true                       # Whether to run AOA sweep

# if sweep_aoa
#     println("Running AOA sweep...")
#     include(joinpath(uns.examples_path, "wing", "wing_aoasweep.jl"))
# end
