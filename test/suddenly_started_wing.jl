using Pkg
this_dir = @__DIR__
FLOWUnsteady_dir = normpath(joinpath(this_dir,".."))
Pkg.activate(FLOWUnsteady_dir)

using LaTeXStrings
using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm
using PythonPlot

# construct VLM system
function construct_rectangular_wing(AR; ns, nc)

    S = 1.0
    c = sqrt(S / AR)
    b = S/c

    yle = [0.0, b/2]
    xle = [0.0, 0.0]
    zle = [0.0, 0.0]
    chord = [c, c]
    theta = [0.0, 0.0]
    phi = [0.0, 0.0]

    grid, surface = vlm.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
            mirror=true, spacing_s=vlm.Sine(), spacing_c=vlm.Uniform(), fcore=(c,Δs) -> 0.001
        )

    surfaces = [surface]
    ref = vlm.Reference(1.0, 1.0, 1.0, SVector{3}(c/2, 0.0, 0.0), 1.0)
    fs = vlm.Freestream(0.0, 0.0, 0.0, SVector{3}(0.0,0.0,0.0), 1.0)

    system = vlm.steady_analysis(surfaces, ref, fs; symmetric=false)
    system.trailing_vortices .= false

    return system
end

function construct_simulation(time_range, alpha;
        preprocessor=DefaultPreprocessor(),
        integrator=ForwardEuler(),
        AR=20, ns=20, nc=1, dynamic=false,
        orientation=zero(Quaternion{Float64}),
        velocity=zero(SVector{3,Float64}),
        save_steps=collect(range(0,length(time_range)-1)),
        omit_fields=(:state,),
        vinf = 10.0, # m/s
        Quasisteady=false,
        model_kwargs...
    )

    # simulation parameters
    freestream=SimpleFreestream(vinf * SVector{3}(-cos(alpha),0,-sin(alpha))) # in North-East-down coordinates

    # construct vehicle
    vlm_system = construct_rectangular_wing(AR; ns, nc)
    model = VortexLatticeModel(vlm_system; Quasisteady, max_timesteps=length(time_range), model_kwargs...)
    vehicle = RigidBodyVehicle(model; dynamic, orientation, velocity, model_coordinates=Aerodynamics(), vehicle_coordinates=FlightDynamics())
    controller = PrescribedKinematics()

    # postprocessor
    history = History(vehicle, controller, save_steps; omit_fields)
    paraview = ParaviewOutput(save_steps)
    postprocessor = MultiPostprocessor((history, paraview))

    # create simulation object
    sim = Simulation(vehicle, time_range;
        freestream, controller, preprocessor, integrator, postprocessor
    )

    return sim
end

function run_simulation(time_range, alpha, run_name, path; kwargs...)
    sim = construct_simulation(time_range, alpha; kwargs...)
    simulate!(sim, time_range; run_name, path)
    return sim
end

function get_lift(path, run_name, i_step, alpha;
        rho = 1.0, vinf = 10.0, S, AR
    )
    FLOWUnsteady.BSON.@load joinpath(FLOWUnsteady_dir,path,run_name*"_history.bson") history
    # force_distribution = history.history[:surface_force][1][:,1,:,i_step]
    # force_rotated = deepcopy(force_distribution)
    salpha, calpha = sincos(alpha)
    R = FLOWUnsteady.StaticArrays.SMatrix{3,3,Float64,9}(calpha,0,-salpha,0,1,0,salpha,0,calpha)
    # for i in 1:size(force_rotated,2)
    #     force_rotated[:,i] .= R * force_rotated[:,i]
    # end
    # force_rotated[1,:] .*= -1
    # force_rotated[3,:] .*= -1


    # # get spanwise coordinate
    # x_distribution_force = history.history[:surface_x][1][:,:,i_step]
    # y_force = zeros(ns*2+1)
    # i_x = 1
    # i_y = 0
    # for _ in 1:ns
    #     i_y += 1
    #     y_force[i_y] = x_distribution_force[2,i_x+1]
    #     i_y += 1
    #     y_force[i_y] = x_distribution_force[2,i_x]
    #     i_x += 2*nc
    # end
    # y_force[end] = x_distribution_force[2,end]

    # # get force
    # Δx = [x_endpoints[i+1]-x_endpoints[i] for i in eachindex(y_force)]
    # b = 2*history.history[:surface_x_vertical][1][2,1,end,i_step]
    # y2b_force = y_force ./ (b/2)
    # f = zeros(3,length(y2b_force))
    # for i in 1:3
    #     f[i,:] .= force_rotated[i,:] ./ Δx
    # end
    # cf = f ./ (0.5*rho*vinf^2 * c)

    # overall forces
    F_history = deepcopy(history.history[:vehicle_force])
    for i in 1:size(F_history,2)
        F_history[:,i] .= (R * F_history[:,i]) .* SVector{3}(-1,1,-1)
    end
    CF_history = F_history ./ (0.5*rho*vinf^2 * S)

    # return y_force, y2b_force, f, cf, F, CF
    return F_history, CF_history
end

function validate_suddenly_started_wing(save_path;
        run_simulations=true,
        # situation-specific arguments
        omit_fields = (:state,),
        model_kwargs = (threshold_unsteady_gamma_max=Inf, overlap_trailing=0.4, FMM=false),
        #model_kwargs = (overlap_trailing=0.4, FMM=false),
        time_range = range(0,step=0.001,length=101),
        alpha = 5*pi/180,
        S=1.0, AR=20,
        vinf = 10.0 # m/s
    )

    # run simulations to validate several alphas
    if run_simulations
        run_name = "validate_suddenly_started_wing"
        path = save_path
        !isdir(path) && mkdir(path)
        sim = run_simulation(time_range, alpha, run_name, path; omit_fields, vinf, model_kwargs...)
    end

    # load files
    run_name = "validate_suddenly_started_wing"
    path = save_path

    # extract forces at the final timestep
    F, CF = get_lift(path, run_name, length(time_range), alpha; rho=1.0, vinf, S, AR)

    # integral quantities
    c = sqrt(S / AR)
    b = S/c
    fig = figure("lift_history")
    fig.clear()
    fig.add_subplot(211, ylabel="lift coefficient")
    fig.add_subplot(212, ylabel="drag coefficient", xlabel="nondimensional time, "*L"\frac{U_{\infty} \Delta t}{c}")
    axs = fig.get_axes()
    axs[0].plot(time_range .* vinf / c, CF[3,1:length(time_range)])
    axs[0].set_ylim([0,0.6])
    axs[1].plot(time_range .* vinf / c, CF[1,1:length(time_range)])
    axs[1].set_ylim([0,0.03])
    legend(["FLOWUnsteady v5.0"])

    # save
    savefig(joinpath(save_path, "suddenly_started_wing.png"))

end


save_path = "/Users/ryan/Dropbox/research/projects/FLOWUnsteady/20240605_ssw_validation_ns13_nc1_smaller_unsteady_particles_4"
!isdir(save_path) && mkdir(save_path)
# model_kwargs = (eta_wake=1.0, overlap_trailing=0.4, FMM=false, multipole_threshold=0.0, max_particles_override=70000, Quasisteady=false, ns=13,nc=4, p_per_step_trailing=1, p_per_step_unsteady=1)# , threshold_unsteady_gamma_min=0.0)
model_kwargs = (eta_wake=1.0, overlap_trailing=0.4, FMM=false, multipole_threshold=0.0, Quasisteady=false, ns=13, nc=1, p_per_step_trailing=1, p_per_step_unsteady=1, threshold_unsteady_gamma_min=0.0, max_particles_override=70000)
# validate_suddenly_started_wing(save_path; time_range=range(0.0,stop=0.1,length=2), run_simulations=true, model_kwargs)
validate_suddenly_started_wing(save_path; time_range=range(0.0,step=0.001,length=135), run_simulations=false, model_kwargs)

