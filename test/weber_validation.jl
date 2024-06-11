using Pkg
this_dir = @__DIR__
FLOWUnsteady_dir = normpath(joinpath(this_dir,".."))
Pkg.activate(FLOWUnsteady_dir)

using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm
using PythonPlot

# construct VLM system
function construct_weber_vlm_system(; ns=20, nc=1)
    function chord(y2b)
        c0 = 20 * 0.0254
        A = 0.8 * c / 0.01
        if y2b < 0.9
            c = c0
        else
            c = A * y2b^2
        end
        return c
    end

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

function construct_weber_simulation(time_range, alpha;
        preprocessor=DefaultPreprocessor(),
        integrator=ForwardEuler(),
        ns=20, nc=1, dynamic=false,
        orientation=zero(Quaternion{Float64}),
        velocity=zero(SVector{3,Float64}),
        save_steps=collect(range(0,length(time_range)-1)),
        omit_fields=(),
        vinf = 163 * 0.3048, # m/s
        Quasisteady=false,
        model_kwargs...
    )

    # simulation parameters
    Re = 1.6e6
    freestream=SimpleFreestream(vinf * SVector{3}(-cos(alpha),0,-sin(alpha))) # in North-East-down coordinates

    # construct vehicle
    vlm_system = construct_weber_vlm_system(; ns, nc)
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
    sim = construct_weber_simulation(time_range, alpha; kwargs...)
    simulate!(sim, time_range; run_name, path)
    return sim
end

function get_lift_distribution(path, run_name, i_step, alpha;
        rho = 1.0, vinf = 163 * 0.3048, c=20 * 0.0254, b=98*0.0254
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
    F = R * history.history[:vehicle_force][:,i_step]
    F = F .* SVector{3}(-1,1,-1)
    CF = F ./ (0.5*rho*vinf^2 * c * b)

    # return y_force, y2b_force, f, cf, F, CF
    return F, CF
end

function validate_weber(save_path;
        run_simulations=true,
        # situation-specific arguments
        omit_fields = (:state,),
        model_kwargs = (threshold_unsteady_gamma_max=0.0, overlap_trailing=0.4, FMM=false),
        #model_kwargs = (overlap_trailing=0.4, FMM=false),
        time_range = range(0,step=0.001,length=101),
        vinf = 163 * 0.3048 # m/s
    )

    # run simulations to validate several alphas
    alpha_range = [2.1, 4.2, 6.3, 8.4, 10.5] .* pi/180
    if run_simulations
        for (i,alpha) in enumerate(alpha_range)
            run_name = "weber_validation_$(i)_alpha"*string(Int(round(alpha*180/pi; sigdigits=1)))
            path = joinpath(save_path,run_name)
            !isdir(path) && mkdir(path)
            sim = run_simulation(time_range, alpha, run_name, path; omit_fields, vinf, model_kwargs...)
        end
    end

    # validation data
    CL = [0.121, 0.238, 0.350, 0.456, 0.559]
    CD = [NaN, 0.005, 0.012, 0.022, 0.035]
    y2b = [0.0, 0.041, 0.082, 0.163, 0.245, 0.367, 0.510, 0.653, 0.898, 0.949]
    cl = [
    	0.118 0.121 0.126 0.129 0.129 0.129 0.131 0.125 NaN 0.087;
    	0.235 0.241 0.248 0.253 0.251 0.251 0.251 0.246 0.192 0.171;
    	0.351 0.358 0.367 0.374 0.375 0.373 0.377 0.365 NaN 0.256;
    	0.466 0.476 0.483 0.494 0.494 0.493 0.493 0.480 NaN 0.340;
    	0.577 0.589 0.597 0.607 0.611 0.605 0.599 0.587 0.415 0.401;
    ]
    cd = [
    	0.044 0.014 0.007 0.002 0.000 -0.001 0.000 -0.001 -0.009 -0.010;
    	0.047 0.016 0.010 0.004 0.002 0.001 0.002 0.001 NaN -0.009;
    	0.059 0.025 0.016 0.009 0.007 0.006 0.006 0.004 -0.002 -0.007;
    	0.078 0.039 0.028 0.017 0.015 0.012 0.012 0.009 NaN -0.005;
    	0.104 0.058 0.045 0.029 0.026 0.023 0.022 0.016 NaN -0.001;
    	0.138 0.084 0.065 0.044 0.041 0.037 0.035 0.026 0.009 0.004;
    ]

    # make plots
    CL_uns = zeros(length(alpha_range))
    CD_uns = zeros(length(alpha_range))

    for (i,alpha) in enumerate(alpha_range)
        # load files
        run_name = "weber_validation_$(i)_alpha"*string(Int(round(alpha*180/pi; sigdigits=1)))
        path = joinpath(save_path,run_name)

        # extract forces at the final timestep
        F, CF = get_lift_distribution(path, run_name, length(time_range), alpha; rho=1.0)
        # y_force, y2b_force, f, cf, F, CF = get_lift_distribution(path, run_name, length(time_range), alpha; rho=1.0)

        # # plot comparison
        # fig = figure("lift_distribution")
        # fig.clear()
        # fig.add_subplot(211, ylabel="sectional lift coefficient")
        # fig.add_subplot(212, ylabel="sectional drag coefficient", xlabel="nondimensional spanwise coordinate")
        # axs = fig.get_axes()
        # n_half = length(y2b_force) >> 1
        # axs[0].plot(y2b_force[n_half+1:end], cf[3,n_half+1:end])
        # axs[1].plot(y2b_force[n_half+1:end], cf[1,n_half+1:end])
        # axs[0].scatter(y2b, cl[i,:])
        # axs[1].scatter(y2b, cd[i,:])
        # legend(["FLOWUnsteady v5.0", "experiment"])

        # # save
        # savefig(joinpath(save_path, run_name*".png"))

        # extract vehicle wide force coefficients
        CL_uns[i] = CF[3]
        CD_uns[i] = CF[1]
    end

    # integral quantities
    fig = figure("lift_polar")
    fig.clear()
    fig.add_subplot(211, ylabel="lift coefficient")
    fig.add_subplot(212, ylabel="drag coefficient", xlabel="angle of attack, degrees")
    axs = fig.get_axes()
    axs[0].plot(alpha_range .* 180/pi, CL_uns)
    axs[0].scatter(alpha_range .* 180/pi, CL)
    axs[1].plot(alpha_range .* 180/pi, CD_uns)
    axs[1].scatter(alpha_range .* 180/pi, CD)
    legend(["FLOWUnsteady v5.0", "experiment"])

    # save
    savefig(joinpath(save_path, "weber_validation.png"))

end


save_path = "/Users/ryan/Dropbox/research/projects/FLOWUnsteady/20240605_weber_validation_unsteady"
!isdir(save_path) && mkdir(save_path)
model_kwargs = (eta_wake=1.0, overlap_trailing=0.4, FMM=false, Quasisteady=false, ns=10,nc=5, p_per_step_trailing=1, p_per_step_unsteady=1, threshold_unsteady_gamma_min=0.001)
# validate_weber(save_path; time_range=range(0.0,stop=0.1,length=2), run_simulations=true, model_kwargs)
validate_weber(save_path; time_range=range(0.0,step=0.001,length=151), run_simulations=true, model_kwargs)

# time_range = range(0,step=0.001,length=101)
# alpha = 2.1*pi/180
# run_name = "20240605_compare_unsteady"
# path = run_name
# omit_fields = (:state,)
# vinf = 163*0.3048
# ns = 2
# nc = 2
# model_kwargs = (eta_wake=1.0,overlap_trailing=0.4,FMM=false, Quasisteady=false, ns,nc, threshold_unsteady_gamma_max=0.0)
# sim = run_simulation(time_range, alpha, run_name, path; omit_fields, vinf, model_kwargs...)
# sys_uns = sim.vehicle.model.vlm_system
#
# run_name = "20240605_compare_steady"
# path = run_name
# model_kwargs = (eta_wake=1.0,overlap_trailing=0.4,FMM=false, Quasisteady=true, ns,nc)
# sim_steady = run_simulation(time_range, alpha, run_name, path; omit_fields, vinf, model_kwargs...)
# sys_steady = sim_steady.vehicle.model.vlm_system

# #####
# ##### vvv future unit test vvv
# #####
#
# time_range = range(0,step=0.001,length=2)
# alpha = 2.1*pi/180
# run_name = "20240601_compare_quasisteady"
# path = run_name
# omit_fields = (:state,)
# vinf = 163*0.3048
# ns = 1
# nc = 1
# model_kwargs = (eta_wake=1.0,overlap_trailing=0.4,FMM=false,Quasisteady=true,ns,nc)
# sim = run_simulation(time_range, alpha, run_name, path; omit_fields, vinf, model_kwargs...)
#
# sys_uns = sim.vehicle.model.vlm_system
# CF_uns, CM_uns = vlm.body_forces(sys_uns; frame=vlm.Body())
#
#
# #####
# ##### steady analysis
# #####
#
# c = 20*0.0254
# b = 44.0/0.9 * 0.0254 * 2
# S = c * b
# V = 163 * 0.3048
# system = construct_weber_vlm_system(; ns, nc)
# ref = vlm.Reference(S, c, b, SVector{3}(-0.4,0,0), V)
# # ref = vlm.Reference(2.0, 1.0, 1.0, SVector{3}(-0.0,0,0), 1.0)
# fs = vlm.Freestream(163*0.3048, 2.1*pi/180, 0.0,SVector{3}(0.0,0,0),1.0)
# solved_system = vlm.steady_analysis(system.surfaces, ref, fs; symmetric=false, xhat=SVector{3}(cos(alpha),0.0,sin(alpha)), derivatives=false)
# CF, CM = vlm.body_forces(solved_system; frame=vlm.Wind())
# CF_body, CM_body = vlm.body_forces(solved_system; frame=vlm.Body())
# # CF_new, CM_new = vlm.body_forces(solved_system.filaments, solved_system.filament_properties, solved_system.surfaces, solved_system.reference[], solved_system.freestream[], [false], vlm.Body(); convention_change=[-1.0,1.0,-1.0])
# CF_new, CM_new = vlm.body_forces(solved_system.surfaces, solved_system.properties, solved_system.reference[], solved_system.freestream[], [false], vlm.Body(); convention_change=[-1.0,1.0,-1.0])
# properties = vlm.get_surface_properties(solved_system)
# vlm.write_vtk("20240601_weber_validation_quasisteady/alpha2", solved_system.surfaces, properties; symmetric=false)
# F = CF * 0.5 * 1.0 * V^2 * S
