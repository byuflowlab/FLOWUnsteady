using Pkg
this_dir = @__DIR__
Pkg.activate(normpath(joinpath(this_dir,"..")))

using FLOWUnsteady
using FLOWUnsteady.StaticArrays
vlm = FLOWUnsteady.vlm

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
        mirror=true, spacing_s=vlm.Sine(), spacing_c=vlm.Uniform())

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
        save_steps=collect(range(0,length(time_range)+1)),
        omit_fields=(),
        model_kwargs...
    )

    # simulation parameters
    Re = 1.6e6
    vinf = 163 * 0.3048 # m/s
    freestream=SimpleFreestream(vinf * SVector{3}(-cos(alpha),0,sin(alpha))) # in North-East-down coordinates

    # construct vehicle
    vlm_system = construct_weber_vlm_system(; ns, nc)
    model = VortexLatticeModel(vlm_system; max_timesteps=length(time_range), model_kwargs...)
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

omit_fields = (:state,)
#model_kwargs = (threshold_unsteady_gamma_max=0.0, overlap_trailing=0.4, FMM=false)
model_kwargs = (overlap_trailing=0.4, FMM=false)
time_range = range(0,step=0.001,length=101)
alpha = 4.2*pi/180
run_name = "weber_4_2_20240528_2_new_shedding_unsteady"
path = "weber_4_2_20240528_2_new_shedding_unsteady"
sim = run_simulation(time_range, alpha, run_name, path; omit_fields, model_kwargs...)

nothing
