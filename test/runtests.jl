using Pkg
this_dir = @__DIR__
Pkg.activate(normpath(joinpath(this_dir,"..")))

using FLOWUnsteady
import FLOWUnsteady.vlm as vlm
using FLOWUnsteady.StaticArrays
using FLOWUnsteady.LinearAlgebra
using Test

@testset "quaternions" begin

v = SVector{3}(1.0,0,0)
axis = SVector{3}(0,0,1.0)
angle = pi/6

v_rotated_sameframe = rotate(v, axis, angle)
v_notrotated_newframe = rotate_frame(v, axis, angle)

@test isapprox(v_rotated_sameframe, SVector{3}(cos(angle),sin(angle),0.0); atol=1e-12)
@test isapprox(v_notrotated_newframe, SVector{3}(cos(-angle),sin(-angle),0.0); atol=1e-12)

# test rotate_frame on chained quaternions

q1 = Quaternion(SVector{3}(1.0,0,0), pi/7)
q2 = Quaternion(SVector{3}(0,0,1.0), pi/5)
theta = 2*pi/3
phi = 11*pi/12
q3 = Quaternion(SVector{3}(sin(theta)*cos(phi),sin(theta)*sin(phi),cos(theta)), pi/3)

v_b = SVector{3}(1.0,2.0,3.0)

q12 = q1*q2
q23 = q2*q3
q123 = q1*q2*q3

v_i_a = rotate_frame(rotate_frame(rotate_frame(v_b,q1),q2),q3)
v_i_b = rotate_frame(rotate_frame(v_b,q12),q3)
v_i_c = rotate_frame(rotate_frame(v_b,q1),q23)
v_i_d = rotate_frame(v_b, q123)

@test isapprox(v_i_a, v_i_b; atol=1e-12)
@test isapprox(v_i_c, v_i_b; atol=1e-12)
@test isapprox(v_i_c, v_i_d; atol=1e-12)

# test rotate on chained quaternions
q21 = q2*q1
q32 = q3*q2
q321 = q3*q2*q1

v_i_w = rotate(rotate(rotate(v_b,q1),q2),q3)
v_i_x = rotate(rotate(v_b,q21),q3)
v_i_y = rotate(rotate(v_b,q1),q32)
v_i_z = rotate(v_b,q321)

@test isapprox(v_i_w, v_i_x; atol=1e-12)
@test isapprox(v_i_w, v_i_y; atol=1e-12)
@test isapprox(v_i_w, v_i_z; atol=1e-12)

# test specific reference frame
# DynamicState objects hold a quaternion
q = Quaternion(SVector{3}(0,0,1.0),pi/2) # means the parent (new) frame is rotated 90 degrees about the body (old) frame z axis
v_old_frame = SVector{3}(1.0,0,0)
v_new_frame = rotate_frame(v_old_frame, q)

@test isapprox(v_new_frame, SVector{3}(0,-1.0,0); atol=1e-12)

end

@testset "reference frames" begin

# create a rigid body state at [1,0,0] with its coordinate system rotated by -pi/6 about the z axis by the RHR
angle = pi/6
state = RigidBodyState("full vehicle", 2, DynamicState(Float64; position=SVector{3}(1.0,0,0), orientation=Quaternion(SVector{3}(0,0,1.0), angle)))

# get the rotation from body frame to inertial frame
q_b2i = quaternion_frame_2_top(state, ())

# try rotating [1,0,0] in the body frame to the inertial frame
v_b = SVector{3}(1.0,0,0)
v_i = rotate_frame(v_b, q_b2i)

@test isapprox(v_i, SVector{3}(cos(angle), -sin(angle), 0.0); atol=1e-12)

# transform vector [0,0,1] in the body coordinate system to the inertial frame
v_b = SVector{3}(0,0,1.0)
v_i = transform_parent_2_top(v_b, state, (1,))

@test isapprox(v_i, SVector{3}(1,0,1.0); atol=1e-12)

# transform vector [1,0,1] in the body coordinate system to the inertial frame
v_b = SVector{3}(1.0,0,1.0)
v_i = transform_parent_2_top(v_b, state, (1,))

@test isapprox(v_i, SVector{3}(1 + cos(angle), -sin(angle), 1.0); atol=1e-12)

# add a nested substate
new_frame = RigidBodyState("subframe", 2, DynamicState(Float64; position=SVector{3}(0,1.0,1.0), orientation=Quaternion(SVector{3}(1.0,0,0), pi/12)))
add_substate!(state, (), new_frame)

# add another substate
new_frame = RigidBodyState("subframe2", 1, DynamicState(Float64; position=SVector{3}(0.0,0,0), orientation=Quaternion(SVector{3}(0,0,1.0),-angle)))
add_substate!(state, (), new_frame)

# transform origin from the substate to the inertial frame
v_b = SVector{3}(0,0,0.0)
v_i = transform_parent_2_top(v_b, state, (1,1))

@test isapprox(v_i, SVector{3}(1+cos(pi/3),sin(pi/3),1.0); atol=1e-12)

# transform vector from substate to the inertial frame
v_b = SVector{3}(0,0,1.0)
v_i = transform_parent_2_top(v_b, state, (1,1))

@test isapprox(v_i, SVector{3}(sin(pi/12)*sin(pi/6), sin(pi/12)*cos(pi/6), cos(pi/12)) + SVector{3}(1+cos(pi/3),sin(pi/3),1); atol=1e-12)

# rotate vector from substate2 to inertial frame (shouldn't change at all)
v_b = SVector{3}(0.1,0.2,0.3)
q_b2i = quaternion_frame_2_top(state,(2,))
v_i = rotate_frame(v_b, q_b2i)

@test isapprox(v_i, v_b; atol=1e-12)

# rotate vector from substate to inertial frame
v_b = SVector{3}(0.0,0,1.0)
q_b2i = quaternion_frame_2_top(state, (1,))
v_i = rotate_frame(v_b, q_b2i)

@test isapprox(v_i, SVector{3}(sin(pi/6)*sin(pi/12),cos(pi/6)*sin(pi/12),cos(pi/12)); atol=1e-12)

# check quaternion multiplication
q2 = state.state.orientation
q1 = state.substates[1].state.orientation
v_i_test = rotate_frame(rotate_frame(v_b,q1),q2)

@test isapprox(v_i_test, SVector{3}(sin(pi/6)*sin(pi/12),cos(pi/6)*sin(pi/12),cos(pi/12)); atol=1e-12)

# add another nested substate
new_frame = RigidBodyState("subframe", 2, DynamicState(Float64; position=SVector{3}(2.0,4.0,-3.0), orientation=Quaternion(SVector{3}(0.4,1.0,2.0)/norm([0.4,1,2.0]), -3*pi/13)))
add_substate!(state, (1,), new_frame)

# rotate vector from the new substate to inertial frame
v_b = SVector{3}(0.1,0.2,0.3)
q_b2i = quaternion_frame_2_top(state,(1,1))
v_i = rotate_frame(v_b, q_b2i)
v_i_test = rotate_frame(rotate_frame(rotate_frame(v_b, state.substates[1].substates[1].state.orientation), state.substates[1].state.orientation), state.state.orientation)

@test isapprox(v_i, v_i_test; atol=1e-12)

#--- test `transform_2_top` ---#

state_arr = Array{RigidBodyState{Float64,Int},0}(undef)
state_arr[] = RigidBodyState("frame 1", 1, DynamicState(Float64; position=SVector{3}(1.0,0,0), angular_velocity=SVector{3}(0,1.0,0)))

vb = SVector{3}(1,2,3.0)
vi1 = FLOWUnsteady.transform_parent_2_top(vb, state_arr[], ())
vi2 = FLOWUnsteady.transform_parent_2_top(vb, state_arr[], (1,))

@test isapprox(vb, vi1; atol=1e-12)
@test isapprox(vb+SVector{3}(1.0,0,0), vi2; atol=1e-12)

new_frame = RigidBodyState("subframe", 2, DynamicState(Float64; angular_velocity=SVector{3}(0,1.0,0), position=SVector{3}(0.0,0,0), orientation=Quaternion(SVector{3}(1.0,0,0), pi/2)))
add_substate!(state_arr[], (), new_frame)

vb = SVector{3}(0,0,1.0)
vi3 = FLOWUnsteady.transform_parent_2_top(vb, state_arr[], (1,1))

@test isapprox(SVector{3}(1.0,1.0,0), vi3; atol=1e-12)

#--- rotate a reference frame ---#

# visualize all frames first
# visualize(state_arr[]; name_prefix="test_")

# now lets say we have a very simple model containing a single vector at a point
struct SimpleModel{TF} <: AbstractModel{TF,false}
    location::Vector{SVector{3,TF}}
    orientation::Vector{SVector{3,TF}}
end
sm = SimpleModel([SVector{3}(1.0,1.0,0), SVector{3}(0,0,0.0)], [SVector{3}(0.0,1.0,0.0), SVector{3}(0,0,1.0)])

function FLOWUnsteady.transform!(sm::SimpleModel, map::Int, translation, rotation, center_of_rotation) # overload transform! for the test
    # rotate first
    sm.location[map] = rotate(sm.location[map] - center_of_rotation, rotation) + center_of_rotation

    # now translate
    sm.location[map] = sm.location[map] + translation
end

# now use `transform!` to rotate our simple model about [1,0,0] about axis [1,0,0] by pi/2 to obtain [1,0,1]
state_arr = Array{RigidBodyState{Float64,Int},0}(undef)
state_arr[] = RigidBodyState("frame 1", 1, DynamicState(Float64; position=SVector{3}(1.0,0,0), angular_velocity=SVector{3}(1.0,0,0)))
dt = pi/2
FLOWUnsteady.transform!(sm, state_arr, dt)

@test isapprox(sm.location[1], SVector{3}(1.0,0,1.0); atol=1e-12)

# now add another nested frame at a new location
state_arr = Array{RigidBodyState{Float64,Int},0}(undef)
state_arr[] = RigidBodyState("frame 1", 1, DynamicState(Float64; position=SVector{3}(1.0,0,0)))
new_frame = RigidBodyState("subframe", 2, DynamicState(Float64; angular_velocity=SVector{3}(0,1.0,0), position=SVector{3}(0.0,0,0), orientation=Quaternion(SVector{3}(1.0,0,0), pi/2)))
add_substate!(state_arr[], (), new_frame)

new_frame = RigidBodyState("subframe 2", 3, DynamicState(; orientation=Quaternion(SVector{3}(1.0,0,0.0), pi/2), angular_velocity=SVector{3}(0,1.0,0), position=SVector{3}(0,1.0,0)))
add_substate!(state_arr, (1,), new_frame)
push!(sm.location, SVector{3}(0.0,0,1))

# perform manual rotation
point = SVector{3}(0,0,1.0)
origin = FLOWUnsteady.transform_2_top(SVector{3}(0,1.0,0), state_arr, (1,))
origin_manual = rotate_frame(SVector{3}(0,1.0,0), state_arr[].state.orientation) + state_arr[].state.position

@test isapprox(origin, origin_manual; atol=1e-12)

ω = SVector{3}(0,1.0,0)
ω_topframe = rotate_frame(ω, state_arr[].state.orientation)

new_point_manual = rotate(point - origin, ω, pi/2) + origin

FLOWUnsteady.transform!(sm, state_arr, dt)

@test isapprox(sm.location[3], SVector{3}(2.0, 0.0, 1.0); atol=1e-12)

end

function build_vlm(;
        Quasisteady=false,
        orientation=Quaternion(SVector{3}(1.0,0,0),0),
        velocity=SVector{3,Float64}(10.0,0,0),
        dynamic=false,
        max_timesteps=10,
        shed_starting=true,
        model_kwargs...
    )

    # Unsteady Wing and Tail
    ns_multiplier = 1
    nc_multiplier = 1
    time_step = 0.05 # in [6.4, 3.2, 1.6, 0.8, 0.4, 0.2, 0.1, 0.05]

    # wing
    xle = [0.0, 0.2]
    yle = [0.0, 5.0]
    zle = [0.0, 1.0]
    chord = [1.0, 0.6]
    theta = [2.0*pi/180, 2.0*pi/180]
    phi = [0.0, 0.0]
    ns = Int(12 * ns_multiplier)
    nc = 1 * nc_multiplier
    spacing_s = vlm.Uniform()
    spacing_c = vlm.Uniform()
    mirror = true

    # horizontal stabilizer
    xle_h = [0.0, 0.14]
    yle_h = [0.0, 1.25]
    zle_h = [0.0, 0.0]
    chord_h = [0.7, 0.42]
    theta_h = [0.0, 0.0]
    phi_h = [0.0, 0.0]
    ns_h = Int(6 * ns_multiplier)
    nc_h = 1 * nc_multiplier
    spacing_s_h = vlm.Uniform()
    spacing_c_h = vlm.Uniform()
    mirror_h = true

    # vertical stabilizer
    xle_v = [0.0, 0.14]
    yle_v = [0.0, 0.0]
    zle_v = [0.0, 1.0]
    chord_v = [0.7, 0.42]
    theta_v = [0.0, 0.0]
    phi_v = [0.0, 0.0]
    ns_v = 5 * ns_multiplier
    nc_v = 1 * nc_multiplier
    spacing_s_v = vlm.Uniform()
    spacing_c_v = vlm.Uniform()
    mirror_v = false

    # adjust chord lengths to match AVL (which uses chord length in the x-direction)
    chord = @. chord/cos(theta)
    chord_h = @. chord_h/cos(theta_h)
    chord_v = @. chord_v/cos(theta_v)

    Sref = 9.0
    cref = 0.9
    bref = 10.0
    rref = [0.5, 0.0, 0.0]
    Vinf = 1.0
    ref = vlm.Reference(Sref, cref, bref, rref, Vinf)

    alpha = 5.0*pi/180
    beta = 0.0
    Omega = [0.0; 0.0; 0.0]
    fs = vlm.Freestream(Vinf, alpha, beta, Omega)

    symmetric = [false, false, false]

    # horseshoe vortices
    wgrid, wing = vlm.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
        mirror=mirror, spacing_s=spacing_s, spacing_c=spacing_c)

    hgrid, htail = vlm.wing_to_surface_panels(xle_h, yle_h, zle_h, chord_h, theta_h, phi_h, ns_h, nc_h;
        mirror=mirror_h, spacing_s=spacing_s_h, spacing_c=spacing_c_h)
    vlm.translate!(htail, [4.0, 0.0, 0.0])

    vgrid, vtail = vlm.wing_to_surface_panels(xle_v, yle_v, zle_v, chord_v, theta_v, phi_v, ns_v, nc_v;
        mirror=mirror_v, spacing_s=spacing_s_v, spacing_c=spacing_c_v)
    vlm.translate!(vtail, [4.0, 0.0, 0.0])

    surfaces = [wing, htail, vtail]
    surface_id = [1, 2, 3]

    system = vlm.System(Float64, surfaces)
    for i in eachindex(surfaces)
        system.surfaces[i] .= surfaces[i]
    end

    for properties in system.properties
        for i in eachindex(properties)
            properties[i] = eltype(properties)(0.0, zero(SVector{3,Float64}), zero(SVector{3,Float64}), zero(SVector{3,Float64}), zero(SVector{3,Float64}))
        end
    end
    for dproperties in system.dproperties
        for properties in dproperties
            for i in eachindex(properties)
                properties[i] = eltype(properties)(0.0, zero(SVector{3,Float64}), zero(SVector{3,Float64}), zero(SVector{3,Float64}), zero(SVector{3,Float64}))
            end
        end
    end

    model = VortexLatticeModel(system; Quasisteady, max_timesteps, shed_starting, model_kwargs...)
    vehicle = RigidBodyVehicle(model; dynamic, orientation, velocity, model_coordinates=Aerodynamics(), vehicle_coordinates=FlightDynamics())

    return vehicle
end

function create_sim(alpha=5.0*pi/180;
        dynamic=false,
        orientation=Quaternion(SVector{3}(1.0,0,0),0),
        velocity=SVector{3,Float64}(0.0,0,0),
        time_range=range(0,stop=1.0,length=101),
        freestream=SimpleFreestream(10.0*SVector{3}(-cos(alpha),0,-sin(alpha))),
        controller=PrescribedKinematics(),
        integrator=ForwardEuler(),
        preprocessor=DefaultPreprocessor(),
        model_kwargs...
    )

    save_steps = 0:length(time_range)-1

    # create vehicle
    vehicle = build_vlm(; dynamic, orientation, velocity, max_timesteps=length(time_range), model_kwargs...)
    history = History(vehicle, controller, save_steps)
    paraview = ParaviewOutput(save_steps)
    postprocessor = MultiPostprocessor((history, paraview))

    # create simulation object
    sim = Simulation(vehicle, time_range;
            freestream, controller, preprocessor, integrator, postprocessor
        )
    return sim, time_range
end

model_kwargs = ()#(threshold_unsteady_gamma_max=0.0,)
sim, time_range = create_sim(;
        time_range=range(0,stop=1.0,length=401), model_kwargs...
    )

simulate!(sim, time_range; run_name="test_20240524_2_unsteady_pedrizzetti", path="test_20240524_2_unsteady_pedrizzetti")
