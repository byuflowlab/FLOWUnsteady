abstract type Ground end

# struct Lattice <: Ground
#     gfield::ParticleGround
#     save_ground::Bool
#     run_name::Bool
#     save_path::String
#     update_A::Bool
# end

struct Mirror{TF} <: Ground
    ground_point::Array{TF,1}
    ground_normal::Array{TF,1}
end

struct Panel{TF} <: Ground
    ground_point::Vector{TF}
    ground_axes::Matrix{TF}
    Delta_x::TF
    Delta_y::TF
    nx::Int64
    ny::Int64
    kernel::PS.Kernel
    panel_shape::PS.PanelShape
end

for fname in ["panel","mirror"]
    include(joinpath(module_path,"FLOWUnsteady_ground_"*fname*".jl"))
end

# function generate_ground_effect_fun(pfield::vpm.ParticleField, ground_method::Lattice, save_ground, save_path, run_name)
#     gfield = ground_method.gfield
#     update_A = ground_method.update_A
#     ground_function(pfield, t, dt; kwargs...) = ground_effect!(pfield, gfield, dt, save_ground, run_name, save_path, update_A)
#     return ground_function
# end

function generate_ground_effect_fun(pfield::vpm.ParticleField, ground_method::Mirror, save_ground, save_path, run_name)
    ground_point = ground_method.ground_point
    ground_normal = ground_method.ground_normal
    ground_function(pfield, t, dt) = mirror_ground!(pfield, ground_point, ground_normal, save_ground, run_name, save_path)
    return ground_function
end

"Writes a function to update ground panel strengths and updates pfield.Uextra."
function generate_ground_effect_fun(pfield::vpm.ParticleField, ground_method::Panel, save_ground, save_path, run_name; save_time = false, optargs...)

    # extract user input
    ground_point = ground_method.ground_point
    ground_axes = ground_method.ground_axes
    Delta_x = ground_method.Delta_x
    nx = ground_method.nx
    Delta_y = ground_method.Delta_y
    ny = ground_method.ny
    kernel = ground_method.kernel
    panel_shape = ground_method.panel_shape

    # build grid centered about the origin
    parametric_function(t) = t * ground_axes[:,1] * Delta_x
    parametric = range(-0.5,stop=0.5,length=nx)
    dims = 3
    points = PS.curve(parametric_function, parametric, dims)
    extrude_vector = ground_axes[:,2] * Delta_y
    extrude_sections = range(-0.5,stop=0.5,length=ny)
    grid = PS._extrude(points, extrude_vector, extrude_sections)

    # translate to point
    grid .+= ground_point

    # build panels
    panels = PS.Panels(grid; kernel, panel_shape)

    # get panel collection path
    panel_collection = joinpath(save_path, run_name*"_panels")

    # build PanelGround
    gfield = PanelGround(panels, panel_collection)

    # build ground function
    ground_function(pfield, t, dt) =
        ground_effect!(pfield, gfield, dt, save_ground, run_name, save_path, save_time)

    # set pfield.Uextra
    Uextra(X) = PS.v_induced(panels, kernel, panel_shape, X)
    pfield.Uextra = Uextra

    return ground_function
end
