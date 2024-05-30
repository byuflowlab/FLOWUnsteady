#=##############################################################################
# DESCRIPTION
    Actuator disk model, surface vortex sheet, and powered wake for
    aeropropulsive effects of embedded fans.

# ABOUT
  * Created   : May 2024
  * License   : MIT
=###############################################################################



################################################################################
# ACTUATOR DISK STRUCT
################################################################################
struct ActuatorDisk

end
# ----------------- End of ActuatorDisk ----------------------------------------




################################################################################
# SURFACE VORTEX SHEET STRUCT
################################################################################
mutable struct SurfaceVortexSheet

    # User inputs
    grid::gt.Grid                               # Surface grid
    gamma0::Float64                             # Starting strength of sheet
    gamma1::Float64                             # End strength of sheet
    distribution::Function                      # Strength distribution
    overlap::Float64                            # Overlap between particles
    sigma::Float64                              # Prescribed value of particle smoothing

    # Properties
    nparticles::Int                             # Number of particles in the sheet

end

function SurfaceVortexSheet(slices_path::String;
                            gamma1::Real=1.0, gamma0::Real=gamma1,
                            distribution=s->1.0,
                            overlap=1.25,
                            sigma=-1.0,
                            optargs...)

    grid = generate_svs_grid(slices_path; optargs...)
    nparticles = grid.ncells

    return SurfaceVortexSheet(grid,
                                gamma0, gamma1, distribution,
                                overlap, sigma,
                                nparticles)

end

"""
    `generate_svs_grid(slices_path;
                            offset=zeros(3),
                            rotation::Meshes.Rotation=one(Meshes.QuatRotation),
                            scaling=1.0,
                            file_pref="slice", file_ext=".vtu",
                            NDIVS_section=200, NDIVS_path=100,
                            verify_splines=true,
                            save_path=nothing,
                            paraview=false)`

Generate the surface grid of a surface vortex sheet by lofting a set of 3D VTK
slices.

# Arguments
* `slices_path`:        Path where to read slices from
* `file_pref`:          Prefix of slice files
* `file_ext`:           Extension of slice files
* `NDIVS_section`:      Number of divisions to split slice contours into
* `NDIVS_path`:         Number of divisions to split loft path into
"""
function generate_svs_grid(slices_path;
                            offset=zeros(3),
                            rotation::Meshes.Rotation=one(Meshes.QuatRotation),
                            scaling=1.0,
                            file_pref="slice", file_ext=".vtu",
                            NDIVS_section=200, NDIVS_path=100,
                            verify_splines=true,
                            save_path=nothing,
                            paraview=false)

    # ------------ Read surface sections ---------------------------------------

    # List all the slices to read
    files = [f for f in readdir(slices_path)
                    if contains(f, file_pref) && contains(f, file_ext)]

    if verify_splines
        fig = plt.figure(figsize=[7, 5]*5/9)
        ax = fig.gca()
        ax.set_title("Raw")
    end

    slices = []

    # Read all the slices
    for filename in files

        # Read VTK
        vtk = ReadVTK.VTKFile(joinpath(slices_path, filename))
        points = ReadVTK.get_points(vtk)
        cells = getproperty.(ReadVTK.to_meshcells(ReadVTK.get_cells(vtk)), :connectivity)

        # Remove repeated cells
        hashdict = SortedDict{Tuple{Int, Int}, Tuple{Int, Int}}()

        for (ci, cell) in enumerate(cells)
            hash = tuple(sort(cell)...)
            if !haskey(hashdict, hash)
                hashdict[hash] = (ci, length(hashdict)+1)
            end
        end

        cells = cells[getindex.(sort(collect(values(hashdict)), by=x->x[2]), 1)]

        # Remove unused points
        usedpoints = unique(vcat(cells...))
        points = points[:, usedpoints]

        # Map starting points to a cell index
        meshconnectivity1 = Dict((cell[1], ci) for (ci, cell) in enumerate(cells))

        # Map ending points to a cell index
        meshconnectivity2 = Dict((cell[2], ci) for (ci, cell) in enumerate(cells))

        # Sort points to be contiguous
        points_sorted = zeros(size(points, 1), size(points, 2)+1)

        pointi = 1
        counteri = 1
        while pointi != 1 || counteri == 1

            # Store this point
            points_sorted[:, counteri] .= points[:, pointi]

            # Find the cell that starts with this point
            celli = meshconnectivity1[pointi]

            # Case that unstructured grid is locally circular: shake the box
            if pointi == cells[meshconnectivity1[cells[celli][2]]][2]

                # Find the cell that end with this point
                celli = meshconnectivity2[pointi]

                # Fetch the point where the cell starts
                pointi = cells[celli][1]

            # Case that the unstructured grid is well connected
            else

                # Fetch the point where the cell ends
                pointi = cells[celli][2]

            end

            counteri += 1
        end

        # Close the contour
        points_sorted[:, end] .= points_sorted[:, 1]

        # Convert mesh to Meshes.jl object
        vertices = [tuple(point...) for point in eachcol(points_sorted)]
        connec = [Meshes.connect(tuple(cell...)) for cell in cells]
        msh = Meshes.SimpleMesh(vertices, connec)

        # Transform the original mesh: Translate, rotate, and scale
        msh = msh |> Meshes.Translate(offset...) |> Meshes.Rotate(rotation) |> Meshes.Scale(scaling)

        # Format transformed vertices into a matrix of points
        final_points = [v.coords[i] for i in 1:3, v in msh.vertices]

        if verify_splines
            ax.plot(final_points[2, :], final_points[3, :])
        end

        # Collect the slices
        push!(slices, final_points)

    end

    # Calculate the centroid of each slice
    centroids = [vcat(mean(slice[1, :]), gt.centroid(slice[2:3, :])...) for slice in slices]

    # Estimate the normal of each slice
    normals = [cross(slice[:, 1]-C, slice[:, 2]-C) for (slice, C) in zip(slices, centroids)]
    normals ./= norm.(normals)

    # Center the slices around the origin

    if verify_splines
        fig = plt.figure(figsize=[7, 5]*5/9)
        ax = fig.gca()
        ax.set_title("Centered")
    end

    for (centroid, slice) in zip(centroids, slices)

        slice .-= centroid

        if verify_splines
            ax.plot(slice[2, :], slice[3, :])
        end
    end

    # Define loft sections
    xmax = maximum(getindex.(centroids, 1))
    xmin = minimum(getindex.(centroids, 1))
    sections = [ # (non-dimensional arclength position along path, contour)
                    (
                        (centroid[1] - xmin)/(xmax - xmin),
                        collect(slice[2:3, :]')
                    ) for (centroid, slice) in zip(centroids, slices)
                ]

    # Sort sections by increasing x position
    sort!(sections, by=section->section[1])
    sort!(centroids, by=X->X[1])

    # @show [mean(getindex.(centroids, i)) for i in 1:3]


    # ------------ Define loft path ------------------------------------------------

    # Points along path
    path_Xs = deepcopy(centroids)

    # Section normals along path
    path_normals = deepcopy(normals)

    # Twist of section contours along path
    path_twists = [0 for X in path_Xs]

    # Collect the path
    path = collect(zip(path_Xs, path_normals, path_twists))


    # ------------ Generate lofted surface grid --------------------------------

    # Generate loft
    grid = gt.surface_pathloft(sections, path,
                                NDIVS_section, NDIVS_path;
                                sort_rediscretization=true,
                                output_path_normals=true,
                                output_path_xpos=true,
                                save_path=save_path, file_pref=file_pref*"-loft",
                                verify_spline=verify_splines,
                                paraview=paraview)

    return grid
end

"""
    `add_svs_static_particles!(pfield::vpm.ParticleField,
svs::SurfaceVortexSheet)`

Adds the surface vortex sheet `svs` as static particles to the field `pfield`.
"""
function add_static_particles!(pfield::vpm.ParticleField,
                                    svs::SurfaceVortexSheet; optargs...)

    return add_static_particles!(pfield,
                                    svs.grid,
                                    svs.distribution;
                                    gamma0=svs.gamma0, gamma1=svs.gamma1,
                                    overlap=svs.overlap,
                                    sigma=svs.sigma <= 0 ? nothing : svs.sigma,
                                    optargs...
                                    )

end

function add_static_particles!(pfield::vpm.ParticleField,
                                    grid, distribution;
                                    gamma1=1.0, gamma0=gamma1,
                                    overlap=1.25, sigma=nothing)

    # ------------ Convert loft surface into a particle field ------------------

    pathnormals = grid.field["pathnormal"]["field_data"]
    pathpositions = grid.field["pathposition"]["field_data"]
    cartind = CartesianIndices(grid._ndivscells[1:2])
    Ctot, tangent = (zeros(3) for i in 1:2)
    BmA, CmA, CmB, AmC, BmAcrossCmA = (zeros(3) for i in 1:5)
    Xc, normal = (zeros(3) for i in 1:2)
    t, X, Gamma = (zeros(3) for i in 1:3)

    for ci in 1:grid.ncells

        # Fetch node indices of this cell
        cell = gt.get_cell(grid, ci)

        # Restart memory
        areatot = 0
        Ctot .= 0
        tangent .= 0
        len = 0

        # Split non-planar polygon into triangles to approximate area, centroid, and tangent
        for ti in 0:length(cell)-3            # Iterate over triangles

            # Local node indices
            a, b, c = 1, ti+2, ti+3

            # Nodes of the triangle
            A = gt.get_node(grid, cell[a])
            B = gt.get_node(grid, cell[b])
            C = gt.get_node(grid, cell[c])

            # Precalculations
            map!(-, BmA, B, A)
            map!(-, CmA, C, A)
            map!(-, CmB, C, B)
            map!(-, AmC, A, C)
            cross!(BmAcrossCmA, BmA, CmA)

            # Area, centroid, and normal
            area = 0.5*norm(BmAcrossCmA)
            map!(+, Xc, A, B, C); Xc ./= 3
            normal .= BmAcrossCmA; normal ./= norm(normal)

            # Calculate path normal as the average of nodes
            pathnormal = mean( view(pathnormals, cell[[a, b, c]]) )
            pathnormal ./= norm(pathnormal)

            # Tangent vector
            cross!(t, pathnormal, normal)
            t ./= norm(t)

            # Cumulative values
            for i in 1:3; Ctot[i] += area*Xc[i]; end
            areatot += area
            tangent .+= t

        end

        Ctot /= areatot
        tangent /= norm(tangent)

        # Estimate transverse length of panel
        for ti in 0:length(cell)-3            # Iterate over triangles

            # Local node indices
            a, b, c = 1, ti+2, ti+3

            # Nodes of the triangle
            A = gt.get_node(grid, cell[a])
            B = gt.get_node(grid, cell[b])
            C = gt.get_node(grid, cell[c])

            # Precalculations
            map!(-, BmA, B, A)
            map!(-, CmB, C, B)
            map!(-, AmC, A, C)

            # Estimation
            len = max(len, abs(dot(BmA, tangent)), abs(dot(CmB, tangent)), abs(dot(AmC, tangent)))

        end

        # Calculate vortex sheet strength at this cell
        gamma = gamma0 + (gamma1 - gamma0) * distribution(mean(pathpositions[cell]))

        # Convert vortex sheet element into a vortex particle
        X .= Ctot
        Gamma .= tangent
        Gamma .*= gamma*areatot
        _sigma = !isnothing(sigma) ? sigma : overlap * sqrt(areatot)

        # Add particle
        vpm.add_particle(pfield, X, Gamma, _sigma;
                                static=true,
                                # Store the following data in place holders
                                circulation=gamma, vol=len)
    end

end

"""
Return the maximum number of static particles.
"""
_get_m_static(svs::SurfaceVortexSheet) = svs.grid.ncells
# ----------------- End of SurfaceVortexSheet ----------------------------------






################################################################################
# POWERED WAKE STRUCT
################################################################################
struct PoweredWake

end
# ----------------- End of PoweredWake -----------------------------------------







################################################################################
# PROPULSION STRUCT
################################################################################
"""
Definition of a propulsion system comprising of an array of actuator disks
(propulsors) and their associated surface vortex sheets and powered wake.
All actuator disks (propulsors) in a PropulsorSystem object share the same
control inputs.
"""
mutable struct PropulsionSystem
    actuatordisks::Vector{ActuatorDisk}
    surfacevortexsheets::Vector{SurfaceVortexSheet}
    poweredwakes::Vector{PoweredWake}
    target_deltaVjet::Float64

    function PropulsionSystem(actuatordisks, surfacevortexsheets, poweredwakes;
                                target_deltaVjet=1.0)

        return new(actuatordisks, surfacevortexsheets, poweredwakes, target_deltaVjet)

    end
end

function update(propulsion::PropulsionSystem, deltaVjet::Number)
    propulsion.target_deltaVjet = deltaVjet

    # TODO: Write a convergence algorithm here instead
    for svs in propulsion.surfacevortexsheets
        svs.gamma0 = deltaVjet
        svs.gamma1 = deltaVjet
    end

    return nothing
end


"""
Internal function for adding static particles of a PropulsionSystem to a
ParticleField.
"""
function _static_particles(pfield::vpm.ParticleField, propulsion::PropulsionSystem)

    # Add surface vortex sheets
    for surfacevortexsheet in propulsion.surfacevortexsheets
        add_static_particles!(pfield, surfacevortexsheet)
    end

    return nothing
end

"""
Return the maximum number of static particles.
"""
function _get_m_static(propulsion::PropulsionSystem)
    m = 0

    for surfacevortexsheet in propulsion.surfacevortexsheets
        m += _get_m_static(surfacevortexsheet)
    end

    return m
end
# ----------------- End of Propulsion ------------------------------------------
