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
    flip::Bool                                  # Flip the vortex sheet strength
    overlap::Float64                            # Overlap between particles
    sigma::Float64                              # Prescribed value of particle smoothing

    # Properties
    nparticles::Int                             # Number of particles in the sheet

end

function SurfaceVortexSheet(slices_path::String;
                            gamma1::Real=1.0, gamma0::Real=gamma1,
                            distribution=s->1.0,
                            flip=false,
                            overlap=1.25,
                            sigma=-1.0,
                            optargs...)

    grid = generate_svs_grid(slices_path; optargs...)
    nparticles = grid.ncells

    return SurfaceVortexSheet(grid,
                                gamma0, gamma1, distribution, flip,
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
                            # offsetcentroids=(slice, closed)->zeros(3),
                            offsetcentroids=nothing,
                            sortingfunction=nothing,
                            removeduplicatenodes=false,
                            forceclosure=false,
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

        if removeduplicatenodes
            # Make sure that in duplicate nodes only one is used and the other remains unused
            # to ensure that resulting lines are recognized as being continuous
            distance = (i, j) -> norm(points[:, i] - points[:, j])
            twins = Dict( (i, (j for j in 1:size(points, 2) if distance(i, j) <= 1e-10 )) for i in 1:size(points, 2)) # List of twins for each node
            cells = [ [first(twins[ni]) for ni in cell] for cell in cells] # For duplicated nodes make sure that only one is used
        end

        # Remove unused points
        usedpoints = unique(vcat(cells...))
        toremove = [i for i in 1:size(points, 2) if !(i in usedpoints)]
        points = points[:, usedpoints]

        # Redefine cell connectivity with new points definition
        for rni in reverse(toremove)
            for cell in cells
                for (i, ni) in enumerate(cell)
                    if rni<ni
                        cell[i] -= 1
                    end
                end
            end
        end

        # Map starting points to a cell index
        meshconnectivity1 = Dict((cell[1], ci) for (ci, cell) in enumerate(cells))

        # Map ending points to a cell index
        meshconnectivity2 = Dict((cell[2], ci) for (ci, cell) in enumerate(cells))

        # Check if the slice is a closed contour
        closed = true
        startpointi = 1
        npoints = size(points, 2)

        for pi in keys(meshconnectivity1)
            # Case that a starting point is no one's ending point
            if !haskey(meshconnectivity2, pi)
                closed = false
                startpointi = pi
                break
            end
        end

        # Sort points to be contiguous
        points_sorted = zeros(size(points, 1), npoints+1*closed)

        pointi = startpointi
        counteri = 1
        while pointi != startpointi || counteri == 1

            # Store this point
            points_sorted[:, counteri] .= points[:, pointi]

            # Exit criterion if slice contour is open and encountered last point
            if !closed && counteri==npoints
                break
            end

            # Find the cell that starts with this point
            celli = meshconnectivity1[pointi]

            # Case that unstructured grid is locally circular: shake the box
            if (closed || counteri!=npoints-1) && (cells[celli][2] in keys(meshconnectivity1) && pointi == cells[meshconnectivity1[ cells[celli][2] ]][2])

                # Find the cell that ends with this point
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

        # Force closure if requested
        if forceclosure && !closed
            points_sorted = hcat(points_sorted, points_sorted[:, 1])
            closed = true
        end

        # Close the contour
        if closed
            points_sorted[:, end] .= points_sorted[:, 1]
        end

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
        push!(slices, (final_points, closed))

    end

    # Calculate the centroid of each slice
    if !isnothing(offsetcentroids)
        centroids = [vcat(
                            mean(slice[1, :]),
                            (closed ? gt.centroid(slice[2:3, :]) : (mean(slice[2, :]), mean(slice[3, :])) )...
                        ) for (slice, closed) in slices ]
        # Offset centroids
        centroids .+= [offsetcentroids(slice, closed) for (slice, closed) in slices]
    else
        centroids = [gt.centroid3D(slice) for (slice, closed) in slices]
    end

    # Sort points in each slice
    if !isnothing(sortingfunction)

        newslices = []

        for (si, (points, closed)) in enumerate(slices)

            # Remove repeated nodes
            distance = (i, j) -> norm(points[:, i] - points[:, j])
            to_remove = []

            for i in 1:size(points, 2)

                if i in to_remove
                    nothing
                else
                    for j in i+1:size(points, 2)
                        if distance(i, j) <= 1e3*eps()
                            push!(to_remove, j)
                        end
                    end
                end

            end

            to_keep = [i for i in 1:size(points, 2) if !(i in to_remove)]
            newpoints = points[:, to_keep]

            # Sort slice
            newpoints .= sortslices(newpoints; dims=2, by=X->sortingfunction(X-centroids[si]))

            # Close contour once again
            if closed
                newpoints = hcat(newpoints, newpoints[:, 1])
            end

            push!(newslices, (newpoints, closed))

        end

        slices = newslices

    end

    # Define the tangent unit vector of each slice
    tangents = [slice[:, 1]-C for ((slice, closed), C) in zip(slices, centroids)]
    tangents ./= norm.(tangents)

    # Estimate the normal of each slice
    normals = [cross(tangent, slice[:, 2]-C) for ((slice, closed), C, tangent) in zip(slices, centroids, tangents)]
    normals ./= norm.(normals)

    # Define the oblique unit vector of each slice
    obliques = [cross(normal, tangent) for (normal, tangent) in zip(normals, tangents)]
    obliques ./= norm.(obliques)

    # Center the slices around the origin
    if verify_splines
        fig = plt.figure(figsize=[7, 5]*5/9)
        ax = fig.gca()
        ax.set_title("Centered")
    end

    for (centroid, (slice, closed)) in zip(centroids, slices)

        slice .-= centroid

        if verify_splines
            ax.plot(slice[2, :], slice[3, :])
        end
    end

    # Define loft sections projected onto the unit vector bases
    xmax = maximum(getindex.(centroids, 1))
    xmin = minimum(getindex.(centroids, 1))
    if !isnothing(offsetcentroids)
        sections = [ # (non-dimensional arclength position along path, contour)
                        (
                            (centroid[1] - xmin)/(xmax - xmin),
                            collect(slice[2:3, :]')
                        ) for (centroid, (slice, closed)) in zip(centroids, slices)
                    ]
    else
        sections = [ # (non-dimensional arclength position along path, contour)
                        (
                            (centroid[1] - xmin)/(xmax - xmin),
                            collect(hcat([ [dot(point, t), dot(point, o)] for point in eachcol(slice) ]...)')
                        ) for (centroid, (slice, closed), t, o) in zip(centroids, slices, tangents, obliques)
                    ]
    end

    # Sort sections by increasing x position
    sort!(sections, by=section->section[1])
    sort!(centroids, by=X->X[1])

    # @show [mean(getindex.(centroids, i)) for i in 1:3]


    # ------------ Define loft path --------------------------------------------

    # Points along path
    path_Xs = deepcopy(centroids)

    # Section normals along path
    path_normals = deepcopy(normals)

    # Twist of section contours along path
    # path_twists = [0.0 for X in path_Xs]
    path_twists = [-atand(dot(tangent, [0, 0, 1]), dot(tangent, [0, 1, 0])) for tangent in tangents]

    # Collect the path
    path = collect(zip(path_Xs, path_normals, path_twists))


    # ------------ Pre-processing of open sections -----------------------------
    for (si, (centroid, section)) in enumerate(sections)

        closed = slices[si][2]

        if !closed

            # @show "pre", section[1, :]
            # @show atand(section[1, 1], section[1, 2])
            # @show atand(section[2, 1], section[2, 2])

            # Calculate twist of section relative to first point
            twist = atand(section[1, 1], section[1, 2]) - 179.9999

            # Untwist the section
            for i in 1:size(section, 1)
                y = section[i, 1]
                z = section[i, 2]

                y, z = y*cosd(-twist) + z*sind(-twist), -y*sind(-twist) + z*cosd(-twist)

                section[i, 1] = y
                section[i, 2] = z
            end

            # @show "post", section[1, :]
            # @show atand(section[1, 1], section[1, 2])
            # @show atand(section[2, 1], section[2, 2])

            # Twist the path to undo the untwisting later on
            path[si] = (path[si][1], path[si][2], path[si][3] + twist)

        end

    end


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
                                    svs.distribution,
                                    svs.flip;
                                    gamma0=svs.gamma0, gamma1=svs.gamma1,
                                    overlap=svs.overlap,
                                    sigma=svs.sigma <= 0 ? nothing : svs.sigma,
                                    optargs...
                                    )

end

function add_static_particles!(pfield::vpm.ParticleField,
                                    grid, distribution, flip;
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

        # Flip the vortex sheet strength
        gamma *= (-1)^flip

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
mutable struct PoweredWake

    # Required user inputs
    sheddingline::Matrix                        # Shedding line

    # Optional user inputs
    gamma::Float64                              # Vortex sheet strength
    overlap::Float64                            # Overlap between particles
    sigma::Float64                              # Prescribed value of particle smoothing

    # Properties
    nparticles::Int

    # Constructors
    PoweredWake(
                sheddingline;
                nparticles=size(sheddingline, 2)-1,
                gamma=0.0,
                overlap=2.125,
                sigma=-1.0
                ) = new(
                sheddingline,
                gamma, overlap, sigma,
                nparticles
                )

end


function PoweredWake(filename::String, args...; optargs...)

    sheddingline = generate_pw_line(filename, args...; optargs...)

    return PoweredWake(sheddingline)

end



function generate_pw_line(filename::String, reader::Function, npoints::Int;
                            read_path="",
                            closed=false,
                            sortingfunction=pnl.direction([0, 1, 0]),
                            junctioncriterion=pnl.nojunction,
                            offset=zeros(3),
                            rotation::Meshes.Rotation=one(Meshes.QuatRotation),
                            scaling=1.0,
                            verify_splines=true,
                            )

    # Read trailing edge
    msh = reader(joinpath(read_path, filename))

    # Transform the original mesh: Translate, rotate, and scale
    msh = msh |> Meshes.Translate(offset...) |> Meshes.Rotate(rotation) |> Meshes.Scale(scaling)

    # Format trailing edge vertices into a matrix of points
    points = [v.coords[i] for i in 1:3, v in msh.vertices]

    # Multidiscretize parameters
    discretization = [(1.0, npoints, 1.0, true)]

    # Sort points by the parameterization to make it injective
    points = sortslices(points; dims=2, by=sortingfunction)

    # Rediscretize the shedding line
    out = []
    new_points = gt.rediscretize_line(points, discretization;
                                        parameterization=sortingfunction,
                                        out=out)

    # Remove points close to junctions
    tokeep = [i for (i, X) in enumerate(eachcol(new_points)) if junctioncriterion(X) > 0.0]
    new_points = new_points[:, tokeep]

    @assert size(new_points, 2) !=0 "Junction criterion removed all points!"

    # Close the contour if needed
    len = norm(new_points[:, 1] - new_points[:, end])
    if closed && len!=0
        if len <= 1e2*eps()
            new_points[:, end] .= new_points[:, 1]
        else
            new_points = hcat(new_points, new_points[:, 1])
        end
    end

    # Verification plot
    if verify_splines
        fig = plt.figure(filename, figsize=[7, 5]*1.5)
        ax = fig.gca()

        ax.plot(points[2, :], points[3, :], ".k", label="Raw")
        ax.plot(new_points[2, :], new_points[3, :], ".", label="Spline",
                                                            color="dodgerblue")

        ax.set_aspect("equal")
        ax.spines["right"].set_visible(false)
        ax.spines["top"].set_visible(false)
        ax.legend(loc="best", frameon=false)
    end

    return new_points

end

function shed_particles!(pfield::vpm.ParticleField, pw::PoweredWake; optargs...)

    return _shed_poweredwake_particles!(pfield, pw.sheddingline, pw.gamma;
                                            overlap=pw.overlap,
                                            sigma=pw.sigma <= 0 ? nothing : pw.sigma,
                                            optargs...
                                            )

end


function _shed_poweredwake_particles!(pfield::vpm.ParticleField,
                                        sheddingline::Matrix, gamma::Number;
                                        sigma::Union{Nothing, Number}=nothing,
                                        overlap::Number=2.125,
                                        tag0::Int=pfield.maxparticles+1,
                                        tagoffset::Int=size(sheddingline, 2)-1)

    nparticles = size(sheddingline, 2)-1
    X = zeros(3)
    Gamma = zeros(3)

    # Fetch all particles that were shed in the previous step
    particles = vpm.Particle[P for P in vpm.iterate(pfield)
                                if P.circulation[1] >= tag0 + (pfield.nt-1)*tagoffset]

    # Iterate over line sections and converts them into particles
    for p in 1:nparticles

        # Particle tag: Floor value + time stamp + particle index
        tag = tag0 + pfield.nt*tagoffset + p

        # Particle position
        for i in 1:3
            X[i]     = (sheddingline[i, p+1] + sheddingline[i, p])/2
            Gamma[i] =  sheddingline[i, p+1] - sheddingline[i, p]
        end

        # Length of the sheet element shed in this time step
        dz = _calc_dz_poweredwake(particles, X, Gamma,
                                    pfield.nt, tag, tagoffset)

        # Smoothing radius
        _sigma = isnothing(sigma) ? overlap*max(norm(Gamma), dz) : sigma

        # Particle strength
        Gamma *= gamma*dz

        # Particle circulation
        # NOTE: this is actually never used inside FLOWVPM for anything,
        #       so we will later overwrite it with `tag` and use it as a tag
        circulation = abs(gamma*dz)

        # Add particle to field
        vpm.add_particle(pfield, X, Gamma, _sigma;
                            # circulation=circulation,
                            circulation=tag
                            )

    end

    return nothing
end

function _calc_dz_poweredwake(particles::AbstractVector{<:vpm.Particle}, X, Gamma,
                                nt::Int, tag::Int, tagoffset::Int)

    # First step case: shed empty particles
    if nt==0
        dz = 1e2*eps()
        return dz
    end

    # Tag of corresponding particle that was shed in previous step
    prevtag = tag - tagoffset

    # Find corresponding particle
    previ = findfirst(P -> P.circulation[1]==prevtag, particles)

    # Error case: no such particle exists
    # @assert !isnothing(previ) ""*
        # "Logic error: Could not find particle with tag $(prevtag)"
    if isnothing(previ)
        @warn "Could not find particle with tag $(prevtag) at nt=$(nt)"

        dz = 1e2*eps()
        return dz
    end

    # Calculate displacement in between steps
    prevX = particles[previ].X
    # dX = prevX - X
    dX1 = prevX[1] - X[1]
    dX2 = prevX[2] - X[2]
    dX3 = prevX[3] - X[3]

    # Substract the distance projected in the direction of vortex strength
    # since this is already accounted for in Gamma
    normGamma = norm(Gamma)
    # dX -= dot(dX, Gamma/normGamma) * (Gamma/normGamma)
    dX1 -= (dX1*Gamma[1] + dX2*Gamma[2] + dX3*Gamma[3]) * (Gamma[1]/normGamma)
    dX2 -= (dX1*Gamma[1] + dX2*Gamma[2] + dX3*Gamma[3]) * (Gamma[2]/normGamma)
    dX3 -= (dX1*Gamma[1] + dX2*Gamma[2] + dX3*Gamma[3]) * (Gamma[3]/normGamma)

    # Return effective displacement distance
    dz = sqrt(dX1^2 + dX2^2 + dX3^2)

    return dz

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
    actuatordisks::Vector{ActuatorDisk}             # All actuator disks
    surfacevortexsheets::Vector{SurfaceVortexSheet} # All surface vortex sheets
    poweredwakes::Vector{PoweredWake}               # All powered wakes
    target_deltaVjet::Float64                       # Target jet velocity
    control_deltaVjet::Float64                      # Current control jet velocity
    couplingstrategy::Function                      # Coupling strategy

    function PropulsionSystem(actuatordisks, surfacevortexsheets, poweredwakes;
                                target_deltaVjet=1.0, control_deltaVjet=1.0,
                                couplingstrategy=nocoupling)

        return new(actuatordisks, surfacevortexsheets, poweredwakes,
                    target_deltaVjet, control_deltaVjet, couplingstrategy)

    end
end

nocoupling(deltaVjet, args...) = deltaVjet

function update(propulsion::PropulsionSystem, target_deltaVjet::Number,
                                                    pfield::vpm.ParticleField)

    propulsion.target_deltaVjet = target_deltaVjet

    deltaVjet = propulsion.couplingstrategy(target_deltaVjet, pfield)

    for svs in propulsion.surfacevortexsheets
        svs.gamma0 = deltaVjet
        svs.gamma1 = deltaVjet
    end

    for pw in propulsion.poweredwakes
        pw.gamma = deltaVjet
    end

    for ad in propulsion.actuatordisks
        # TODO
        # ad.gamma =
    end

    propulsion.control_deltaVjet = deltaVjet

    return nothing
end

function shed_wake(propulsion_systems::NTuple{L, PropulsionSystem},
                    pfield::vpm.ParticleField, dt) where {L}

    particles_shed_per_step = _get_pw_nparticles(propulsion_systems)

    # Baseline tag used for powered-wake particles in this time step
    # (an index higher than what the solver will automatically assign to regular
    # particles)
    tag0 = pfield.maxparticles + 1

    for propulsion in propulsion_systems
        for pw in propulsion.poweredwakes

            shed_particles!(pfield, pw;
                            tag0=tag0, tagoffset=particles_shed_per_step)

            tag0 += _get_pw_nparticles(pw)  # Offset by pw particles shed so far

        end
    end

end


_get_pw_nparticles(pw::PoweredWake) = pw.nparticles

"""
Returns the total number of particles shed from powered wakes in a propulsion
system at every time step.
"""
function _get_pw_nparticles(propulsion::PropulsionSystem)
    return sum(_get_pw_nparticles(pw) for pw in propulsion.poweredwakes; init=0)
end

"""
Returns the total number of particles shed from powered wakes in a system of
propulsion systems at every time step.
"""
function _get_pw_nparticles(propulsion_systems::NTuple{L, PropulsionSystem}) where {L}
    return sum(_get_pw_nparticles(p) for p in propulsion_systems; init=0)
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
