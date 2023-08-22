#=##############################################################################
# DESCRIPTION
Tools for processing openvsp degenerate geometry (DegenGeom).

# ABOUT
  * Created   : Aug 2023
  * License   : MIT
=###############################################################################

"""
    read_degengeom(filename)

Read all geometry components from a DegenGeom file written out by OpenVSP

**Arguments**
- `filename::String`: DegenGeom filename

**Returns**
- `comp`: Vector of `VSPComponent` objects
"""
function read_degengeom(filename::String)
    comp = vsp.readDegenGeom(filename)
    return comp
end

"""
    import_vsp(comp; geomType, flip_y)

Imports properties from OpenVSP component to FLOWUnsteady objects

**Arguments**
- `comp::VSPComponent`: Single `VSPComponent` object
- `geomType::String` : wing, fuselage, propeller, duct
- `flip_y::Bool` : Flip y-coordinates about longitudinal plane

**Returns**
- `geom`: FLOWUnsteady geometry
"""
function import_vsp(comp; geomType::String="wing", flip_y::Bool=false)
    if geomType == "wing"

        nXsecs, npts = vsp.degenGeomSize(comp.plate)

        # 2 endpoints for nXsecs, each with 3 coordinates
        section = Array{Float64, 3}(undef, 3, 2, nXsecs)

        # Compute lifting surface coordinates
        x = comp.plate.x .+ comp.plate.zCamber .* comp.plate.nCamberx
        y = comp.plate.y .+ comp.plate.zCamber .* comp.plate.nCambery
        z = comp.plate.z .+ comp.plate.zCamber .* comp.plate.nCamberz

        # Points in degenGeom are written as a list of coordinates for
        # each cross-section starting with trailing edge to leading edge.
        # zCamber is not necessary for trailing edge but retained for future
        #
        # Points are assigned in reverse order to flip wing sides
        # and build wing from left to right as FLOWVPM likes.
        # Wing left half

        for i = 1:nXsecs
            # Extract leading edge
            il = nXsecs-i+1
            ir = npts*i
            section[1, 1, il] = x[ir]
            section[2, 1, il] = y[ir]
            section[3, 1, il] = z[ir]

            # Extract trailing edge
            ir = npts*(i-1)+1
            section[1, 2, il] = x[ir]
            section[2, 2, il] = y[ir]
            section[3, 2, il] = z[ir]
        end

        if flip_y == false
            section[2, :, :] = -1.0*section[2, :, :]  # Flip y-coordinates
        end

        # Find variables required for creating preliminary wing object
        leftxl, lefty, leftzl = section[:, 1, 1]
        leftchord = norm(section[:, 2, 1]-section[:, 1, 1])

        leftchordtwist = 0.0
        lz = section[3, 1, 1]-section[3, 2, 1]
        if abs(lz) > 1E-6
            lx = section[1, 2, 1]-section[1, 1, 1]
            leftchordtwist = atand(lz, lx)
        end

        # Construct preliminary wing object
        wing = vlm.Wing(leftxl, lefty, leftzl, leftchord, leftchordtwist)

        # Assign coordinates for panels, control points and bound vortices
        wing.m = nXsecs-1    # No. of spanwise elements

        ## Discretized wing geometry
        for i = 2:nXsecs
            push!(wing._xlwingdcr, section[1, 1, i])
            push!(wing._xtwingdcr, section[1, 2, i])
            push!(wing._ywingdcr, section[2, 1, i])
            push!(wing._zlwingdcr, section[3, 1, i])
            push!(wing._ztwingdcr, section[3, 2, i])
        end

        ## VLM domain
        for i = 2:nXsecs
            # Bound vortex
            El = section[:, 1, i]
            Et = section[:, 2, i]
            boundVortex = El + 0.25*(Et-El)
            push!(wing._xn, boundVortex[1])
            push!(wing._yn, boundVortex[2])
            push!(wing._zn, boundVortex[3])
        end

        for i = 1:nXsecs-1
            # Control point
            Ml = 0.5*(section[:, 1, i+1] + section[:, 1, i])
            Mt = 0.5*(section[:, 2, i+1] + section[:, 2, i])
            controlPoint = Ml + 0.75*(Mt-Ml)
            push!(wing._xm, controlPoint[1])
            push!(wing._ym, controlPoint[2])
            push!(wing._zm, controlPoint[3])
        end

        geom = wing

    elseif geomType == "fuselage"

        nXsecs, npts = vsp.degenGeomSize(comp.surface_node)

        # nXsecs, each with 3 npts points
        pt = Array{Float64, 3}(undef, 3, nXsecs, npts)

        idx = 1
        for i = 1:nXsecs
            for j = 1:npts
                pt[1, i, j] = comp.surface_node.x[idx]
                pt[2, i, j] = comp.surface_node.y[idx]
                pt[3, i, j] = comp.surface_node.z[idx]
                idx = idx + 1
            end
        end

        # Check if first and last sections are a single point
        # Do this afterwards

        # Build list of cross-sections as (l/b, [x1 y1; x2 y2; ...])
        crosssections = Array{Tuple{Float64, Matrix{Float64}}}(undef, 1)
        crosssections[1] = (1.0, zeros(2, 2))  # Dummy tuple to provide hint of dimensions

        for is = 1:nXsecs
            sectionArr = zeros(npts+1, 2)
            for i = 1:npts
                sectionArr[i, :] = [pt[2, is, i], pt[3, is, i]]
            end
            sectionArr[npts+1, :] = [pt[2, is, 1], pt[3, is, 1]]
            push!(crosssections, (pt[1, is, 1], sectionArr))
        end
        deleteat!(crosssections, 1)  # Remove dummy tuple

        # Dummy values for lofting
        bscale = 1.0
        b_pos = [sec[1] for sec in crosssections]
        chords = (1/bscale)*ones(size(b_pos))
        twists = zeros(size(b_pos))
        LE_x = zeros(size(b_pos))
        LE_z = zeros(size(b_pos))

        geom = gt.generate_loft(crosssections, bscale, b_pos, chords, twists, LE_x, LE_z)

        # Rotate to align centerline with X-axis
        Oaxis = gt.rotation_matrix2(0, 0, 90)
        gt.lintransform!(geom, Oaxis, zeros(3))

        # Convert to trigrid
        geom = gt.GridTriangleSurface(geom, 1)

    elseif geomType == "propeller"
        ArgumentError("Propeller import not implemented")
    elseif geomType == "duct"
        ArgumentError("Duct import not implemented")
    else
        ArgumentError("Invalid geomType string")
    end
    return geom
end



