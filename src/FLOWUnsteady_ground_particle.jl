"""
Contains:

* `pfield::vpm.ParticleField`: particle field with one particle at each desired ground particle location.
* `A::Matrix{Float64}`: array of size `pfield.np` by `pfield.np` used to solve for circulation strengths in each basis vector direction.
* `b::Vector{Float64}`: vector of length `pfield.np` used to solve for circulation strengths in each basis vector direction as `A Gamma = b`.
* `Gamma_basis::Vector{Vector{Float64}}`: vector containing unit vectors used to solve for circulation strengths.
* `ground_normal::Vector{Float64}`: unit vector defines the ground normal vector.
"""
struct ParticleGround{TF,V}
    sources::vpm.ParticleField{TF, V}
    cps::vpm.ParticleField{TF, V}
    sources_per_panel::Int64
    n_control::Int64
    unique::Matrix{Int64}
    A::Matrix{TF}
    Ainv::Union{Matrix{TF},Nothing}
    b::Vector{TF}
    ground_normal::Vector{TF}
end

function ParticleGround(sources::vpm.ParticleField, cps::vpm.ParticleField, unique, ground_normal; invert_A = true, benchmark=false, sources_per_panel =4, optargs...)
    if benchmark
        println("\nBuild A and b arrays\n")
        @time begin
            # check lengths
            n_control = cps.np

            # initialize arrays
            A = zeros(n_control, n_control)
            b = zeros(n_control)

            # build A matrix
            update_A!(A, sources, cps, sources_per_panel, ground_normal)
        end
        println("\nInvert A matrix\n")
        @time begin
            if invert_A
                # invert A matrix
                println("INVERTING GROUND A MATRIX...")
                Ainv = inv(A)
            else
                Ainv = nothing
            end
        end
    else
        # check lengths
        n_control = cps.np

        # initialize arrays
        A = zeros(n_control, n_control)
        b = zeros(n_control)

        # build A matrix
        update_A!(A, sources, cps, sources_per_panel, ground_normal)

        if invert_A
            # invert A matrix
            println("INVERTING GROUND A MATRIX...")
            Ainv = inv(A)
        else
            Ainv = nothing
        end
    end

    return ParticleGround(sources, cps, sources_per_panel, n_control, unique, A, Ainv, b, ground_normal)
end

function ParticleGround(xs_cp, ys_cp, zs_cp, sigma; n_per_panel=4, ground_normal=[0.0, 0.0, 1.0], benchmark=false, uj_function=fmm, optargs...)
    if benchmark
        println("\nbuild_sources:\n")
        sources, unique = @time build_sources(xs_cp, ys_cp, zs_cp, sigma)
        println("\nbuild_cps:\n")
        cps = @time build_cps(xs_cp, ys_cp, zs_cp, n_per_panel; uj_function)
    else
        sources, unique = build_sources(xs_cp, ys_cp, zs_cp, sigma)
        cps = build_cps(xs_cp, ys_cp, zs_cp, n_per_panel; uj_function)
    end

    return ParticleGround(sources, cps, unique, ground_normal; benchmark, optargs...)
end

# function unique_indices(n_per_panel, panels)
#     n_panels = length(panels)
#     unique = zeros(Int64, n_panels*n_per_panel)
#     for i=1:n_panels

#     end
# end

function build_cps(xs_cp, ys_cp, zs_cp, n_per_panel; uj_function=fmm)
    lx = length(xs_cp)
    ly = length(ys_cp)
    lz = length(zs_cp)
    @assert lz == 1 "multiple layers in z not supported yet; got $lz"

    cps = vpm.ParticleField((lx*ly*lz + (lx-1)*(ly-1)) + n_per_panel; UJ=uj_function)

    # add particles
    for z in zs_cp
        for y in ys_cp
            for x in xs_cp
                vpm.add_particle(cps, [x,y,z], zeros(3), 1.0)
            end
        end
    end

    for z in zs_cp
        for (j,y) in enumerate(ys_cp)
            for (i,x) in enumerate(xs_cp)
                if j < ly && i < lx
                    vpm.add_particle(cps, [(xs_cp[i] + xs_cp[i+1])/2, (ys_cp[j] + ys_cp[j+1])/2, z], zeros(3), 1.0)
                end
            end
        end
    end

    return cps
end

function get_distances(list)
    len = length(list)
    deltas = zeros(len - 1)
    for i=1:len-1
        deltas[i] = list[i+1] - list[i]
    end
    return deltas
end

function build_sources(xs_cp, ys_cp, zs_cp, sigma; n_per_panel=4)
    # 4 source particles per control point
    lx_cp = length(xs_cp)
    ly_cp = length(ys_cp)
    lz_cp = length(zs_cp)
    n_sources = (lx_cp * ly_cp * lz_cp + (lx_cp-1) * (ly_cp-1)) * n_per_panel
    sources = vpm.ParticleField(n_sources)

    dxs = get_distances(xs_cp)
    dys = get_distances(ys_cp)
    dzs = 0.0
    # dzs = zeros(lz_cp + 1)

    # map = zeros(Int64, n_per_panel, lz_cp * ly_cp * lx_cp)
    unique = zeros(Int64, n_per_panel, lz_cp * ly_cp * lx_cp + (ly_cp-1) * (lx_cp-1))
    cp_i = 1
    sp_i = 1

    # FCC control points
    for iz in 1:lz_cp
        for iy in 1:ly_cp
            for ix in 1:lx_cp
                # get positions
                dy_north = iy==ly_cp ? dys[iy-1]/2 : dys[iy]/2
                X_north = [xs_cp[ix], ys_cp[iy] + dy_north, zs_cp[iz]]

                dx_west = ix==1 ? dxs[ix]/2 : dxs[ix-1]/2
                X_west = [xs_cp[ix] - dx_west, ys_cp[iy], zs_cp[iz]]

                dx_east = ix==lx_cp ? dxs[ix-1]/2 : dxs[ix]/2
                X_east = [xs_cp[ix] + dx_east, ys_cp[iy], zs_cp[iz]]

                dy_south = iy==1 ? dys[iy]/2 : dys[iy-1]/2
                X_south = [xs_cp[ix], ys_cp[iy] - dy_south, zs_cp[iz]]

                # get smoothing radii
                # sigma_west_east = (dy_south + dy_north) * overlap
                # sigma_south_north = (dx_west + dx_east) * overlap
                sigma_west_east = sigma_south_north = sigma

                # add particles

                # get map indices
                if iy == 1 # first row
                    if ix == 1 # first column
                        vpm.add_particle(sources, X_north, [1.0, 0.0, 0.0], sigma_south_north)
                        unique[1,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_east, [0.0, -1.0, 0.0], sigma_west_east)
                        unique[2,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_south, [-1.0, 0.0, 0.0], sigma_south_north)
                        unique[3,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_west, [0.0, 1.0, 0.0], sigma_west_east)
                        unique[4,cp_i] = sp_i
                        sp_i += 1
                    else
                        vpm.add_particle(sources, X_north, [1.0, 0.0, 0.0], sigma_south_north)
                        unique[1,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_east, [0.0, -1.0, 0.0], sigma_west_east)
                        unique[2,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_south, [-1.0, 0.0, 0.0], sigma_south_north)
                        unique[3,cp_i] = sp_i
                        sp_i += 1
                        vpm.add_particle(sources, X_west, [0.0, 1.0, 0.0], sigma_west_east)
                        unique[4,cp_i] = unique[2,cp_i-1]
                    end
                else # other rows
                    vpm.add_particle(sources, X_north, [1.0, 0.0, 0.0], sigma_south_north)
                    unique[1,cp_i] = sp_i
                    sp_i += 1
                    vpm.add_particle(sources, X_east, [0.0, -1.0, 0.0], sigma_west_east)
                    unique[2,cp_i] = sp_i
                    sp_i += 1
                    vpm.add_particle(sources, X_south, [-1.0, 0.0, 0.0], sigma_south_north)
                    unique[3,cp_i] = unique[1,cp_i - lx_cp]
                    if ix == 1 # first column
                        vpm.add_particle(sources, X_west, [0.0, 1.0, 0.0], sigma_west_east)
                        unique[4,cp_i] = sp_i
                        sp_i += 1
                    else
                        vpm.add_particle(sources, X_west, [0.0, 1.0, 0.0], sigma_west_east)
                        unique[4,cp_i] = unique[2,cp_i-1]
                    end
                end

                cp_i += 1
            end
        end
    end

    # HCP control points
    for iz in 1:lz_cp
        for iy in 1:ly_cp-1
            for ix in 1:lx_cp-1

                # look at the control point to the north-east
                i_linear = (ix+1) + iy * lx_cp
                vpm.add_particle(sources, vpm.get_X(sources, 4 + 4*(i_linear-1)), [1.0,0,0], sigma)
                unique[1,cp_i] = unique[4,i_linear]
                vpm.add_particle(sources, vpm.get_X(sources, 3 + 4*(i_linear-1)), [0,-1.0,0], sigma)
                unique[2,cp_i] = unique[3,i_linear]

                # look at the control point to the south-west
                i_linear = ix + (iy-1) * lx_cp
                vpm.add_particle(sources, vpm.get_X(sources, 2 + 4*(i_linear-1)), [-1.0,0,0], sigma)
                unique[3,cp_i] = unique[2,i_linear]
                vpm.add_particle(sources, vpm.get_X(sources, 1 + 4*(i_linear-1)), [0,1.0,0], sigma)
                unique[4,cp_i] = unique[1,i_linear]

                cp_i += 1
            end
        end
    end

    return sources, unique
end

function update_A!(ground_field::ParticleGround)
    # extract variables
    cps = ground_field.cps # all particles should have null strength
    sources = ground_field.sources # all particles should have unit strengths
    unique = ground_field.unique
    ground_normal = ground_field.ground_normal
    A = ground_field.A

    update_A!(A, sources, cps, n_per_panel, ground_normal)
    return nothing
end

function update_A!(A, sources, cps, n_per_panel, ground_normal)
    n_control = cps.np
    # @show n_control

    # iterate over source "panels"
    for i_source in 1:n_control
        # add source particles to the control point field
        for pi in 1:n_per_panel
            this_i = (i_source-1) * n_per_panel + pi
            X = vpm.get_X(sources, this_i)
            Gamma = vpm.get_Gamma(sources, this_i)
            @assert isapprox(Gamma' * Gamma, 1.0; atol=1e-9) "particles in sources must have unit circulation; got Gamma = $Gamma"
            sigma = vpm.get_sigma(sources, this_i)
            vpm.add_particle(cps, X, Gamma, sigma; static=true)
        end

        # reset field velocities
        vpm._reset_particles(cps)

        # get induced velocity
        cps.UJ(cps)

        # update A
        for icp in 1:n_control
            U = vpm.get_U(cps, icp)
            normal_vi = sum(U .* ground_normal)
            A[icp, i_source] = normal_vi
            if vpm.get_Gamma(cps, icp) != [0.0,0.0,0.0]
                println("Control point $icp Gamma = $(vpm.get_Gamma(cps, icp))")
            end
        end

        # remove source panel particles
        for pi in 1:n_per_panel
            vpm.remove_particle(cps, cps.np)
        end
    end

    return nothing
end

"""
Assumes zero-strength ground particles have already been added to `pfield` at each collocation point, as using `transfer_particles!(pfield, gfield)`.

Note: `b` should contain the number of `gfield` source particles, which is equal to `gfield.pfield.np/2`.
"""
function update_b!(pfield::vpm.ParticleField, ground_field::ParticleGround)
    # get references
    b = ground_field.b
    ground_normal = ground_field.ground_normal
    cps = ground_field.cps # should be zero-strength particles
    n_cps = cps.np

    # add control points to pfield
    for icp in 1:cps.np
        X = vpm.get_X(cps,icp)
        Gamma = vpm.get_Gamma(cps,icp) # should be zero
        vpm.add_particle(pfield, X, Gamma, 1.0)
    end

    # get induced velocity from `pfield` at each particle location
    vpm._reset_particles(pfield)
    pfield.UJ(pfield)

    # update b
    np = pfield.np - n_cps

    for (bi_cp, icp) in enumerate(np+1:1:np+n_cps) # iterate over all control points
        local vi = vpm.get_U(pfield, icp) # get the velocity at the corresponding control point
        b[bi_cp] = -sum(vi .* ground_normal)
    end

    # remove control points
    for ip = pfield.np:-1:np+1
        vpm.remove_particle(pfield, ip)
    end

    return nothing
end

function solve_Gammas(ground_field)
    A = ground_field.A
    Ainv = ground_field.Ainv
    b = ground_field.b
    if !isnothing(Ainv)
        Gammas = Ainv * b
    else
        Gammas = A \ b
    end
    # resid_vec = A*Gammas .- b
    # resid = transpose(resid_vec) * resid_vec
    # condition = cond(A)
    # println("resid = $resid")
    return Gammas
end

"Assumes A and b have been updated."
function add_sources!(pfield::vpm.ParticleField, ground_field::ParticleGround, Gammas)
    unique = ground_field.unique
    cps = ground_field.cps
    n_control = cps.np
    sources = ground_field.sources
    np = pfield.np
    sources_per_panel = ground_field.sources_per_panel

    i_latest = 0
    i = 0
    for icp in 1:n_control
        for isp in 1:sources_per_panel
            i_transfer = unique[isp,icp] # get the index of the particle transferred to pfield
            i += 1 # ith iteration
            if i_transfer > i_latest # a new particle
                vpm.add_particle(pfield, sources.particles[i])
                pfield.particles[np + i_transfer].Gamma .*= Gammas[icp]
                i_latest += 1
            else # an existing particle
                Gamma = sources.particles[i].Gamma .* Gammas[icp]
                pfield.particles[np + i_transfer].Gamma .+= Gamma
            end
        end
    end

    return nothing
end

"""
Adds ground particles of appropriate strength to `pfield`.
"""
function ground_effect!(pfield::vpm.ParticleField, ground_field::ParticleGround, save_field, run_name, save_path, update_A)
    # get references
    A = ground_field.A
    b = ground_field.b
    # sources = ground_field.sources
    # n_sources = get_nsources(gfield)

    # transfer_source_particles!(pfield, gfield; static=true)

    np = pfield.np
    n_sources = ground_field.sources.np

    if update_A; update_A!(ground_field); end

    update_b!(pfield, ground_field)

    Gammas = solve_Gammas(ground_field)

    # add source particles
    add_sources!(pfield, ground_field, Gammas)

    # save particle field
    if save_field
        start_i = np+1
        end_i = np+n_sources
        vpm.save(pfield, run_name*"_ground"; path=save_path, start_i=start_i, end_i=end_i)
    end
    return nothing
end
