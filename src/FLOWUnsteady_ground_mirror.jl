"""
Adds mirrored particles to `pfield` to impose flow tangency at the ground plane.
"""
function mirror_ground!(pfield; ground_point = [0.0,0,0], ground_normal = [0,0,1.0], save_field=true, run_name="", save_path="", kwargs...)

    np = pfield.np
    n_sources = np

    for ip in 1:pfield.np
        # get particle properties
        X = vpm.get_X(pfield, ip)
        G = vpm.get_Gamma(pfield, ip)
        S = vpm.get_sigma(pfield, ip)

        # get new X
        r = X .- ground_point
        dz = r .* ground_normal
        Xnew = X .- 2*dz

        # get new Gamma
        G_perp = G .* ground_normal
        G_para = G .- G_perp
        Gnew = G_perp .- G_para

        # add new particle
        vpm.add_particle(pfield, Xnew, Gnew, S)
    end

    # save particle field
    if save_field
        start_i = np+1
        end_i = np+n_sources
        vpm.save(pfield, run_name*"_ground"; path=save_path, start_i=start_i, end_i=end_i)
    end

    return false
end
