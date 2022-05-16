struct PanelGround{TF}
    panels::PS.Panels{TF}
    control_particles::vpm.ParticleField
    panel_collection
end

function PanelGround(panels::PS.Panels, panel_collection; formulation=vpm.formulation_default, viscous=vpm.Inviscid(), sgsmodel=vpm.sgs_default, sgsscaling=vpm.sgs_scaling_default, optargs...)
    # extract info
    centroids = panels.centroids
    dims, m, n, p = size(centroids)

    # create particle field
    max_particles = m*n*p
    cp_pfield = vpm.ParticleField(max_particles; formulation, viscous, sgsmodel, sgsscaling, optargs...)
    null_gamma = zeros(3)
    ip=1
    for k in 1:p
        for j in 1:n
            for i in 1:m
                vpm.add_particle(cp_pfield, centroids[:,i,j,k], null_gamma, 1.0)
            end
        end
    end

    return PanelGround(panels, cp_pfield, panel_collection)
end

function update_velocities!(panel_ground::PanelGround, Vext=(X,t)->[0.0,0,0])
    for p in vpm.iterator(panel_ground.control_particles)
        p.U .+= PS.v_induced(panel_ground.panels, PS.Source(), PS.Quad(), p.X) + Vext(p.X,0)
    end
end

function get_tag(nt)
    return ".$(nt+1).vts"
end

"Updates the pfield.Uinf function to reflect the influence of panels of appropriate strengths."
function ground_effect!(pfield::vpm.ParticleField, panel_ground::PanelGround, dt, save_field, run_name, save_path)
    # add control point particles to pfield
    control_particles = panel_ground.control_particles
    np = control_particles.np
    pfield_np = pfield.np
    for ip in 1:np
        vpm.add_particle(pfield, vpm.get_X(control_particles,ip), vpm.get_Gamma(control_particles,ip), vpm.get_sigma(control_particles,ip))
    end

    # get wake induced velocities at control particles
    vpm._reset_particles(pfield)
    pfield.UJ(pfield)
    velocities = panel_ground.panels.velocities
    dims, m, n, p = size(panel_ground.panels.centroids)
    iparticle = pfield_np+1
    for k in 1:p
        for j in 1:n
            for i in 1:m
                # iparticle = PS._index_mat_2_vec(panel_ground.panels, i, j, k)
                velocities[:,i,j,k] .= vpm.get_U(pfield, iparticle)
                iparticle += 1
            end
        end
    end

    # solve panels
    PS.solve!(panel_ground.panels, velocities) # solve for panel strengths
    # wake_velocities = deepcopy(velocities)
    velocities .= 0.0 # only look at the panel-induced velocities
    PS.update_velocities!(panel_ground.panels, PS.Source(), PS.Quad()) # add panel induced velocity to the velocity field

    # remove cp particles
    for ip in pfield_np+np:pfield_np+1
        remove_particle(pfield,ip)
    end

    # update_velocities!(panel_ground)

    # NOTE: Requires that the Uextra function is set as follows
    # kernel = PS.Source()
    # panel_shape = PS.Quad()
    # pfield.Uextra = (X) -> PS.v_induced(panel_ground.panels, kernel, panel_shape, X)

    # save particle field
    panel_collection = panel_ground.panel_collection
    if save_field
        start_i = pfield_np+1
        end_i = pfield_np+np
        pfield.t += dt
        # save(pfield, run_name*"_ground_cps"; path=save_path, start_i=start_i, end_i=end_i)
        tag = get_tag(pfield.nt)
        PS.save_vtk!(panel_collection, pfield.t, panel_ground.panels, joinpath(save_path,run_name*"_panels"); tag=tag)
        pfield.t -= dt
    end

    return false
end
