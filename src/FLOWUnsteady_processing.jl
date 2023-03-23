#=##############################################################################
# DESCRIPTION
    Tools for processing the aerodynamic solutions.

# ABOUT
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

"""
Receives an array of vectors `Fs` and decomposes them into components in the
`ihat`, `jhat`, `khat` unitary orthogonal directions (`khat` is deduced).
"""
function decompose(Fs, ihat, jhat)

    if abs(ihat[1]*ihat[1]+ihat[2]*ihat[2]+ihat[3]*ihat[3] - 1) >= 1e-12
        error("Got non-unitary vector ihat=$ihat")
    elseif abs(jhat[1]*jhat[1]+jhat[2]*jhat[2]+jhat[3]*jhat[3] - 1) >= 1e-12
        error("Got non-unitary vector jhat=$jhat")
    end

    khat = zeros(3)
    khat[1] = ihat[2]*jhat[3] - ihat[3]*jhat[2]
    khat[2] = ihat[3]*jhat[1] - ihat[1]*jhat[3]
    khat[3] = ihat[1]*jhat[2] - ihat[2]*jhat[1]

    Fis = [zeros(3) for i in 1:length(Fs)]
    Fjs = [zeros(3) for i in 1:length(Fs)]
    Fks = [zeros(3) for i in 1:length(Fs)]

    for (n, F) in enumerate(Fs)
        Fis[n] .= (F[1]*ihat[1]+F[2]*ihat[2]+F[3]*ihat[3]) * ihat
        Fjs[n] .= (F[1]*jhat[1]+F[2]*jhat[2]+F[3]*jhat[3]) * jhat
        Fks[n] .= (F[1]*khat[1]+F[2]*khat[2]+F[3]*khat[3]) * khat
    end

    return Fis, Fjs, Fks
end


"""
    remove_particles_strength(minGamma2, maxGamma2; every_nsteps=1)

Returns an `extra_runtime_function` that removes all particles with a vortex
strength magnitude that is larger than `sqrt(maxGamma2)` or smaller than
`sqrt(minGamma2)`. Use `every_nsteps` to indicate every how many steps to
remove particles.
"""
function remove_particles_strength(minGamma2::Real, maxGamma2::Real; every_nsteps::Int=1)

    function wake_treatment(sim, PFIELD, T, DT, args...; optargs...)
        if sim.nt%every_nsteps==0

            for i in vpm.get_np(PFIELD):-1:1
                P = vpm.get_particle(PFIELD, i)

                if !(minGamma2 <= P.Gamma[1]*P.Gamma[1] + P.Gamma[2]*P.Gamma[2] + P.Gamma[3]*P.Gamma[3] <= maxGamma2)
                    vpm.remove_particle(PFIELD, i)
                end
            end

        end

        return false
    end

    return wake_treatment
end


"""
    remove_particles_lowstrength(critGamma2, step)

Returns an `extra_runtime_function` that removes all particles with a vortex
strength magnitude that is smaller than `sqrt(critGamma2)`.
Use this wake treatment to avoid unnecesary computation by removing particles
that have negligibly-low strength.

`step` indicates every how many steps to remove particles.
"""
remove_particles_lowstrength(crit_Gamma2, step) = remove_particles_strength(crit_Gamma2, Inf; every_nsteps=step)



"""
    remove_particles_sigma(minsigma, maxsigma; every_nsteps=1)

Returns an `extra_runtime_function` that removes all particles with a smoothing
radius that is larger than `maxsigma` or smaller than `minsigma`.
Use `every_nsteps` to indicate every how many steps to remove particles.
"""
function remove_particles_sigma(minsigma::Real, maxsigma::Real; every_nsteps::Int=1)

    function wake_treatment(sim, PFIELD, T, DT, args...; optargs...)
        if sim.nt%every_nsteps==0

            for i in vpm.get_np(PFIELD):-1:1
                P = vpm.get_particle(PFIELD, i)

                if !(minsigma <= P.sigma[1] <= maxsigma)
                    vpm.remove_particle(PFIELD, i)
                end
            end

        end

        return false
    end

    return wake_treatment
end


"""
    remove_particles_box(Pmin::Vector, Pmax::Vector, step::Int)

Returns an `extra_runtime_function` that every `step` steps removes all
particles that are outside of a box of minimum and maximum vertices `Pmin`
and `Pmax`.

Use this wake treatment to avoid unnecesary computation by removing particles
that have gone beyond the region of interest.
"""
function remove_particles_box(Pmin, Pmax, step::Int)

    function wake_treatment(sim, PFIELD, T, DT, args...; optargs...)
        if sim.nt%step==0

            for i in vpm.get_np(PFIELD):-1:1
                P = vpm.get_particle(PFIELD, i)

                if (  (P.X[1] < Pmin[1] || P.X[1] > Pmax[1])
                        ||
                      (P.X[2] < Pmin[2] || P.X[2] > Pmax[2])
                        ||
                      (P.X[3] < Pmin[3] || P.X[3] > Pmax[3])
                   )
                    vpm.remove_particle(PFIELD, i)
                end
            end

        end

        return false
    end

    return wake_treatment
end


"""
    remove_particles_sphere(Rsphere2, step::Int; Xoff::Vector=zeros(3))

Returns an `extra_runtime_function` that every `step` steps removes all
particles that are outside of a sphere of radius `sqrt(Rsphere2)` centered
around the vehicle or with an offset `Xoff` from the center of the vehicle.

Use this wake treatment to avoid unnecesary computation by removing particles
that have gone beyond the region of interest.
"""
function remove_particles_sphere(Rsphere2, step::Int; Xoff=zeros(3))

    function wake_treatment(sim, PFIELD, T, DT, args...; optargs...)

        Xvehicle = sim.vehicle.system.O

        for i in vpm.get_np(PFIELD):-1:1
            P = vpm.get_particle(PFIELD, i)
            X1 = P.X[1] - (Xvehicle[1] + Xoff[1])
            X2 = P.X[2] - (Xvehicle[2] + Xoff[2])
            X3 = P.X[3] - (Xvehicle[3] + Xoff[3])

            if X1*X1 + X2*X2 + X3*X3 > Rsphere2
                vpm.remove_particle(PFIELD, i)
            end
        end

        return false
    end

    return wake_treatment
end
