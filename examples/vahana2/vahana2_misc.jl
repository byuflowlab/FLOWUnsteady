#=##############################################################################
# DESCRIPTION
    Miscellaneous functions.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Feb 2021
  * License   : MIT
=###############################################################################


function remove_particles_lowstrength(sim, PFIELD, T, DT; crit=0.00001^2)

    for i in vpm.get_np(PFIELD):-1:1
        P = vpm.get_particle(PFIELD, i)

        if P.Gamma[1]*P.Gamma[1] + P.Gamma[2]*P.Gamma[2] + P.Gamma[3]*P.Gamma[3] < crit
            vpm.remove_particle(PFIELD, i)
        end
    end

    return false
end


function remove_particles_sphere(sim, PFIELD, T, DT; Rsphere2=(1.25*5.86)^2, Xoff=[4.0, 0, 0])

    Xvehicle = sim.vehicle.system.O

    for i in vpm.get_np(PFIELD):-1:1
        P = vpm.get_particle(PFIELD, i)
        X1 = P.X[1] - (Xvehicle[1] - Xoff[1])
        X2 = P.X[2] - (Xvehicle[2] - Xoff[2])
        X3 = P.X[3] - (Xvehicle[3] - Xoff[3])

        if X1*X1 + X2*X2 + X3*X3 > Rsphere2
            vpm.remove_particle(PFIELD, i)
        end
    end

    return false
end
