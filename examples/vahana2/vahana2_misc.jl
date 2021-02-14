#=##############################################################################
# DESCRIPTION
    Miscellaneous functions.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Feb 2021
  * License   : MIT
=###############################################################################


function remove_particles_lowstrength(sim, PFIELD, T, DT)

    for i in vpm.get_np(PFIELD):-1:1
        P = vpm.get_particle(PFIELD, i)

        if P.Gamma[1]*P.Gamma[1] + P.Gamma[2]*P.Gamma[2] + P.Gamma[3]*P.Gamma[3] < 0.001^2
            vpm.remove_particle(PFIELD, i)
        end
    end

    return false
end
