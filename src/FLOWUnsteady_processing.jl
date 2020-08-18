#=##############################################################################
# DESCRIPTION
    Tools for aerodynamic calculations.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################



"""
    Calculates the aerodynamic force at each element of a VLM system using it's
current `Gamma` solution, the velocity induced by the particle field `pfield`,
and the kinematic velocity calculated from `prev_vlm_system`. It saves the force
as the field `vlm_system.sol["Ftot"]`
"""
function calc_aerodynamicforce(vlm_system::Union{vlm.Wing, vlm.WingSystem},
                                prev_vlm_system, pfield, Vinf, dt, rho; t=0.0,
                                vpm_solver="ExaFMM", per_unit_span=false,
                                Vout=nothing, lenout=nothing,
                                lencrit=-1)

    m = vlm.get_m(vlm_system)    # Number of horseshoes

    # Nodes of every horseshoe
    Ap = _get_Xs(vlm_system, "Ap")
    A = _get_Xs(vlm_system, "A")
    B = _get_Xs(vlm_system, "B")
    Bp = _get_Xs(vlm_system, "Bp")

    # Midpoints of bound vortices
    ApA = (Ap .+ A)/2
    AB = (A .+ B)/2
    BBp = (B .+ Bp)/2

    # Evaluate VPM on each midpoint
    Xs = vcat(ApA, AB, BBp)
    Vvpm = Vvpm_on_Xs(pfield, Xs; dt=dt)

    # Evaluate VLM on each midpoint
    Vvlm = vlm.Vind.(Ref(vlm_system), Xs; t=t, ign_col=true, ign_infvortex=true)

    # Evaluate Vinf on each midpoint
    Vinfs = Vinf.(Xs, t)

    # Evaluate kinematic velocity on each node
    Vtran = _Vkinematic(vlm_system, prev_vlm_system, dt; t=t,
                                                targetX=["Ap", "A", "B", "Bp"])

    Ftot = [zeros(3) for i in 1:m]

    for i in 1:m                 # Iterate over horseshoes
        for j in 1:3             # Iterate over bound vortices (1==ApA, 2==AB, 3==BBp)

            # Bound vortex' length vector
            if j==1
                l = (A[i]-Ap[i])
            elseif j==2
                l = (B[i]-A[i])
            else
                l = (Bp[i]-B[i])
            end

            # Bound vortex' midpoint
            # X = Xs[i + m*(j-1)]

            # Kinematic velocity at midpoint
            Vtranmid = (Vtran[i + m*(j-1)] + Vtran[i + m*j])/2

            # Effective velocity at midpoint
            V = Vvpm[i + m*(j-1)] + Vvlm[i + m*(j-1)] + Vinfs[i + m*(j-1)] + Vtranmid
            if Vout != nothing
                push!(Vout, [Vvpm[i + m*(j-1)], Vvlm[i + m*(j-1)], Vinfs[i + m*(j-1)], Vtranmid])
            end

            # Circulation
            Gamma = vlm_system.sol["Gamma"][i]

            # Normalization factor
            if per_unit_span
                # len = norm(cross(V/norm(V),l))
                unitV = V/sqrt(V[1]*V[1]+V[2]*V[2]+V[3]*V[3])
                projl = zeros(3)
                projl[1] = unitV[2]*l[3] - unitV[3]*l[2]
                projl[2] = unitV[3]*l[1] - unitV[1]*l[3]
                projl[3] = unitV[1]*l[2] - unitV[2]*l[1]
                len = sqrt(projl[1]*projl[1]+projl[2]*projl[2]+projl[3]*projl[3])
            else
                len = 1
            end
            if lenout != nothing
                push!(lenout, per_unit_span==false || len>lencrit ? len : -10*lencrit)
            end

            # Kutta–Joukowski theorem: F = rho * V × vecGamma
            if per_unit_span==false || len > lencrit # NOTE: Use lencrit to avoid dividing by zero
                Ftot[i][1] += rho * Gamma * (V[2]*l[3] - V[3]*l[2]) / len
                Ftot[i][2] += rho * Gamma * (V[3]*l[1] - V[1]*l[3]) / len
                Ftot[i][3] += rho * Gamma * (V[1]*l[2] - V[2]*l[1]) / len
            end

        end
    end

    vlm._addsolution(vlm_system, per_unit_span ? "ftot" : "Ftot", Ftot; t=t)

    return Ftot
end

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
