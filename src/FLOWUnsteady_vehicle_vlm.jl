#=##############################################################################
# DESCRIPTION
    Vehicles based on FLOWVLM (VLM and blade-element) models.

# ABOUT
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

################################################################################
# ABSTRACT VLM VEHICLE TYPE
################################################################################
"""
    `AbstractVLMVehicle{N, M, R} <: AbstractVehicle{N, M, R}`

Implementations of `AbstractUVLMVehicle` are expected to have the following
fields:

# Required inputs
* `system::vlm.WingSystem`

# Optional inputs
* `tilting_systems::NTuple{N, vlm.WingSystem}`
* `rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}`
* `vlm_system::vlm.WingSystem`
* `wake_system::vlm.WingSystem`
* `grids::Array{gt.GridTypes, 1}`

# Internal properties
* `V::Array{R, 1}`                          # Current vehicle velocity
* `W::Array{R, 1}`                          # Current vehicle angular velocity
* `prev_data::Array{Any, 1}`                # Information about previous step
* `grid_O::Array{Array{R, 1}, 1}`           # Origin of every grid

They also need to implement the following functions required by
`AbstractVehicle`:

* `shed_wake(...)`
* `generate_static_particle_fun(...)`
* `save_vtk(...)`

See the documentation and code of `AbstractVehicle` for more details.

In general, in order for implementation to work correctly, it is required that
the components of `wake_system` are also components of either `vlm_system` or
`rotor_systems`, and that none of the components of `rotor_systems` are also
components of `vlm_system`.
"""
abstract type AbstractVLMVehicle{N, M, R} <: AbstractVehicle{N, M, R} end

# Implementations
for header_name in ["unsteady", "quasisteady"]
  include("FLOWUnsteady_vehicle_vlm_"*header_name*".jl")
end


##### FUNCTIONS  ###############################################################
get_ntltsys(self::AbstractVLMVehicle) = typeof(self).parameters[1]

get_nrtrsys(self::AbstractVLMVehicle) = typeof(self).parameters[2]


function add_dV(self::AbstractVLMVehicle, dV)
    self.V .+= dV
    return nothing
end

function add_dW(self::AbstractVLMVehicle, dW)
    self.W .+= dW
    return nothing
end

function set_V(self::AbstractVLMVehicle, V)
    self.V .= V
    return nothing
end

function set_W(self::AbstractVLMVehicle, W)
    self.W .= W
    return nothing
end

function tilt_systems(self::AbstractVLMVehicle{N,M,R}, angles::NTuple{N, Array{R2, 1}}
                                                    ) where{N, M, R, R2<:Real}
    # Iterate over tilting system
    for i in 1:get_ntltsys(self)
        sys = self.tilting_systems[i]
        Oaxis = gt.rotation_matrix2([-a for a in angles[i]]...)
        vlm.setcoordsystem(sys, sys.O, Oaxis)
    end
    return nothing
end
function tilt_systems(self::AbstractVLMVehicle{0,M,R}, angles::Tuple{})  where{M, R}
    return nothing
end


function nextstep_kinematic(self::AbstractVLMVehicle, dt::Real)
    dX = dt*self.V                  # Translation
    dA = 180/pi * dt*self.W         # Angular rotation (degrees)

    O = self.system.O + dX          # New origin of the system (translation)
    M = gt.rotation_matrix2([-a for a in dA]...) # Rotation matrix
    Oaxis = M*self.system.Oaxis     # New axes of the system (rotation)

    # Save state of current vehicle
    _update_prev_vlm_system(self, deepcopy(self.vlm_system))
    _update_prev_wake_system(self, deepcopy(self.wake_system))
    _update_prev_rotor_systems(self, deepcopy(self.rotor_systems))

    # Translation and rotation
    vlm.setcoordsystem(self.system, O, Oaxis)

    # Translation and rotation of every grid
    for i in 1:length(self.grids)
        self.grid_O[i] .+= dX

        # Translation
        gt.lintransform!(self.grids[i], Array(1.0I, 3, 3) , dX)

        # Brings the grid back to the global origin
        gt.lintransform!(self.grids[i], Array(1.0I, 3, 3) , -self.grid_O[i])

        # Rotation and brings the grid back to its position
        gt.lintransform!(self.grids[i], M, self.grid_O[i])
    end

    return nothing
end

"""
Precalculations before calling the solver.

Calculates kinematic velocity on VLM an adds them as a solution field
"""
function precalculations(self::AbstractVLMVehicle, Vinf::Function,
                                pfield::vpm.ParticleField, t::Real, dt::Real,
                                nt::Int)

    if nt!=0
        # ---------- 1) Recalculate wake horseshoes with kinematic velocity -
        # Calculate kinematic velocity at every control point
        Vkin = _Vkinematic_wake(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.wake_system, "Vkin", Vkin; t=t)

        Vkin = _Vkinematic_vlm(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.vlm_system, "Vkin", Vkin; t=t)

        # Recalculate horseshoes
        vlm.getHorseshoes(self.vlm_system; t=t, extraVinf=_extraVinf1)
        vlm.getHorseshoes(self.wake_system; t=t, extraVinf=_extraVinf1)

        # ---------- 2) Paste previous Gamma solution ----------------------
        for i in 1:length(self.wake_system.wings)
            wing = vlm.get_wing(self.wake_system, i)
            prev_wing = vlm.get_wing(_get_prev_wake_system(self), i)

            if typeof(prev_wing) != vlm.Rotor
                sol = deepcopy(prev_wing.sol["Gamma"])
            else
                sol = deepcopy(prev_wing._wingsystem.sol["Gamma"])
            end

            vlm._addsolution(wing, "Gamma", sol; t=t)
        end
        for i in 1:length(self.vlm_system.wings)
            wing = vlm.get_wing(self.vlm_system, i)
            prev_wing = vlm.get_wing(_get_prev_vlm_system(self), i)

            if typeof(prev_wing) != vlm.Rotor
                sol = deepcopy(prev_wing.sol["Gamma"])
            else
                sol = deepcopy(prev_wing._wingsystem.sol["Gamma"])
            end

            vlm._addsolution(wing, "Gamma", sol; t=t)
        end
    end

    return nothing
end

function save_vtk_base(self::AbstractVLMVehicle, filename; path=nothing,
                                   num=nothing, save_wopwopin=false,
                                   infinite_vortex=false, optargs...)
    strn = vlm.save(self.system, filename; path=path, num=num,
                                    infinite_vortex=infinite_vortex, optargs...)

    for (i, grid) in enumerate(self.grids)
        strn *= gt.save(grid, filename*"_Grid$i"; format="vtk", path=path, num=num)
    end

    # Generate inputs for PSU-WOPWOP
    if save_wopwopin
        for (si, rotors) in enumerate(self.rotor_systems)
            # Compact patch for loading
            generate_vtkliftinglines(rotors, filename*"_Sys$(si)", path; num=num,
                                                                suf="_compact")
            # Loft for thickness
            # NOTE: this is not needed since FLOWNoise can read the lofted VTK
            # straight up
            # for (ri, rotors) enumerate(rotors)
            #     vlm.save(rotor, filename*"_Sys$(si)_Rotor$(ri)"; path=path, addtiproot=true,
            #                             wopwop=true, wopbin=true, wopv=1.0, num=num)
            # end

            # Loading file
            save_gammas = [[vlm.get_blade(rotor, i).sol["Gamma"]
                                      for i in 1:rotor.B] for rotor in rotors]
            # NOTE TO SELF: Ftot is a force per unit length
            save_Ftot = [[vlm.get_blade(rotor, i).sol["Ftot"]
                                      for i in 1:rotor.B] for rotor in rotors]
            loadfname = "loading_Sys$(si)"*(num!=nothing ? ".$num" : "")*".jld"
            JLD.save(joinpath(path, loadfname), "rotorgammas", save_gammas,
                                                        "Ftot", save_Ftot)
        end
    end

    return strn
end





##### INTERNAL FUNCTIONS  ######################################################

_get_prev_vlm_system(self::AbstractVLMVehicle) = self.prev_data[1]
_get_prev_wake_system(self::AbstractVLMVehicle) = self.prev_data[2]
_get_prev_rotor_systems(self::AbstractVLMVehicle) = self.prev_data[3]
function _update_prev_vlm_system(self::AbstractVLMVehicle, system)
    self.prev_data[1] = system
end
function _update_prev_wake_system(self::AbstractVLMVehicle, system)
    self.prev_data[2] = system
end
function _update_prev_rotor_systems(self::AbstractVLMVehicle, rotor_systems)
    self.prev_data[3] = rotor_systems
end


"""
Return the maximum number of static particles.
"""
function _get_m_static(self::AbstractVLMVehicle)
    m = vlm.get_m(self.vlm_system)

    for rotors in self.rotor_systems
        for rotor in rotors
            m += vlm.get_m(rotor)
        end
    end

    return m
end

"""
Returns the local translational velocity of every control point in `vlm_system`.
"""
function _Vkinematic_vlm(self::AbstractVLMVehicle, args...; optargs...)
    return _Vkinematic(self.vlm_system, _get_prev_vlm_system(self), args...;
                                                                    optargs...)
end

"""
Returns the local translational velocity of every control point in `wake_system`.
"""
function _Vkinematic_wake(self::AbstractVLMVehicle, args...; optargs...)
    return _Vkinematic(self.wake_system, _get_prev_wake_system(self), args...;
                                                                    optargs...)
end

function _Vkinematic(system, prev_system, dt::Real; t=0.0, targetX="CP")

    cur_Xs = _get_Xs(system, targetX; t=t)
    prev_Xs = _get_Xs(prev_system, targetX; t=t)

    return [-(cur_Xs[i]-prev_Xs[i])/dt for i in 1:size(cur_Xs, 1)]
end

"""
Returns the local translational velocity of every midpoint in the ri-th rotor in
the si-th system formatted as the output of `_get_midXs()`.
"""
function _Vkinematic_rotor(rotor_systems::NTuple{M, Array{vlm.Rotor, 1}},
                           prev_rotor_systems::NTuple{M, Array{vlm.Rotor, 1}},
                           si, ri, dt::Real
                          ) where{M}

    cur_Xs = _get_midXs(rotor_systems[si][ri])
    prev_Xs = _get_midXs(prev_rotor_systems[si][ri])

    Vkin = [-(cur_Xs[i]-prev_Xs[i])/dt for i in 1:size(cur_Xs, 1)]

    return Vkin

end

function _get_Xs(system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
                                                        targetX::String; t=0.0)
    if targetX=="CP"
        Xs = [vlm.getControlPoint(system, i
                                        ) for i in 1:vlm.get_m(system)]
    else
        Xs = [vlm.getHorseshoe(system, i; t=t)[vlm.VLMSolver.HS_hash[targetX]
                                        ] for i in 1:vlm.get_m(system)]
    end
    return Xs
end

function _get_Xs(system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor},
                                        targetXs::Array{T,1}; t=0.0) where{T}
    return vcat([_get_Xs(system, targetX; t=t) for targetX in targetXs]...)
end

"Returns the i-th bound-vortex midpoint of the j-th blade in `rotor`"
function _get_midX(rotor::vlm.Rotor, j::Int, i::Int)
    return (vlm.getHorseshoe(vlm.get_blade(rotor, j), i)[2] +
            vlm.getHorseshoe(vlm.get_blade(rotor, j), i)[3]) / 2
end

"Returns all bound-vortex midpoints of the j-th blade in `rotor`"
function _get_midXs(rotor::vlm.Rotor, j::Int)
    return [_get_midX(rotor, j, i)
                            for i in 1:vlm.get_m(vlm.get_blade(rotor, j)) ]
end
"Returns all bound-vortex midpoints in `rotor`"
_get_midXs(rotor::vlm.Rotor) = vcat([_get_midXs(rotor, j) for j in 1:rotor.B]...)

"Returns all bound-vortex midpoints in an array of rotors"
_get_midXs(rotors::Array{vlm.Rotor, 1}) = vcat([_get_midXs(rotor) for rotor in rotors]...)

"Returns all bound-vortex midpoints in multiple rotor systems"
function _get_midXs(rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}) where{M}
    return vcat([_get_midXs(rotors) for rotors in rotor_systems]...)
end

"Receives an array `midXs` formatted as the output of `_get_midXs(rotor_systems)`
and returns the section of the array that corresponds to the ri-th rotor in the
si-th system (formatted linearly).
"
function _parse_midXs(rotor_systems::NTuple{M, Array{vlm.Rotor, 1}},
                        midXs::Array{Array{R, 1}}, si, ri) where{R<:Real, M}

    if si>length(rotor_systems) || si<=0
        error("Got invalid system index $si; max is $(length(rotor_systems))")
    end

    # Find lower bound of midXs belonging to rotor ri
    i_low = 1
    for (sj, rotors) in enumerate(rotor_systems[1:si])       # Iterate over rotor systems
        for (rj, rotor) in enumerate(rotors)                 # Iterative over rotors
            if sj==si && rj==ri
                break
            end
            i_low += vlm.get_m(rotor)
        end
    end

    # Find upper bound of midXs belonging to rotor ri
    rotor = rotor_systems[si][ri]
    i_up = i_low-1 + vlm.get_m(rotor)

    # # Format it as blades
    out = midXs[i_low:i_up]
    # out = [out[ ((i-1)*rotor.m+1):(i*rotor.m) ] for i in 1:rotor.B]
    return out
end

"""
    Receives the output of `parse_midXs(rotor_systems, midXs, si, ri)` and
outputs the array formatted as blades.
"""
function _format_blades(arr::Array{Array{R, 1}, 1},
                        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}},
                        si, ri) where{R, M}

    rotor = rotor_systems[si][ri]
    arr = [arr[ ((i-1)*rotor.m+1):(i*rotor.m) ] for i in 1:rotor.B]

    return arr
end

"""
Function for FLOWVLM to fetch and add extra velocities to what every
control point imposes as the boundary condition
"""
function _extraVinf1(i, t; wing=nothing)
    if wing==nothing; error("Logic error!"); end;
    return wing.sol["Vkin"][i]
end
function _extraVinf2(i, t; wing=nothing)
    if wing==nothing; error("Logic error!"); end;
    return wing.sol["Vvpm"][i] + wing.sol["Vkin"][i]
end
function _extraVinf3(i, t; wing=nothing)
    if wing==nothing; error("Logic error!"); end;
    return wing.sol["Vind"][i] + wing.sol["Vkin"][i]
end



"""
Receives the FLOWVLM object `system` (Wing/WingSystem/Rotor), and adds vortex
particles to the particle field `pfield` at each trailing edge position where
an infinite vortex starts. `dt` indicates the length of the interval of time
that the vortex shedding represents.

Give it a previous system to detect differences in circulation and add
unsteady particles.
"""
function VLM2VPM(system::Union{vlm.Wing, vlm.WingSystem, vlm.Rotor}, pfield, dt,
                    Vinf;
                    t=0.0, prev_system=nothing, unsteady_shedcrit=-1.0,
                    shed_starting=false,
                    p_per_step=1, sigmafactor=1.0, overwrite_sigma=nothing,
                    check=true, debug=false, tol=1e-6, omit_shedding=[])

  m = vlm.get_m(system)   # Number of lattices

  # Velocity at horseshoe points Ap and Bp
  if prev_system==nothing
      Vinfs_Ap = vlm.getVinfs(system; t=t, target="Ap")
      Vinfs_Bp = vlm.getVinfs(system; t=t, target="Bp")
  else
      Vinfs_Ap = []
      Vinfs_Bp = []
      for i in 1:m
          Ap = vlm.getHorseshoe(system, i)[1]
          prev_Ap = vlm.getHorseshoe(prev_system, i)[1]
          push!(Vinfs_Ap, -(Ap-prev_Ap)/dt + Vinf(Ap, t))

          Bp = vlm.getHorseshoe(system, i)[4]
          prev_Bp = vlm.getHorseshoe(prev_system, i)[4]
          push!(Vinfs_Bp, -(Bp-prev_Bp)/dt + Vinf(Bp, t))
      end
  end

  # Adds a particle at each infinite vortex
  prev_HS = [nothing for i in 1:8]
  for i in 1:m  # Iterates over lattices
    HS = vlm.getHorseshoe(system, i)

    Ap, A, B, Bp, CP, infDA, infDB, Gamma = HS
    (prev_Ap, prev_A, prev_B, prev_Bp, prev_CP,
      prev_infDA, prev_infDB, prev_Gamma) = prev_HS
    cntgs = true          # Contiguous horseshoes flag

    if nothing in HS; error("Logic error! $HS"); end;
    if true in [isnan.(elem) for elem in HS]; error("Logic error! $HS"); end;

    # ----------- Case of left wing tip ---------------------------
    if i==1
      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma                             # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      # Check if the system loops around
      if norm(Ap - vlm.getHorseshoe(system, m)[4]) / norm(Bp - Ap) <= tol
          gamma -= vlm.getHorseshoe(system, m)[8]
      end

      if unsteady_shedcrit<=0
          if !(i in omit_shedding)
              add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
          end
      end

    # ----------- Case of wing tip on discontinuous wing --------
    elseif norm(Ap - prev_Bp) / norm(Bp - Ap) > tol
      if debug; println("Wing tip found at i=$i. $Ap!=$prev_Bp"); end;

      cntgs = false

      # Adds particle at previous Bp
      X = prev_Bp                               # Particle position
      gamma = -prev_Gamma                       # Infinite vortex circulation
      V = norm(Vinfs_Bp[i-1])                   # Freestream at X
      infD = -prev_infDB                        # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(prev_Bp-prev_Ap)/2)^2*V*dt # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          if !(i-1 in omit_shedding)
              add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
          end
      end

      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma                             # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          if !(i in omit_shedding)
              add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
          end
      end

    # ----------- Case of contiguous horseshoes -----------------
    else

      # Verify logic
      if check
        crit1 = norm(Vinfs_Ap[i]-Vinfs_Bp[i-1]) / norm((Vinfs_Ap[i]+Vinfs_Bp[i-1])/2)
        crit2 = norm(infDA-prev_infDB) / norm((infDA+prev_infDB)/2)
        if crit1 > tol
          error("Logic error! Vinfs_Ap[i]!=Vinfs_Bp[i-1] "*
                  "( $(Vinfs_Ap[i])!=$(Vinfs_Bp[i-1]) )")
        elseif crit2 > tol
          error("Logic error! infDA!=prev_infDB "*
                  "( $(infDA)!=$(prev_infDB) )")
        elseif norm(infDA) - 1 > tol
          error("Logic error! norm(infDA)!= 1 "*
                  "( $(norm(infDA)) )")
        end
      end

      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma - prev_Gamma                # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = sigmafactor*V*dt                  # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = -infD*V*dt                             # Distance the TE travels

      if unsteady_shedcrit<=0
          if !(i in omit_shedding)
              add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
          end
      end


      # ----------- Case of discontinuous right wing tip -----------------------
      if i==m && norm(Bp - vlm.getHorseshoe(system, 1)[1]) / norm(Bp - Ap) > tol
        # Adds particle at Bp
        X = Bp                                    # Particle position
        gamma = -Gamma                            # Infinite vortex circulation
        V = norm(Vinfs_Bp[i])                     # Freestream at X
        infD = -infDB                             # Direction of vorticity
        sigma = sigmafactor*V*dt                  # Vortex blob radius
        vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
        l = -infD*V*dt                             # Distance the TE travels

        if unsteady_shedcrit<=0
            if !(i in omit_shedding)
                add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                            l, p_per_step; overwrite_sigma=overwrite_sigma)
            end
        end
      end
    end

    # Adds unsteady particle
    if prev_system!=nothing && unsteady_shedcrit>0

      (p_Ap, p_A, p_B, p_Bp, p_CP,
          p_infDA, p_infDB, p_Gamma) = vlm.getHorseshoe(prev_system, i)
      X = (Ap+Bp)/2                               # LE midpoint position
      p_X = (p_Ap+p_Bp)/2                         # Previous LE midpoint position
      gamma = Gamma - p_Gamma                 # Bound circulation increase
      infD = (!cntgs || i==1 ? Ap : (prev_Ap+prev_Bp)/2) - X # Direction of circulation
      sigma = sigmafactor*norm(B-A)               # Vortex blob radius
      vol = 4/3*pi*(sigma/2)^3                    # Volume of particle
      l = -(X-p_X) + Vinf(X, t)*dt                # Distance the TE travels

      # Adds particle only if difference is greater than certain threshold
      if abs(gamma/p_Gamma) > unsteady_shedcrit && !(i in omit_shedding)
          add_particle(pfield, X, gamma, 1.0, 1.0, infD, sigma, vol,
                                        l, 1; overwrite_sigma=overwrite_sigma)
      end

      if shed_starting  && !(i in omit_shedding)
          gamma = Gamma                           # Starting vortex circulation
          add_particle(pfield, X, gamma, 1.0, 1.0, infD, sigma, vol,
                                        2*l, 1; overwrite_sigma=overwrite_sigma)
      end
    end

    prev_HS = HS
  end
end

function VLM2VPM_draggingline(vlmobject::vlm.WingSystem, prev_vlmobject,
                                                            args...; optargs...)
    for (wi, wing) in enumerate(vlmobject.wings)
        VLM2VPM_draggingline(wing, prev_vlmobject.wings[wi], args...; optargs...)
    end
end

function VLM2VPM_draggingline(vlmobject::Union{vlm.Wing, vlm.Rotor}, prev_vlmobject,
                    pfield, dt, Vinf,
                    d;   # Dipole width
                    t=0.0,
                    prescribed_Cd=nothing,
                    p_per_step=1, sigmafactor=1.0, overwrite_sigma=nothing,
                    )

    # NOTE: omite_shedding not implemented yet

    m = vlm.get_m(vlmobject)   # Number of lattices

    # Adds a particle at each infinite vortex
    prev_HS = [nothing for i in 1:8]
    for i in 1:m  # Iterates over lattices

        HS = vlm.getHorseshoe(vlmobject, i)
        Ap, A, B, Bp, CP, infDA, infDB, _ = HS

        (p_Ap, p_A, p_B, p_Bp, p_CP,
            p_infDA, p_infDB, _) = vlm.getHorseshoe(prev_vlmobject, i)

        if nothing in HS; error("Logic error! $HS"); end;
        if true in [isnan.(elem) for elem in HS]; error("Logic error! $HS"); end;

        Xmid = (Ap + Bp)/2                          # Current TE midpoint
        p_Xmid = (p_Ap + p_Bp)/2                    # Previous TE midpoint
        l_TE = -(Xmid-p_Xmid) + Vinf(Xmid, t)*dt    # Distance the TE travels
        l_span = Bp-Ap                              # Spanwise TE length
        nhat = cross(CP-A, B-A)                     # Unit vector normal to the control point
        nhat ./= norm(nhat)

        if prescribed_Cd != nothing
            # Dragging line dipole strength (see Caprace's thesis, 2020)
            # NOTE: Here I'm correcting DG's model dividing by chord length and later
            #       multiplying by the traveled distance to account for the time step
            mu_drag = 0.5*norm(l_TE/dt)*prescribed_Cd
        else
            if typeof(vlmobject)==vlm.Rotor
                mu_drag = vlmobject._wingsystem["mu"][i]
            else
                error("Automatic calculation of dipole strength for Wing"*
                        " objects has not been implemented yet. Please provide"*
                        " an airfoil drag coefficient Cd through `prescribed_Cd`")
            end
        end

        gamma = mu_drag*norm(l_TE)                  # Circulation of the dragging vortex ring
        sigma = sigmafactor*norm(Bp-Ap)             # Vortex blob radius
        vol = 4/3*pi*(sigma/2)^3                    # Volume of particle

        # Add lower vortex
        X = Xmid - d/2*nhat
        dir = Bp-Ap                                 # Direction and length of vortex
        # if !(i in omit_shedding)
            add_particle(pfield, X, gamma, 1.0, 1.0, dir, sigma, vol,
                              l_TE, p_per_step; overwrite_sigma=overwrite_sigma)
        # end

        # Add right vortex
        X = Bp
        dir = d*nhat
        # if !(i in omit_shedding)
            add_particle(pfield, X, gamma, 1.0, 1.0, dir, sigma, vol,
                              l_TE, p_per_step; overwrite_sigma=overwrite_sigma)
        # end

        # Add lower vortex
        X = Xmid + d/2*nhat
        dir = Ap-Bp
        # if !(i in omit_shedding)
            add_particle(pfield, X, gamma, 1.0, 1.0, dir, sigma, vol,
                              l_TE, p_per_step; overwrite_sigma=overwrite_sigma)
        # end

        # Add left vortex
        X = Ap
        dir = -d*nhat
        # if !(i in omit_shedding)
            add_particle(pfield, X, gamma, 1.0, 1.0, dir, sigma, vol,
                              l_TE, p_per_step; overwrite_sigma=overwrite_sigma)
        # end

        prev_HS = HS
    end
end
##### END OF ABSTRACT VLM VEHICLE ##############################################
