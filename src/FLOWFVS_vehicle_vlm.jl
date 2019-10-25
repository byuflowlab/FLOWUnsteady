################################################################################
# VLM VEHICLE TYPE
################################################################################
"""
    `Vehicle(system; optargs...)`

Type handling all geometries and subsystems that define a flight vehicle made
out of VLM (Wing, WingSystem, Rotor) components.

# ARGUMENTS
* `system::vlm.WingSystem`:        System of all FLOWVLM objects. This system
                                    is considered as the entire vehicle. Not all
                                    components in this system will be solved,
                                    but they will all be rotated and translated
                                    during maneuver.
# OPTIONAL ARGUMENTS
* `tilting_systems::Tuple(vlm.WingSystem, ...)`:   Tuple of all FLOWVLM
                                    tilting objects, where `tilting_systems[i]`
                                    contains the i-th FLOWVLM system of lifting
                                    surfaces and rotors that tilt together.
* `rotors_systems::Tuple(Array{vlm.Rotor,1}, ...)`:   Tuple of groups of Rotors
                                    that share a common RPM.
* `vlm_system::vlm.WingSystem`:    System of all FLOWVLM objects to be solved
                                    through the VLM solver.
* `wake_system::vlm.WingSystem`:   System of all FLOWVLM objects that will
                                    shed a VPM wake.
* `grids::Array{gt.GridTypes, 1}`: Array of grids that will be translated and
                                    rotated along with `system`.
"""
struct VLMVehicle{N, M, R} <: AbstractVehicle{N, M, R}

    # Required inputs
    system::vlm.WingSystem

    # Optional inputs
    tilting_systems::NTuple{N, vlm.WingSystem}
    rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}
    vlm_system::vlm.WingSystem
    wake_system::vlm.WingSystem
    grids::Array{gt.GridTypes, 1}

    # Internal properties
    V::Array{R, 1}                          # Current vehicle velocity
    W::Array{R, 1}                          # Current vehicle angular velocity
    prev_data::Array{vlm.WingSystem, 1}     # Information about previous step
    grid_O::Array{Array{R, 1}, 1}           # Origin of every grid


    VLMVehicle{N, M, R}(
                    system;
                    tilting_systems=NTuple{0, vlm.WingSystem}(),
                    rotor_systems=NTuple{0, Array{vlm.Rotor, 1}}(),
                    vlm_system=vlm.WingSystem(),
                    wake_system=vlm.WingSystem(),
                    grids=Array{gt.GridTypes, 1}(),
                    V=zeros(3), W=zeros(3),
                    prev_data=[deepcopy(vlm_system), deepcopy(wake_system)],
                    grid_O=Array{Array{Float64, 1}, 1}(),
                ) where {N, M, R} = new(
                    system,
                    tilting_systems,
                    rotor_systems,
                    vlm_system,
                    wake_system,
                    grids,
                    V, W,
                    prev_data,
                    grid_O,
                )
end

# Implicit N and M constructor
VLMVehicle(system::vlm.WingSystem;
        V::Array{R, 1}=zeros(3), W::Array{R, 1}=zeros(3),
        tilting_systems::NTuple{N, vlm.WingSystem}=NTuple{0, vlm.WingSystem}(),
        rotor_systems::NTuple{M, Array{vlm.Rotor, 1}}=NTuple{0, Array{vlm.Rotor, 1}}(),
        grids=Array{gt.GridTypes, 1}(),
        optargs...
        ) where {N, M, R} = VLMVehicle{N, M, R}( system;
                                V=V, W=W,
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                grids=Array{gt.GridTypes, 1}(grids),
                                grid_O=[zeros(R, 3) for i in 1:length(grids)],
                                optargs...)


##### FUNCTIONS  ###############################################################
get_ntltsys(self::VLMVehicle) = typeof(self).parameters[1]

get_nrtrsys(self::VLMVehicle) = typeof(self).parameters[2]


function add_dV(self::VLMVehicle, dV)
    self.V .+= dV
    return nothing
end

function add_dW(self::VLMVehicle, dW)
    self.W .+= dW
    return nothing
end

function set_V(self::VLMVehicle, V)
    self.V .= V
    return nothing
end

function set_W(self::VLMVehicle, W)
    self.W .= W
    return nothing
end

function tilt_systems(self::VLMVehicle{N,M,R}, angles::NTuple{N, Array{R2, 1}}
                                                    ) where{N, M, R, R2<:Real}
    # Iterate over tilting system
    for i in 1:get_ntltsys(self)
        sys = self.tilting_systems[i]
        Oaxis = gt.rotation_matrix2([-a for a in angles[i]]...)
        vlm.setcoordsystem(sys, sys.O, Oaxis)
    end
    return nothing
end
function tilt_systems(self::VLMVehicle{0,M,R}, angles::Tuple{})  where{M, R}
    return nothing
end


function nextstep_kinematic(self::VLMVehicle, dt::Real)
    dX = dt*self.V                  # Translation
    dA = 180/pi * dt*self.W         # Angular rotation (degrees)

    O = self.system.O + dX          # New origin of the system (translation)
    M = gt.rotation_matrix2([-a for a in dA]...) # Rotation matrix
    Oaxis = M*self.system.Oaxis     # New axes of the system (rotation)

    # Save state of current vehicle
    _update_prev_vlm_system(self, deepcopy(self.vlm_system))
    _update_prev_wake_system(self, deepcopy(self.wake_system))

    # Translation and rotation
    vlm.setcoordsystem(self.system, O, Oaxis)

    # Translation and rotation of every grid
    for i in 1:length(self.grids)
        self.grid_O[i] .+= dX

        # Translation
        gt.lintransform!(self.grids[i], eye(3), dX)

        # Brings the grid back to the global origin
        gt.lintransform!(self.grids[i], eye(3), -self.grid_O[i])

        # Rotation and brings the grid back to its position
        gt.lintransform!(self.grids[i], M, self.grid_O[i])
    end

    return nothing
end


"""
Precalculations before calling the solver.

Calculates kinematic and wake-induced velocity on VLM an adds them as a
solution field
"""
function precalculations(self::VLMVehicle, Vinf::Function,
                                pfield::vpm.ParticleField, t::Real, dt::Real)

    if t!=0
        # ---------- 1) Recalculate wake horseshoes with kinematic velocity -
        # Calculate kinematic velocity at every control point
        Vkin = _Vkinematic_wake(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.wake_system, "Vkin", Vkin; t=t)

        Vkin = _Vkinematic_vlm(self, dt; t=t, targetX="CP")
        vlm._addsolution(self.vlm_system, "Vkin", Vkin; t=t)

        # Recalculate horseshoes
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
    end

    return nothing
end

function shed_wake(self::VLMVehicle, Vinf::Function,
                            pfield::vpm.ParticleField, dt::Real; t=0.0,
                            unsteady_shedcrit=-1.0, p_per_step=1,
                            sigmafactor=1.0, overwrite_sigma=nothing)
    if t!=0
        VLM2VPM(self.wake_system, pfield, dt, Vinf; t=t,
                    prev_system=_get_prev_wake_system(self),
                    unsteady_shedcrit=unsteady_shedcrit,
                    p_per_step=p_per_step, sigmafactor=sigmafactor,
                    overwrite_sigma=overwrite_sigma, check=false)
    end
end


function solve(self::VLMVehicle, Vinf::Function, pfield::vpm.ParticleField,
                            vpm_solver::String, t::Real, dt::Real, rlx::Real)
    # TIME-STEPPING PROCEDURE:
    # -1) Solve one particle field time step
    # 0) Translate and rotate systems
    # 1) Recalculate horseshoes with kinematic velocity
    # 2) Paste previous Gamma solution to new system position after translation
    # 3) Shed semi-infinite wake after translation
    # 4) Calculate wake-induced velocity on VLM and Rotor system
    # 5) Solve VLM and Rotor system
    # 6) Shed unsteady-loading wake after new solution
    # 7) Save new solution as prev solution
    # Iterate
    #
    # On the first time step (pfield.nt==0), it only does steps (5) and (7),
    # meaning that the unsteady wake of the first time step is never shed.
    if t==0
        vlm.solve(self.vlm_system, Vinf; t=t)

        # TODO: Solve Rotor systems
    else
        # ---------- 4) Calculate VPM velocity on VLM and Rotor system -----
        # Control points
        Xs = _get_Xs(self.vlm_system, "CP"; t=t)

        # VPM-induced velocity at every control point
        Vvpm = Vvpm_on_Xs(pfield, Xs, vpm_solver)
        vlm._addsolution(self.vlm_system, "Vvpm", Vvpm; t=t)

        # ---------- 5) Solve VLM system -----------------------------------
        # Wake-coupled solution
        vlm.solve(self.vlm_system, Vinf; t=t, extraVinf=_extraVinf2, keep_sol=true,
                                        vortexsheet=(X,t)->zeros(3))
        # Wake-decoupled solution
        # vlm.solve(vlm_system, Vinf; t=t, keep_sol=true)

        # Relaxes (rlx->1) or stiffens (rlx->0) the VLM solution
        if rlx > 0
            rlxd_Gamma = rlx*self.vlm_system.sol["Gamma"] +
                                (1-rlx)*_get_prev_vlm_system(self).sol["Gamma"]
            vlm._addsolution(self.vlm_system, "Gamma", rlxd_Gamma)
        end

        # ---------- 5) Solve Rotor system ---------------------------------
        # for rotor in rotors
        #     RPM = cur_RPMs[rotor]
        #     # TODO: Add kinematic velocity
        #     # TODO: Add VPM-induced velocity and get rid of CCBlade
        #     vlm.solvefromCCBlade(rotor, Vinf, RPM, rho; t=T, sound_spd=sound_spd,
        #                             Uinds=nothing, _lookuptable=false, _Vinds=nothing)
        # end

    end
end

function save_vtk(self::VLMVehicle, filename; path=nothing, num=nothing,
                                                                    optargs...)
    strn = vlm.save(self.system, filename; path=path, num=num, optargs...)

    for (i, grid) in enumerate(self.grids)
        strn *= gt.save(grid, filename*"_Grid$i"; path=path, num=num)
    end

    return strn
end

##### INTERNAL FUNCTIONS  ######################################################

_get_prev_vlm_system(self::VLMVehicle) = self.prev_data[1]
_get_prev_wake_system(self::VLMVehicle) = self.prev_data[2]
function _update_prev_vlm_system(self::VLMVehicle, system)
    self.prev_data[1] = system
end
function _update_prev_wake_system(self::VLMVehicle, system)
    self.prev_data[2] = system
end

"""
Returns the local translational velocity of every control point in `vlm_system`.
"""
function _Vkinematic_vlm(self::VLMVehicle, args...; optargs...)
    return _Vkinematic(self.vlm_system, _get_prev_vlm_system(self), args...;
                                                                    optargs...)
end

"""
Returns the local translational velocity of every control point in `wake_system`.
"""
function _Vkinematic_wake(self::VLMVehicle, args...; optargs...)
    return _Vkinematic(self.wake_system, _get_prev_wake_system(self), args...;
                                                                    optargs...)
end

function _Vkinematic(system, prev_system, dt::Real; t=0.0, targetX="CP")

    cur_Xs = _get_Xs(system, targetX; t=t)
    prev_Xs = _get_Xs(prev_system, targetX; t=t)

    return [-(cur_Xs[i]-prev_Xs[i])/dt for i in 1:size(cur_Xs, 1)]
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
                    p_per_step=1, sigmafactor=1.0, overwrite_sigma=nothing,
                    check=true, debug=false, tol=1e-6)

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

      if unsteady_shedcrit<=0
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
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
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
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
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                                l, p_per_step; overwrite_sigma=overwrite_sigma)
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
          add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                              l, p_per_step; overwrite_sigma=overwrite_sigma)
      end


      # ----------- Case of right wing tip --------------------------
      if i==m
        # Adds particle at Bp
        X = Bp                                    # Particle position
        gamma = -Gamma                            # Infinite vortex circulation
        V = norm(Vinfs_Bp[i])                     # Freestream at X
        infD = -infDB                             # Direction of vorticity
        sigma = sigmafactor*V*dt                  # Vortex blob radius
        vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
        l = -infD*V*dt                             # Distance the TE travels

        if unsteady_shedcrit<=0
            add_particle(pfield, X, gamma, dt, V, infD, sigma, vol,
                            l, p_per_step; overwrite_sigma=overwrite_sigma)
        end
      end
    end

    # Adds unsteady particle
    if prev_system!=nothing && unsteady_shedcrit>0

      (p_Ap, p_A, p_B, p_Bp, p_CP,
          p_infDA, p_infDB, p_Gamma) = vlm.getHorseshoe(prev_system, i)
      X = (Ap+Bp)/2                               # LE midpoint position
      p_X = (p_Ap+p_Bp)/2                         # Previous LE midpoint position
      gamma = Gamma - p_Gamma                     # Bound circulation increase
      infD = (!cntgs || i==1 ? Ap : (prev_Ap+prev_Bp)/2) - X # Direction of circulation
      sigma = sigmafactor*norm(B-A)               # Vortex blob radius
      vol = 4/3*pi*(sigma/2)^3                    # Volume of particle
      l = -(X-p_X) + Vinf(X, t)*dt                # Distance the TE travels

      # Adds particle only if difference is greater than 1%
      if abs(gamma/p_Gamma) > unsteady_shedcrit
        add_particle(pfield, X, gamma, 1.0, 1.0, infD, sigma, vol,
                                        l, 1; overwrite_sigma=overwrite_sigma)
      end
    end

    prev_HS = HS
  end
end
##### END OF VEHICLE ###########################################################
