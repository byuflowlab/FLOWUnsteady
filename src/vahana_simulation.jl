"""
`multirotor` can either be a Rotor object, or an Array{Rotor,1} for multirotors
(it assumes all rotors have the same geometry).
"""
function vehicle_simulation(multirotor, Vinf::Function, fun_omegas::Function,
                    nsteps, dt;
                    # Outputs
                    save_path=nothing, run_name="vahana",
                    create_savepath=true,
                    prompt=true, paraview=true,
                    verbose=true, v_lvl=0,
                    save_code=joinpath(module_path,""),
                    # Simulation parameters
                    rho=1.225, mu=1.81e-5, sound_spd=343,
                    # Options
                    buffer=true, hub_particles=true,
                    steady=true, unsteady_shedcrit=0.01,
                    p_per_step=1, shed_every_step=1,
                    break_function=nothing,
                    max_wake_shedding=nothing, wake_removal_O=zeros(3),
                    nsteps_del=10, del_particles_function=nothing,
                    nsteps_save=1, nsteps_save_restart=nothing,
                    extraruntime_function=nothing,
                    # Particle field solver
                    solver="ExaFMM", sigmafactor=2.0, minsigma=nothing,
                    timemeth="rk", strtch="transpose", regularized=true,
                    nsteps_relax=1, relaxfactor=0.3, pse=true,
                    pf_restart_file=nothing, restart_p_n=-1,
                    gm_restart_file=nothing,
                    parallel_Uphi=true, lifting_surface=true,
                    vlm_sigma=nothing, lf_sigma=nothing,
                    wake_coupled=false, coupling_method="noniterative",
                    iterative_rlx=0.7, vlm_CP=false,
                    plot_convergence=true,
                    vpm_Uphi=true, vpm_Uphi2=false,
                    P2PType=3,
                    visc_cs=false, cs_beta=1.5, sgm0=nothing,
                    lagrand_lambda=nothing, max_lagrand_np=10,
                    rbf_itmax=15, rbf_tol=1e-3)

  restart_file = pf_restart_file

  if typeof(multirotor)==vlm.Rotor
    rotors = vlm.Rotor[multirotor]
  elseif typeof(multirotor)==Array{vlm.Rotor, 1}
    rotors = multirotor
  else
    error("invalid `multirotor` type $(typeof(multirotor))")
  end

  if p_per_step!=1 && shed_every_step!=1
      error("Invalid sheds per steps!")
  end

  if vlm_CP; println("*"^72*"\nINDUCED VELOCITIES AT CONTROL POINTS\n"*"*"^72); end;

  if verbose; println("\t"^v_lvl*"Defining parameters..."); end;
  # -------------- PARAMETERS --------------------------------------------------
  # Geometry
  R = rotors[1].rotorR          # Rotor radius
  Rhub = rotors[1].hubR         # Hub radius
                                # Mean chord
  # Operation
  nu = mu/rho     # (m^2/s) kinematic viscosity

  # Simulation
  angles = zeros(size(rotors,1))                   # (deg) current azimuthal angle
  # nsteps_save = 1               # Steps between outputing vtks
  # buffer = true               # Whether to add a wake-buffer region
  # hub_particles = true        # Whether to add particles at the root

  nsteps_restart = (nsteps_save_restart!=nothing ?
                            nsteps_save_restart : 99999999999999)


  function fun_Vinf(X,t)
      return Vinf(X,t)
  end


  # -------------- LIFTING SURFACE Vind FUNCTIONS  -----------------------------

  if wake_coupled
    if vlm_sigma==nothing
        vlm.VLMSolver._blobify(true)
        vlm.VLMSolver._smoothing_rad(4*3*(R-Rhub)/(vlm.get_mBlade(rotors[1])))
    elseif vlm_sigma==0
        vlm.VLMSolver._blobify(false)
    else
        vlm.VLMSolver._blobify(true)
        vlm.VLMSolver._smoothing_rad(vlm_sigma)
    end
  end

  # Parallel evaluation of lifting surface
  updateHSs() = vcat([[deepcopy(vlm.getHorseshoe(rotor, i))
                        for i in 1:vlm.get_m(rotor)] for rotor in rotors]...)
  HSs = nothing
  limO = deepcopy(rotors[1]._wingsystem.O)
  limOaxis = deepcopy(rotors[1]._wingsystem.Oaxis[1,:])
  function fun_vlmVind_par(X,t)
    out = zeros(3)

    if abs(dot(X, limOaxis))<0.5*R
      for HS in HSs
          out += vlm.VLMSolver.V(HS, X; ign_infvortex=true)
      end
    end

    return out
  end

  # Serial evaluation of lifting surface
  "Returns velocity induced by bound vortices"
  function fun_vlmVind_ser(X,t)
    out = zeros(3)

    for rotor in rotors
      if abs(dot(X, rotor._wingsystem.Oaxis[1,:]))<0.5*R && norm(X-rotor._wingsystem.O)<1.5*R
        out += vlm.Vind(rotor, X; t=t, ign_infvortex=true)
      end
    end

    return out
  end

  # Lifting surface evaluated as a particle field
  max_particles = vlm.get_m(rotors[1])*size(rotors,1)*(4*nsteps+1)
  surfpfield = vpm.ParticleField(max_particles, x->zeros(3), nothing, solver)

  "Returns velocity and derivates induced by bound vortices as a VPM"
  function fun_vpmVind(Xs, t)
    surfpfield.nt = p_field.nt

    # Adds a particle for every bound vortex of the VLM
    for (Ap, A, B, Bp, _, _, _, Gamma) in HSs

      for (i,(x1, x2)) in enumerate([(Ap,A), (A,B), (B,Bp)])
        # add_particle(surfpfield, (x1+x2)/2, Gamma, 1.0, 1.0, (x2-x1),
        #                                 sigma, 1.0, sigmafactor; ms=minsigma)
        sigma = lf_sigma!=nothing ? lf_sigma : vlm_sigma!=nothing ? vlm_sigma : 0.1*norm(A-Ap)

        vpm.addparticle(surfpfield, vcat((x1+x2)/2, Gamma*(x2-x1), sigma, 0))
      end
    end

    np = vpm.get_np(surfpfield)

    # Adds probing particles
    for x in Xs; vpm.addparticle(surfpfield, vcat(x, zeros(3), 1e-10, 0)); end;

    # Calculates induced velocity and derivatives
    Utot, dUdx, dUdy, dUdz, _, _, _ = vpm.U_ExaFMM(
                                    surfpfield._p_field, vpm.get_np(surfpfield))

    # Formats output
    aux00 = [dUdx, dUdy, dUdz]
    Uphis = [Utot[np+i,j] for i in 1:size(Xs,1), j in 1:3]
    JUphis = [aux00[k][np+i,j] for i in 1:size(Xs,1), j in 1:3, k in 1:3]

    # Debugging
    # if save_path!=nothing
    #   vpm.save_vtk(surfpfield, "surfpfield"; path=save_path, save_sigma=true)
    #   vpm.save_vtk(surfpfield, "surfpfield"; path=save_path, save_sigma=true, num=0)
    # end

    # Removes all particles
    for i in vpm.get_np(surfpfield):-1:1; vpm.delparticle(surfpfield, i); end;

    return Uphis, JUphis
  end



  "Freestream for particle field"
  function pVinf(X,t)
    return fun_Vinf(X,t)
  end

  generate_static_particles = nothing

  "Lifting surface induced velocity for particle field"
  if lifting_surface
    if vpm_Uphi

      if parallel_Uphi==false
        error("VLM->VPM Uphi requested without parallel_Uphi flag."*
              " This flag is required for updating HSs.")
      end

      function generate_static_particles(args...)
        out = Array{Float64,1}[]

        # Adds a particle for every bound vortex of the VLM
        for (Ap, A, B, Bp, _, _, _, Gamma) in HSs
          for (i,(x1, x2)) in enumerate([(Ap,A), (A,B), (B,Bp)])
            sigma = lf_sigma!=nothing ? lf_sigma : vlm_sigma!=nothing ? vlm_sigma : 0.1*norm(A-Ap)
            push!(out, vcat((x1+x2)/2, Gamma*(x2-x1), sigma, 0))
          end
        end

        return out
      end

      pVphi = nothing

    elseif vpm_Uphi2
      if parallel_Uphi==false
        error("VLM->VPM Uphi requested without parallel_Uphi flag."*
              " This flag is required for updating HSs.")
      end
      pVphi = fun_vpmVind

    elseif parallel_Uphi
      pVphi = fun_vlmVind_par

    else
      pVphi = fun_vlmVind_ser
    end

  else
    pVphi = nothing
  end

  "Returns the velocity induced by both the lifting surface and the wake"
  function fun_surfwakeVind(X,t)

    # out=zeros(3)
    # VLM lifting surface
    out = fun_vlmVind_ser(X,t)

    # VPM wake
    out += vpm.reg_Uomega(p_field, X)

    return out
  end


  """Given the velocity induced at each control point (Vind = Vwake, no lifting
  surface), solves for the Gamma field (circulation) on each blade by looking at
  the airfoil polar at the effective angle of attack of every section. It also
  includes the fields Ftot, L, D, and S.

  This method solves iteratively until the circulation distribution converges.

  NOTE: Vind is expected to be in the global coordinate system.
  NOTE: Vind is expected to be formated as Vind[k][i][j] being the velocity vector
  of the j-th control point in the i-th blade in the k-th rotor.
  """
  function solvefromVite(ROTORS::Array{vlm.Rotor,1}, Vind::Array{Array{Array{T, 1}, 1},1},
                            t::Real, fun_Vinf, omegas, args...;
                            maxite::Int64=10, tol::Real=0.01, rlx=0.8, optargs...
                            ) where{T<:vlm.FArrWrap}


    if "Gamma" in keys(vlm.get_blade(ROTORS[1], 1).sol)
        prev_sol = [[vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B] for rotor in ROTORS]
    else
        prev_sol = nothing
    end
    ite = 0
    err = nothing


    for n in 1:maxite

      this_sol = []
      for (k,rotor) in enumerate(ROTORS)
        if n==1 && prev_sol==nothing
          surfVind = Vind[k]
        else
          # Adds V induced by lifting surfaces
          surfVind = [[ Vind[k][i][j] + fun_vlmVind_ser(
                    vlm_CP ? vlm.getControlPoint(vlm.get_blade(rotor, i), j) :
                    (vlm.getHorseshoe(vlm.get_blade(rotor, i), j)[2] +
                     vlm.getHorseshoe(vlm.get_blade(rotor, i), j)[3]) / 2 , t
                  ) for j in 1:size(Vind[k][i],1)] for i in 1:size(Vind[k],1)]
        end

        vlm.solvefromV(rotor, surfVind, fun_Vinf, omegas[k], args...; optargs...)
      end
      this_sol = [[vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B] for rotor in ROTORS]

      if n!=1
        # Checking convergence: Average variation
        err = mean([
                     mean([
                            mean( abs.(prev_sol[j][i]-this_sol[j][i])./abs.(prev_sol[j][i]) ) for i in 1:length(rotor.B)
                          ]) for (j,rotor) in enumerate(ROTORS)
                    ])
        println("ite:$n\terr:$err")
        if err < tol
          break
        end

        # Relaxation
        for (j,rotor) in enumerate(ROTORS)
            for i in 1:rotor.B
              blade = vlm.get_blade(rotor, i)
              blade.sol["Gamma"][:] = rlx*prev_sol[j][i] .+ (1-rlx)*this_sol[j][i]
            end
        end
      end

      if plot_convergence
        fig = figure("gammadistr")
        cratio = t/(nsteps*dt)
        cratio = cratio > 1 ? 1 : cratio
        if n==1
          xlabel("Element index")
          ylabel(L"Circulation $\Gamma$ (m$^2$/s)")
          title("Convergence of Circulation Distribution")
          plot(1:size(init_sol,1), init_sol, "ok", label="CCBlade")
          legend(loc="best")
          grid(true, color="0.8", linestyle="--")
        end
        plot(1:size(init_sol,1), init_sol, "ok")
        rlx_sol = vcat([vcat([vlm.get_blade(rotor, i).sol["Gamma"] for i in 1:rotor.B]...) for rotor in ROTORS]...)
        plot(1:size(rlx_sol,1), rlx_sol, alpha=0.5, color=(1-cratio, n/maxite, cratio))
      end

      prev_sol = this_sol
      ite += 1
    end

    if ite==maxite
      warn("Iterative Rotor solvefromV reached max iterations without converging."
          *" maxite:$maxite\t error:$err")
    end
  end

  # -------------- VLM INITIALIZATION ------------------------------------------
  if verbose; println("\t"^v_lvl*"Calculating initial rotor solution..."); end;

  # Calculates circulation from CCBlade
  for rotor in rotors
    vlm.solvefromCCBlade(rotor, fun_Vinf, fun_omegas(rotor, 0)/(2*pi)*60, rho; t=0.0,
                            sound_spd=sound_spd)
  end

  if gm_restart_file!=nothing

      restart_gammas = JLD.load(gm_restart_file, "rotorgammas")
      # restart_gammas, Ftot = JLD.load(gm_restart_file, "rotorgammas", "Ftot")

      # Sets restart Gamma distribution as the initial solution
      for (ri,rotor) in enumerate(rotors)

          vlm._addsolution(rotor, "Gamma", vcat(restart_gammas[ri]...))
          # vlm._addsolution(rotor, "Ftot", vcat(Ftot[ri]...))

          # for bi in 1:rotor.B
          #     vlm.get_blade(rotor, bi).sol["Gamma"] = restart_gammas[ri][bi]
          # end
      end
  end

  if parallel_Uphi
    @everywhere vlm.VLMSolver._mute_warning(true)
  else
    vlm.VLMSolver._mute_warning(true)
  end

  prev_rotors = nothing

  if steady
    warn("Steady option was chosen. Initial rotor circulation will be"*
          " maintained throughout simulation outside of rampup revolutions.")
      steady_flag = true
      rotorsgammas = [[vlm.get_blade(rotor, i).sol["Gamma"] for i in 1:rotor.B]
                                                            for rotor in rotors]
  end

  init_sol = []
  for rotor in rotors
    init_sol = vcat(init_sol, [vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B]...)
  end


  # -------------- VPM INITIALIZATION ------------------------------------------
  if verbose; println("\t"^v_lvl*"Initializing particle field..."); end;
  # solver = "ExaFMM"                   # Direct, or FMM interactions

  # New particle field
  if restart_file==nothing
    max_particles = 4*vlm.get_m(rotors[1])*size(rotors,1)*nsteps
    # NOTE: vlmVind should be in place of the phi function, but I haven't
    #       implemented that into the VPM yet.
    p_field = vpm.ParticleField(max_particles, pVinf, pVphi, solver)

  # Restart run
  else
    # Number of particles along blade on restart file
    if restart_p_n==-1
      error("Number of particles along blade on restart file is required.")
    end
    r_p_n = restart_p_n*size(rotors,1)

    p_field = vpm.read(restart_file, pVinf, pVphi; solver_method=solver)
    p_field.nt = 0

    # Deletes last particles to make room for the first step
    angles = zeros(rotors, 1)
    for i in p_field.np:-1:p_field.np-2*(r_p_n-1)
      vpm.delparticle(p_field, i)
    end

    # CODE A CLEAN UP FUNCTION HERE (Moves the particles back to start of array)
    if r_p_n>0

        error("Congrats! You found a hole in the code: DEPRECATED SECTION OF CODE")
      np_to_del = B*(r_p_n-1)*nsteps_per_rev*7
      # for i in np_to_del:-1:1
      #   vpm.delparticle(p_field, i)
      # end
      p_field._p_field[1:p_field.np-np_to_del, :] = p_field._p_field[np_to_del+1:p_field.np, :]
      p_field._p_field[p_field.np-np_to_del+1:p_field.np, :] = zeros(np_to_del, vpm._n_param)
      p_field.np = p_field.np - np_to_del
    end
  end

  nt_init = p_field.nt


  # ---------------- SCHEMES -------------------------------------------------
  vpm.set_TIMEMETH(timemeth)          # Time integration scheme
  vpm.set_STRETCHSCHEME(strtch)       # Vortex stretching scheme
  vpm.set_RELAXETA(relaxfactor/dt)    # Relaxation param
  # nsteps_relax = 1                  # Steps between relaxation
  # sigmafactor = 2.0                 # Particle overlap
  vpm.set_PSE(pse)                    # Viscous diffusion through PSE
  vpm.set_CS(visc_cs)                 # Viscous diffusion through CS
  p_field.nu = nu                     # Kinematic viscosity

  if regularized
    vpm.set_P2PTYPE(Int32(P2PType))
  else # Singular kernel, transpose scheme works better on this
    vpm.set_P2PTYPE(Int32(2))
    if nsteps_relax!=1
      warn("Singular scheme with relaxation requested:"*
          " This is not recommended due to singular instabilities.")
    end
  end

  if lagrand_lambda!=nothing
    vpm.set_LAGRANDIST(true) # Whether to fix lagrangian distortion while doing CS
    vpm.set_LAMBDA(lagrand_lambda)
    vpm.set_MAXLAGRANDNP(Int32(max_lagrand_np*vlm.get_m(rotors[1])))

  else
    vpm.set_LAGRANDIST(false)
  end


  # -------------- VPM RUNTIME FUNCTION ----------------------------------------
  add_particles(pfield, t, dt) = VLM2VPM(rotors, pfield, dt, sigmafactor;
                                          t=t, add_hub=hub_particles,
                                          prev_system=prev_rotors,
                                          ms=minsigma, p_per_step=p_per_step,
                                          unsteady_shedcrit=unsteady_shedcrit)
  function def_del_particles(pfield, t, dt)
    if max_wake_shedding!=nothing
      for i in vpm.get_np(pfield):-1:1
        if norm(vpm.get_x(pfield, i)-wake_removal_O) > max_wake_shedding
          vpm.delparticle(pfield, i)
        end
      end
    end
  end

  del_particles = del_particles_function!=nothing ? del_particles_function : def_del_particles

  function runtime_function(PFIELD, T, DT)

      # println("T=$T")
      # for (ri, rotor) in enumerate(rotors)
      #     println("\tRotor #$ri")
      #     for bi in 1:rotor.B
      #         println("\t\tBlade #$bi: $(vlm.get_blade(rotor, bi).sol["Gamma"])")
      #     end
      # end

    if buffer && PFIELD.nt!=0
      if PFIELD.nt%shed_every_step==0
          add_particles(PFIELD, T, shed_every_step*DT)
      end
    end

    # Rotates the rotor
    if PFIELD.nt!=0
      rotations = [fun_omegas(rotor, T)*DT for rotor in rotors]          # (rad)
      angles += rotations

      if wake_coupled
        if steady
          prev_rotors = nothing
        else
          if PFIELD.nt%shed_every_step==0
              prev_rotors = vlm.Rotor[deepcopy(rotor) for rotor in rotors]
          end
        end


        prev_gammas = [[vlm.get_blade(rotor, i).sol["Gamma"]
                                for i in 1:rotor.B] for rotor in rotors]


          # Call function that applies kinematics to vehicle
          if extraruntime_function!=nothing
            extraruntime_function(PFIELD, T, DT, rotors, angles*180/pi, nsteps, PFIELD.nt)
          end

          for (ri, rotor) in enumerate(rotors)
            # Rotates every rotor
            vlm.rotate(rotor, rotations[ri]*(180/pi))
          end

          if coupling_method=="iterative"

            # Pastes previous gamma solution for Vind evaluation
            for (k,rotor) in enumerate(rotors)
              for i in 1:rotor.B
                blade = vlm.get_blade(rotor, i)
                blade.sol["Gamma"] = prev_gammas[k][i]
              end
            end

            # Induced velocity at every CP of every blade of every rotor by wake
            Vinds = [ [ [
                    vpm.reg_Uomega(p_field,
                        vlm_CP ? vlm.getControlPoint(vlm.get_blade(rotor, bi), i) : (
                        vlm.getHorseshoe(vlm.get_blade(rotor, bi), i)[2] +
                        vlm.getHorseshoe(vlm.get_blade(rotor, bi), i)[3]) / 2 ) for i in 1:vlm.get_mBlade(rotor)] for bi in 1:rotor.B] for rotor in rotors]


            # Calculates new circulation from lookup tables
            solvefromVite(rotors, Vinds, T, fun_Vinf, [fun_omegas(rotor, T) for rotor in rotors]/(2*pi)*60, rho;
                                rlx=iterative_rlx,
                                t=T, sound_spd=sound_spd)

            elseif coupling_method=="noniterative"

              # Pastes previous gamma solution for Vind evaluation
              for (k,rotor) in enumerate(rotors)
                for i in 1:rotor.B
                  blade = vlm.get_blade(rotor, i)
                  blade.sol["Gamma"] = prev_gammas[k][i]
                end
              end

              # Induced velocity at every CP of every blade of every rotor (includes lifting surface of self)
              Vinds = [ [ [
                      fun_surfwakeVind(
                        vlm_CP ? vlm.getControlPoint(vlm.get_blade(rotor, bi), i) :
                        (vlm.getHorseshoe(vlm.get_blade(rotor, bi), i)[2] +
                         vlm.getHorseshoe(vlm.get_blade(rotor, bi), i)[3]) / 2, T)
                      for i in 1:vlm.get_mBlade(rotor)] for bi in 1:rotor.B] for rotor in rotors]

              for (ri, rotor) in enumerate(rotors)
                vlm.solvefromV(rotor, Vinds[ri], fun_Vinf, fun_omegas(rotor, T)/(2*pi)*60,
                                rho; t=T, sound_spd=sound_spd)
              end

              if plot_convergence
                fig = figure("gammadistr")
                cratio = PFIELD.nt/nsteps
                cratio = cratio > 1 ? 1 : cratio
                if PFIELD.nt<=1
                  xlabel("Element index")
                  ylabel(L"Circulation $\Gamma$ (m$^2$/s)")
                  title("Convergence of Circulation Distribution")
                  plot(1:size(init_sol,1), init_sol, "ok", label="CCBlade")
                  legend(loc="best")
                  grid(true, color="0.8", linestyle="--")
                end
                this_sol = []
                for rotor in rotors
                  this_sol = vcat(this_sol, [vlm.get_blade(rotor, j).sol["Gamma"] for j in 1:rotor.B]...)
                end
                plot(1:size(init_sol,1), init_sol, "ok")
                plot(1:size(this_sol,1), this_sol, alpha=0.5, color=(1-cratio, 0, cratio))
              end

            else
              error("Invalid coupling method $(coupling_method).")
            end


      elseif steady


        # Call function that applies kinematics to vehicle
        if extraruntime_function!=nothing
            extraruntime_function(PFIELD, T, DT, rotors, angles*180/pi, nsteps, PFIELD.nt)
        end

        prev_rotors = nothing
        for (ri, rotor) in enumerate(rotors)
          vlm.rotate(rotor, rotations[ri]*(180/pi))
        end

        if steady_flag
          # Pastes the initial CCBlade solution back
          for (k,rotor) in enumerate(rotors)
            for i in 1:rotor.B
              blade = vlm.get_blade(rotor, i)
              blade.sol["Gamma"] = rotorsgammas[k][i]
            end
          end
        else
          # Calculates final circulation from CCBlade
          for rotor in rotors
            vlm.solvefromCCBlade(rotor, fun_Vinf, fun_omegas(rotor, T)/(2*pi)*60, rho;
                                                    t=T, sound_spd=sound_spd)
          end
          rotorsgammas = [[vlm.get_blade(rotor, i).sol["Gamma"]
                                    for i in 1:rotor.B] for rotor in rotors]
          steady_flag = true
        end

      else
        if PFIELD.nt%shed_every_step==0
            prev_rotors = vlm.Rotor[deepcopy(rotor) for rotor in rotors]
        end


        # Call function that applies kinematics to vehicle
        if extraruntime_function!=nothing
            extraruntime_function(PFIELD, T, DT, rotors, angles*180/pi, nsteps, PFIELD.nt)
        end

        for (ri, rotor) in enumerate(rotors)
          vlm.rotate(rotor, rotations[ri]*(180/pi))

          # Calculates new circulation from CCBlade
          vlm.solvefromCCBlade(rotor, fun_Vinf, fun_omegas(rotor, T)/(2*pi)*60, rho;
                                          t=T, sound_spd=sound_spd)
        end
      end
    end

    # Adds particles
    if !buffer
      if PFIELD.nt%shed_every_step==0
          add_particles(PFIELD, T, shed_every_step*DT)
      end
    end

    # Deletes particles
    if PFIELD.nt%nsteps_del==0
      del_particles(PFIELD, T, DT)
    end

    if save_path!=nothing && PFIELD.nt%nsteps_save==0
      for (k,rotor) in enumerate(rotors)
        aux1 = run_name*(size(rotors,1)>1 ? "_Rotor$k" : "")
        vlm.save(rotor, aux1; save_horseshoes=save_horseshoes,
                              only_horseshoes=false,
                              infinite_vortex=save_infvortex,
                              airfoils=save_airfoils,
                              path=save_path, num=PFIELD.nt, t=DT,
                              wopwop=(nt_init==PFIELD.nt))
      end
    end

    if save_path!=nothing && PFIELD.nt%nsteps_restart==0
        save_gammas = [[vlm.get_blade(rotor, i).sol["Gamma"]
                                  for i in 1:rotor.B] for rotor in rotors]
        save_Ftot = [[vlm.get_blade(rotor, i).sol["Ftot"]
                                  for i in 1:rotor.B] for rotor in rotors]

        # NOTE: Here I'm not sure whether I should be dividing by the span-wise
        # length or the actual length
        save_ftot = [[[vlm.get_blade(rotor, i).sol["Ftot"][j] / norm(
                            vlm.getHorseshoe(vlm.get_blade(rotor, i), j)[2] -
                            vlm.getHorseshoe(vlm.get_blade(rotor, i), j)[3] )
                            for j in 1:vlm.get_m(vlm.get_blade(rotor, i))]
                                  for i in 1:rotor.B] for rotor in rotors]

        JLD.save(joinpath(save_path,"gammas.$(PFIELD.nt).jld"),
                                                "rotorgammas", save_gammas,
                                                "Ftot", save_Ftot,
                                                "ftot", save_ftot)
    end

    if parallel_Uphi
      HSs = updateHSs()
    end

    return (break_function!=nothing ? break_function(rotors, PFIELD) : false)
  end


  # --------------- RUN THE VPM ----------------------------------------------
  vpm.run_vpm!(p_field, dt, nsteps; save_path=save_path, run_name=run_name,
                    verbose=verbose,
                    create_savepath=create_savepath,
                    runtime_function=runtime_function, solver_method=solver,
                    nsteps_save=nsteps_save,
                    save_code=save_code,
                    prompt=prompt,
                    nsteps_relax=nsteps_relax,
                    nsteps_restart=nsteps_restart,
                    parallel_Uphi=parallel_Uphi,
                    group_Uphi=vpm_Uphi2,
                    static_particles_function=generate_static_particles,
                    beta=cs_beta, sgm0=sgm0,
                    save_sigma=true, rbf_ign_iterror=true,
                    rbf_itmax=rbf_itmax, rbf_tol=rbf_tol)


  # --------------- VISUALIZATION ----------------------------------------------
  if save_path!=nothing

    if paraview
      println("Calling Paraview...")
      strn = ""

      for (k, rotor) in enumerate(rotors)
        aux1 = run_name*(size(rotors,1)>1 ? "_Rotor$k" : "")
        for i in 1:rotor.B
          strn = strn * aux1 * "_Blade$(i)_vlm...vtk;"
        end
        for i in 1:rotor.B
          strn = strn * aux1 * "_Blade$(i)_loft...vtk;"
        end
      end

      strn = strn * run_name * "_pfield...vtk;"

      run(`paraview --data="$save_path$strn"`)
    end
  end

  return p_field
end




"Receives the FLOWVLM object `system` (Wing/WingSystem/Rotor), and adds vortex
particles to the particle field `pfield` at each trailing edge position where
an infinite vortex starts. `dt` indicates the length of the interval of time
that the vortex shedding represents.

Give it a previous system to detect differences in circulation and add
unsteady particles."
function VLM2VPM(system, pfield::vpm.ParticleField, dt::Float64, sigmafactor;
                  t::Float64=0.0, check=true, debug=false, tol=1e-6,
                  add_hub=false, prev_system=nothing, ms=nothing, p_per_step=1,
                  unsteady_shedcrit=0.01)

  if !(typeof(system) in [vlm.Wing, vlm.WingSystem, vlm.Rotor])
    error("Invalid system type $(typeof(system))")
  end

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
          push!(Vinfs_Ap, (Ap-prev_Ap)/dt)

          Bp = vlm.getHorseshoe(system, i)[4]
          prev_Bp = vlm.getHorseshoe(prev_system, i)[4]
          push!(Vinfs_Bp, (Bp-prev_Bp)/dt)
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
      sigma = V*dt                              # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = infD*V*dt                             # Distance the TE travels

      if add_hub
        add_particle(pfield, X, gamma, dt, V, infD, sigma, vol, sigmafactor,
                                                        l, p_per_step; ms=ms)
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
      sigma = V*dt                              # Vortex blob radius
      vol = pi*(norm(prev_Bp-prev_Ap)/2)^2*V*dt # Volume of particle
      l = infD*V*dt                             # Distance the TE travels

      add_particle(pfield, X, gamma, dt, V, infD, sigma, vol, sigmafactor,
                                                      l, p_per_step; ms=ms)

      # Adds particle at Ap
      X = Ap                                    # Particle position
      gamma = Gamma                             # Infinite vortex circulation
      V = norm(Vinfs_Ap[i])                     # Freestream at X
      infD = -infDA                             # Direction of vorticity
      sigma = V*dt                              # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = infD*V*dt                             # Distance the TE travels

      if add_hub
        add_particle(pfield, X, gamma, dt, V, infD, sigma, vol, sigmafactor,
                                                        l, p_per_step; ms=ms)
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
      sigma = V*dt                              # Vortex blob radius
      vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
      l = infD*V*dt                             # Distance the TE travels

      add_particle(pfield, X, gamma, dt, V, infD, sigma, vol, sigmafactor,
                                                      l, p_per_step; ms=ms)


      # ----------- Case of right wing tip --------------------------
      if i==m
        # Adds particle at Bp
        X = Bp                                    # Particle position
        gamma = -Gamma                            # Infinite vortex circulation
        V = norm(Vinfs_Bp[i])                     # Freestream at X
        infD = -infDB                             # Direction of vorticity
        sigma = V*dt                              # Vortex blob radius
        vol = pi*(norm(Bp-Ap)/2)^2*V*dt           # Volume of particle
        l = infD*V*dt                             # Distance the TE travels

        add_particle(pfield, X, gamma, dt, V, infD, sigma, vol, sigmafactor,
                                                        l, p_per_step; ms=ms)
      end
    end

    # Adds unsteady particle
    if prev_system!=nothing

      (p_Ap, p_A, p_B, p_Bp, p_CP,
          p_infDA, p_infDB, p_Gamma) = vlm.getHorseshoe(prev_system, i)
      X = (Ap+Bp)/2                               # LE midpoint position
      p_X = (p_Ap+p_Bp)/2                         # Previous LE midpoint position
      gamma = Gamma - p_Gamma                     # Bound circulation increase
      infD = (!cntgs || i==1 ? Ap : (prev_Ap+prev_Bp)/2) - X # Direction of circulation
      sigma = norm(B-A)                           # Vortex blob radius
      vol = 4/3*pi*(sigma/2)^3                    # Volume of particle
      l = infD*V*dt                             # Distance the TE travels

      # Adds particle only if difference is greater than 1%
      if abs(gamma/p_Gamma) > unsteady_shedcrit
        add_particle(pfield, X, gamma, 1.0, 1.0, infD, sigma, vol, sigmafactor,
                                                        l, 1; ms=ms)
      end
    end

    prev_HS = HS
  end
end


function VLM2VPM(arrsystem::Array{vlm.Rotor, 1}, pfield::vpm.ParticleField,
                                                    dt::Float64, sigmafactor;
                  prev_system=nothing, optargs...)

  if prev_system!=nothing && typeof(prev_system)!=Array{vlm.Rotor, 1}
    error("Invalid prev_system argument."
      *"Received type $(typeof(prev_system)), expected $(Array{vlm.Rotor, 1}).")
  end

  for (k, rotor) in enumerate(arrsystem)
    aux1 = (prev_system!=nothing ? prev_system[k] : nothing)
    VLM2VPM(rotor, pfield, dt, sigmafactor; prev_system=aux1, optargs...)
  end

end

function add_particle(pfield::vpm.ParticleField, X::Array{Float64, 1},
                        gamma::Float64, dt::Float64,
                        V::Float64, infD::Array{Float64, 1},
                        sigma::Float64, vol::Float64,
                        sigmafactor::Real, l::Array{T1, 1}, p_per_step::Int64;
                        ms=nothing) where {T1<:Real}
  Gamma = gamma*(V*dt)*infD       # Vectorial circulation


  if p_per_step!=1 && overwrite_sigma==nothing
      warn("Overwrite_sigma is expected to be (Vtip*dt/p_per_step) for"*
            " p_per_step!=1, got nothing")
  end


  # Decreases p_per_step for slowly moving parts of blade
  aux = min((sigma/p_per_step)/overwrite_sigma, 1)
  pps = max(1, min(p_per_step, floor(Int, 1/(1-(aux-1e-14))) ))

  if overwrite_sigma==nothing
    sigmap = sigmafactor*sigma/pps
  else
    sigmap = sigmafactor*overwrite_sigma
  end

  if ms!=nothing && sigmap<ms; sigmap = ms; end;


  # Adds p_per_step particles along line l
  dX = l/pps
  for i in 0:pps-1
      particle = vcat(X + i*dX, Gamma/pps, sigmap, vol/pps)
      vpm.addparticle(pfield, particle)
  end
end
