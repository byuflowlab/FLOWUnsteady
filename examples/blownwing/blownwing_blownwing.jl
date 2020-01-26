#=##############################################################################
# DESCRIPTION
    Blown wing with one rotor half way the midspan on each side.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Dec 2019
  * License   : MIT
=###############################################################################

function blownwing(; xfoil=true,
                        pos_bs=[-0.5, 0.5],  # Position on the semispan of every rotor
                        CWs=[false, false],  # Clock-wise rotation of every rotor
                        # OUTPUT OPTIONS
                        save_path=nothing,
                        run_name="blownwing",
                        prompt=true,
                        verbose=true, v_lvl=0
                        )

    # ------------ PARAMETERS --------------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Defining parameters..."); end;

    # Rotor parameters
    rotor_file = "apc10x7.csv"          # Rotor geometry
    data_path = uns.def_data_path       # Path to rotor database
    pitch = 0.0                         # (deg) collective pitch of blades
    n_r = 10                            # Number of blade elements
    # xfoil = false                     # Whether to run XFOIL
    # pos_bs = [-0.5, 0.5]                # Position on the semispan of every rotor
    # CWs = [false, false]                # Clock-wise rotation of every rotor
    offx = 1.25                         # Clearance to LE as a fraction of R
    init_rot = 90                       # Initial rotation


    # Read radius of this rotor and number of blades
    R, B = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]

    # Simulation parameters
    J = 0.6                             # Advance ratio Vinf/(nD)
    ReD07 = 1.5e6                       # Diameter-based Reynolds at 70% span
    ReD = ReD07/0.7                     # Diameter-based Reynolds
    rho = 1.225                         # (kg/m^3) air density
    mu = 1.81e-5                        # (kg/ms) air dynamic viscosity
    nu = mu/rho
    sound_spd = 343                     # (m/s) speed of sound
    RPM = uns.calc_RPM(ReD, J, R, 2*R, nu) # RPM
    magVinf = J*RPM/60*2*R              # (m/s) freestream velocity
    Minf = magVinf / sound_spd
    Mtip = 2*pi*RPM/60*R / sound_spd
    Rob = 0.075                         # Ratio of rotor radius over span
    b = R/Rob                           # (m) wing span
    Reb = magVinf*b/nu

    # magVinf = 163*0.3048              # (m/s) freestream
    rhoinf = rho                        # (kg/m^3) air density
    alpha = 4.2                         # (deg) angle of attack
    qinf = 0.5*rhoinf*magVinf^2         # (Pa) static pressure

    # Wing geometry
    twist = 0.0                         # (deg) root twist
    lambda = 45.0                       # (deg) sweep
    gamma = 0.0                         # (deg) Dihedral
    # b = 98*0.0254                     # (m) span
    ar = 5.0                            # Aspect ratio
    tr = 1.0                            # Taper ratio
    n_w = 4*2^3                         # Number of horseshoes
    r_w = 3.0                           # Expansion ratio of horeshoes
    n_rot = 1.75                        # Factor for HS density behind rotors

    chord = b/ar
    Rec = magVinf*chord/nu

    # Freestream function
    # Vinf(X, t) = magVinf*[cos(alpha*pi/180), 0.0, sin(alpha*pi/180)]
    # Vinf(X, t) = 1e-12*ones(3)      # (Don't make this zero or things will break)
    # Here I had to give it an initial freestream or the unsteady shedding would
    # in the first step
    Vinf(X, t) = t==0 ? magVinf*[1,0,0] : 1e-12*ones(3)


    # Solver parameters
    nrevs = 30                          # Number of revolutions in simulation
    nsteps_per_rev = 72                 # Time steps per revolution
    # nsteps_per_rev = 36
    p_per_step = 2                    # Sheds per time step
    # p_per_step = 1
    ttot = nrevs/(RPM/60)               # (s) total simulation time
    nsteps = nrevs*nsteps_per_rev       # Number of time steps
    lambda_vpm = 2.125                  # Core overlap
    overwrite_sigma = lambda_vpm * 2*pi*R/(nsteps_per_rev*p_per_step) # Smoothing core size
    nsteps_relax = 1                    # Relax VPM every this many steps
    vpm_relax = 0.3                     # VPM relaxation factor
    vpm_surface = true                  # Whether to include surfaces in the VPM
    surf_sigma = R/10                   # Smoothing radius of lifting surface
    # vlm_sigma = surf_sigma              # Smoothing radius of VLM
    vlm_sigma = -1                      # NOTE: keep this disabled or the VLM will go awry
    shed_unsteady = true                # Shed particles from unsteady loading
                                        # Max particles for memory pre-allocation
    max_particles = ((2*n_r+1)*B*length(CWs) + 2*n_w+1)*(nrevs*nsteps_per_rev+1)*p_per_step
    plot_disc = false                   # Plot blade discretization for debugging
    wake_coupled = true                 # Coupled VLM solver with VPM
    vlm_rlx = 0.75                      # VLM relaxation (deactivated with -1)
    # vlm_rlx = 0.0
    # vlm_init = true                     # Initialize with the VLM semi-infinite wake solution
    vlm_init = false

    if verbose
        println("\t"^(v_lvl+2)*"J:\t\t$(J)")
        println("\t"^(v_lvl+2)*"ReD07:\t\t$(ReD07)")
        println("\t"^(v_lvl+2)*"RPM:\t\t$(ceil(Int, RPM))")
        println("\t"^(v_lvl+2)*"Mtip:\t\t$(round(Mtip, 3))")
        println("\t"^(v_lvl+2)*"Minf:\t\t$(round(Minf, 3))")
        println("\t"^(v_lvl+2)*"R/b:\t\t$(round(Rob, 3))")
        println("\t"^(v_lvl+2)*"b:\t\t$(round(b, 2)) m")
        println("\t"^(v_lvl+2)*"c:\t\t$(round(chord, 2)) m")
        println("\t"^(v_lvl+2)*"Reb:\t\t$(ceil(Int, Reb))")
        println("\t"^(v_lvl+2)*"Rec:\t\t$(ceil(Int, Rec))")
        println("\t"^(v_lvl+2)*"Solution time: "*
            "$(round(magVinf*nrevs/(RPM/60)/b, 3)) spans ($nrevs revolutions)")
        println("\t"^(v_lvl+2)*"vlm_fsgm:\t\t$(round(vlm_sigma/b, 6))")
    end

    # ------------ GENERATING GEOMETRY -----------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating geometry..."); end;

    if length(pos_bs) != length(CWs)
        error("Invalid rotor definition: length(pos_bs) != length(CWs)")
    end

    # Generate clockwise rotor
    rotor_cw = uns.generate_rotor(rotor_file; pitch=pitch,
                                            n=n_r, CW=true, ReD=ReD,
                                            verbose=verbose, xfoil=xfoil,
                                            data_path=data_path,
                                            plot_disc=plot_disc,
                                            verbose=verbose, v_lvl=v_lvl+1)
    # Generate counter-clockwise rotor
    rotor_ccw = vlm.Rotor(!rotor_cw.CW, rotor_cw.r, rotor_cw.chord,
                                            rotor_cw.theta, rotor_cw.LE_x,
                                            rotor_cw.LE_z, rotor_cw.B,
                                            rotor_cw.airfoils)
    vlm.initialize(rotor_ccw, rotor_cw.m)

    base_props = [rotor_cw, rotor_ccw]      # Base rotors

    # Generate the actual rotors to use along wing
    rotors = vlm.Rotor[]
    # warn("Rotors disabled!")
    # pos_bs = []
    # CWs = []
    for (i, pos) in enumerate(pos_bs)
        # Rotor geometry
        copy_prop = base_props[2^!CWs[i]]
        this_prop = deepcopy(copy_prop)
        # Position
        x = abs(pos)*b/2*tan(pi/180*lambda) - offx*R
        y = pos*b/2
        z = abs(pos)*b/2*tan(pi/180*gamma)
        # Account for angle of attack of wing
        nrm = norm([x,z])
        x = nrm*cos(pi/180*alpha)
        z = -nrm*sin(pi/180*alpha)
        this_O = [x, y, z]
        # Orientation
        this_Oaxis = Float64[1 0 0; 0 1 0; 0 0 1]

        vlm.setcoordsystem(this_prop, this_O, this_Oaxis; user=true)

        # Rotates props
        vlm.rotate(this_prop, (-1)^(!CWs[i]) * init_rot)

        # Adds the original polars that don't get copied in deepcopy
        this_prop.airfoils = copy_prop.airfoils
        this_prop._polars = copy_prop._polars
        this_prop._polarroot = copy_prop._polarroot
        this_prop._polartip = copy_prop._polartip

        push!(rotors, this_prop)
    end

    # Wing discretization
    _pos_bs = unique(abs.(pos_bs))
    if length(pos_bs)!=0 && length(pos_bs)!=1 && length(pos_bs)/length(_pos_bs)!=2
        error("Logic error: Position of rotors is expected to be symmetric.")
    end
    refinement = []
    for pos_b in _pos_bs
        # Add wing section before rotor
        pos = pos_b - R/(b/2)                   # Position along semi span
        if pos > 0                              # Length of segment
            len = pos - (length(refinement)!=0 ? refinement[end][1] : 0)
            nfrac = len                         # fraction of horseshoes
            if length(refinement) != 0
                push!(refinement, [len/2, nfrac/2, r_w])
                push!(refinement, [len/2, nfrac/2, 1/r_w])
            else
                push!(refinement, [len, nfrac, 1/r_w])
            end
        end

        # Add rotor section
        pos = min(pos_b + R/(b/2), 1.0)
        len = pos - (length(refinement)!=0 ? refinement[end][1] : 0)
        nfrac = len
        push!(refinement, [len, n_rot*nfrac, 1.01])
    end
    if length(refinement) == 0
        push!(refinement, [1.0, 1.0, 1/r_w])
    end
    # Add tip
    sumc = sum(ref[1] for ref in refinement)
    if abs(sumc-1.0) > 0.0001
        len = abs(sumc-1)
        nfrac = 1.5*len
        push!(refinement, [len/2, nfrac/2, r_w])
        push!(refinement, [len/2, nfrac/2, 1/r_w])
    end

    # Generate wing
    wing = vlm.simpleWing(b, ar, tr, twist, lambda, gamma;
                                                n=n_w, refinement=refinement)

    # Pitch wing to corresponding angle of attack
    O = zeros(3)                                    # Coordinate system origin
    Oaxis = gt.rotation_matrix2(0.0, -alpha, 0.0)   # Coordinate system axes
    vlm.setcoordsystem(wing, O, Oaxis)


    # ------------- SIMULATION SETUP -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Simulation setup..."); end;

    # ----- VEHICLE DEFINITION
    # System of all FLOWVLM objects
    system = vlm.WingSystem()
    vlm.addwing(system, "Wing", wing)
    for (ri, rotor) in enumerate(rotors)
        vlm.addwing(system, "Rotor$ri", rotor)
    end

    # Systems of rotors
    rotor_systems = (rotors, )

    # System solved through VLM solver
    vlm_system = vlm.WingSystem()
    vlm.addwing(vlm_system, "Wing", wing)

    # Wake-shedding system
    wake_system = system

    # FVS's Vehicle object
    vehicle = uns.VLMVehicle(   system;
                                vlm_system=vlm_system,
                                rotor_systems=rotor_systems,
                                wake_system=wake_system
                             )

    # ----- MANEUVER DEFINITION
    RPM_fun(t) = 1.0                # RPM (normalized by reference RPM) as a
                                    # function of normalized time

    angle = ()                      # Angle of each tilting system (none in this case)
    sysRPM = (RPM_fun, )            # RPM of each rotor system
    Vvehicle(t) = [-1,0,0]          # Translational velocity of vehicle over Vref
    anglevehicle(t) = zeros(3)      # (deg) angle of the vehicle

    Vref = magVinf                  # (m/s) aircraft velocity during cruise
    RPMref = RPM                    # Reference RPM

    # FVS's Maneuver object
    maneuver = uns.KinematicManeuver(angle, sysRPM, Vvehicle, anglevehicle)

    # Plot maneuver path and controls
    uns.plot_maneuver(maneuver; vis_nsteps=nsteps)

    # ----- SIMULATION DEFINITION
    Vinit = Vref*Vvehicle(0)       # Initial vehicle velocity
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot; Vinit=Vinit)

    monitor_rotor = generate_monitor_prop(J, rho, RPM, nsteps; save_path=save_path,
                                                    run_name=run_name*"_rotors")
    monitor_wing = generate_monitor_wing(wing, b, ar, nsteps, Vinf, rhoinf, qinf,
                                                    magVinf, wake_coupled;
                                                    save_path=save_path,
                                                    run_name=run_name*"_wing")
    monitor(args...) = monitor_rotor(args...) || monitor_wing(args...)

    # ------------- RUN SIMULATION ---------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Running simulation..."); end;
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      vpm_surface=vpm_surface,
                                      surf_sigma=surf_sigma,
                                      vlm_init=vlm_init,
                                      vlm_rlx=vlm_rlx,
                                      max_particles=max_particles,
                                      nsteps_relax=nsteps_relax,
                                      relaxfactor=vpm_relax,
                                      wake_coupled=wake_coupled,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl+1,
                                      save_code=splitdir(@__FILE__)[1],
                                      save_horseshoes=false
                                      )
    return pfield, wing, rotors
end
