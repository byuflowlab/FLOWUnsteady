#=##############################################################################
# DESCRIPTION
    Geometry generation.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


#=

Geometry functions (e.g., `generategeometry_vahana`) are expected to:

    OUTPUTS
    * `system::vlm.WingSystem`:         System of all FLOWVLM objects.
    * `rotors::Array{vlm.Rotor, 1}`:    System of all FLOWVLM Rotor objects.
    * `tilting_systems::Tuple(vlm.WingSystem, ...)`:   Tuple of all FLOWVLM
                                        tilting objects, where
                                        `tilting_systems[i]` contains the i-th
                                        FLOWVLM system of lifting surfaces and
                                        rotors that tilt together.
    * `rotors_tilting_systems::Tuple(Array{vlm.Rotor,1}, ...)`:   Tuple of all
                                        FLOWVLM Rotor tilting objects, where
                                        `rotors_ tilting_systems[i]` contains
                                        the rotors associated to the i-th
                                        tilting system.
    * `fuselage::gt.Grid`:              Grid object of the fuselage.
    * `grounds::gt.Grid`:               Grid object of the ground.
    * `strn::String`                    String containing all vtk file names
                                        that will be generated if `system` is
                                        saved.

    NOTES
    * Maneuver functions are expected to return `(Vaircraft, angles, RPMs)`,
        where `Vaircraft(t)` is the translation velocity of both `system`
        and `fuselage`, `angles(t)[i]` is the tilt angle of
        `tilting_systems[i]`, `RPMs(t)[i]` is the RPM of
        `rotors_tilting_systems[i]`. `angles(t)[end]` is the tilt angle of the
        entire aircraft.
=#


################################################################################
# VAHANA
################################################################################
#=

Unsteady simulation of eVTOL transition maneuver.

REFERENCES
* Vahana geometry: Droandi, G., Syal, M., and Bower, G., “Tiltwing Multi-Rotor
Aerodynamic Modeling in Hover, Transition and Cruise Flight Conditions,” AHS
International 74th Annual Forum & Technology Display, 2018, p. 2018.
=#

"""
    Generates the geometry of Vahana aircraft
"""
function generategeometry_vahana(;
                                    # AIRCRAFT OPTIONS
                                    rotor_file="apc10x7_vahana.jl", # Rotor
                                    # rotor_file="ning_vahana.jl",
                                    data_path=data_path,
                                    xfoil=true,                     # Run XFOIL
                                    n_factor::Int=1,                # Refinement factor
                                    # OUTPUT OPTIONS
                                    run_name="vahana",
                                    verbose=true, v_lvl=0)

    ############################################################################
    # PARAMETERS
    ############################################################################

    if verbose; println("\t"^(v_lvl)*"Defining parameters..."); end;

    # ------------ GEOMETRIC PARAMETERS ------------------------------------
    init_ori = 90.0                 # Initial orientation of wings
    init_ori_prop = 0*90.0            # Initial orientation of rotors

    # Rotors
    # NOTE: The radii defined here must match the radius in rotor_file
    R_w = 0.75                    # (m) main wing rotor radius
    R_tw = R_w                    # (m) tandem wing rotor radius
    CW_w = true                   # Clockwise main wing rotor
    CW_tw = false                 # Clockwise tandem wing rotor
    np_w = 2                      # Number of propellers on wing on each side
    np_tw = 2                     # Number of propellers on tandem wing each side
    n_ccb = 7*n_factor            # Number of CCBlade elements
    ReD07 = 1.5e6                 # Prop Reynolds number of relative velocity at r/R=0.7
    ReD = ReD07/0.7               # Reynolds at tip
    pitch = 0.0                   # (deg) pitch of propellers
    soD = 0.1                     # Tip-to-tip distance of rotors over diameter
    xoc_offset = 0.1              # Axial distance of rotors from LE over chord

    # Wing
    b_w = 5.86                    # (m) span
    AR_w = 7.4                    # Aspect ratio
    tr_w = 1.0                    # Taper ratio
    twist_r_w = 7.5               # (deg) twist at root
    twist_t_w = twist_r_w         # (deg) twist at tip
    lambda_w = 0.0                # (deg) sweep
    gamma_w = 0.0                 # (deg) dihedral
    n_w = 12*n_factor             # Number of horseshoes per side of wing
    r_w = 2.0                     # Horseshoe expansion ratio
    md_w = 0.2                    # Ratio of length of middle section
    pivot_w = 1/4                 # Pivot point along chord of tilt-wing

    # Wing winglets
    b_wl = b_w/4                  # (m) span of winglet from top to bottom
    AR_wl = 3.0                   # Aspect ratio
    tr_wl = (b_wl/AR_wl)/(b_w/AR_w/tr_w)      # Taper ratio
    twist_r_wl = 2.5              # (deg) twist at root
    twist_t_wl = 0.0              # (deg) twist at tip
    lambda_wl = 40.0              # (deg) sweep
    gamma_wl = 15.0               # (deg) dihedral
    n_wl = 4*n_factor             # Number of horseshoes per side of wing
    r_wl = 2.0                    # Horseshoe expansion ratio

    # Tandem wing
    b_tw = b_w*1.0                # (m) span
    AR_tw = 9.5                   # Aspect ratio
    tr_tw = 1.0                   # Taper ratio
    twist_r_tw = 4.0              # (deg) twist at root
    twist_t_tw = twist_r_tw       # (deg) twist at tip
    lambda_tw = 0.0               # (deg) sweep
    gamma_tw = 0.0                # (deg) dihedral
    n_tw = n_w                    # Number of horseshoes per side of wing
    r_tw = r_w                    # Horseshoe expansion ratio
    md_tw = md_w                  # Ratio of length of middle section
    pivot_tw = pivot_w            # Pivot point along chord of tilt-wing

    # Fuselage
    l_f = 5.86                    # (m) length
    h_f = 2.81*2/3                # (m) height
    c_pos_f1 = 0                  # (m) span position of first chord
    c_pos_f2 = 0.2*h_f            # (m) span position of second chord
    c_pos_f3 = 0.7*h_f
    c_pos_f4 = h_f
    c_f1 = 0.66*l_f               # (m) length of first chord
    c_f2 = 0.75*l_f
    c_f3 = 0.75*l_f
    c_f4 = 0.50*l_f
    x_pos_f1 = 0.025*l_f          # (m) x-position of first chord
    x_pos_f2 = 0.0*l_f
    x_pos_f3 = 0.20*l_f
    x_pos_f4 = 0.50*l_f

    # ------------ ASSEMBLY PARAMETERS ------------------------------------
    # Position of wings on fuselage
    h_pos_w = 0.90*h_f            # (m) height position of wing
    h_pos_tw = 0.15*h_f           # (m) height position of tandem wing
    l_pos_w = 0.95*l_f-b_w/AR_w/tr_w   # (m) length position of wing
    l_pos_tw = 0.05*l_f           # (m) length position of tandem wing

    # Position of propellers on main wing
    # d_prop_w = d_factor*b_w/2/np_w# Distance between propellers on wing
    d_prop_w = (1+soD)*(2*R_w)
    y_pos_prop_w = Float64[b_w/2 - i*d_prop_w for i in 0:np_w-1] # y-positions

    # Position of propellers on tandem wing
    # d_prop_tw = d_factor*b_tw/2/np_tw# Distance between propellers on wing
    d_prop_tw = (1+soD)*(2*R_tw)
    y_pos_prop_tw = Float64[b_tw/2 - i*d_prop_tw for i in 0:np_tw-1] # y-positions


    ############################################################################
    # ASSEMBLY
    ############################################################################

    if verbose; println("\t"^(v_lvl)*"Generating geometry..."); end;

    # ------------ ROTORS ------------------------------------------------
    # Generates base propellers (one on each rotation orientation)
    propellers = vlm.Rotor[]
    if verbose; println("\t"^(v_lvl+1)*"Generating first propeller..."); end;
    @time push!(propellers, generate_rotor(pitch; n=n_ccb, CW=!CW_w, ReD=ReD,
                          verbose=verbose, xfoil=xfoil, rotor_file=rotor_file,
                          data_path=data_path, plot_disc=false))
    if verbose; println("\t"^(v_lvl+1)*"Generating second propeller..."); end;
    # @time push!(propellers, generate_rotor(pitch; n=n_ccb, CW=CW_w, ReD=ReD,
    #                         verbose=verbose, xfoil=xfoil, rotor_file=rotor_file))
    @time push!(propellers, vlm.Rotor(!propellers[1].CW, propellers[1].r,
                              propellers[1].chord, propellers[1].theta,
                              propellers[1].LE_x, propellers[1].LE_z,
                              propellers[1].B, propellers[1].airfoils))
    @time vlm.initialize(propellers[2], propellers[1].m)


    # ------------ MAIN WING ---------------------------------------------
    # Generates wing
    if verbose; println("\t"^(v_lvl+1)*"Generating main wing assembly..."); end;

    # wing = vlm.simpleWing(b_w, AR_w, tr_w, twist_r_w, lambda_w, gamma_w;
    #                                           twist_tip=twist_t_w, n=n_w, r=r_w)

    # Middle section
    # tip = b/ar
    # root = tip/tr
    # newtip = root + md*(tip-root) = tip/tr + md*tip*(1 - 1/tr) = tip * ( md + (1/tr)*(1-md) )
    # newtr = newtip/root = 1 + md*(tip/root - 1) = 1 + md*(tr - 1)
    # newar = b/newtip = ar / ( md + (1/tr)*(1-md) )
    pos_md_w = [-md_w, 0.0, md_w]
    clen_md_w = [md_w + (1/tr_w)*(1-md_w), 1/tr_w, md_w + (1/tr_w)*(1-md_w)]
    twist_md_w = [twist_r_w + md_w*(twist_t_w-twist_r_w), twist_r_w, twist_r_w + md_w*(twist_t_w-twist_r_w)]
    wing_md = vlm.complexWing(b_w, AR_w, ceil(Int, md_w*n_w), pos_md_w, clen_md_w, twist_md_w,
                          lambda_w*ones(2), gamma_w*ones(2);
                          symmetric=false, chordalign=0.0,
                          _ign1=true)
    # Left section
    pos_l_w = [-1, pos_md_w[1]]
    clen_l_w = [1, clen_md_w[1]]
    twist_l_w = [twist_t_w, twist_md_w[1]]
    wing_L = vlm.complexWing(b_w, AR_w, ceil(Int, (1-md_w)*n_w/2), pos_l_w, clen_l_w,
                            twist_l_w, lambda_w*ones(1), gamma_w*ones(1);
                            symmetric=false, chordalign=0.0,
                            _ign1=true)
    # Right section
    pos_r_w = [pos_md_w[end], 1]
    clen_r_w = [clen_md_w[end], 1]
    twist_r_w = [twist_md_w[end], twist_t_w]
    wing_R = vlm.complexWing(b_w, AR_w, ceil(Int, (1-md_w)*n_w/2), pos_r_w, clen_r_w,
                            twist_r_w, lambda_w*ones(1), gamma_w*ones(1);
                            symmetric=false, chordalign=0.0,
                            _ign1=true)

    # Generates winglets
    winglet_R = vlm.simpleWing(b_wl, AR_wl, tr_wl, twist_r_wl, lambda_wl, gamma_wl;
                                          twist_tip=twist_t_wl, n=n_wl, r=r_wl)
    winglet_L = vlm.simpleWing(b_wl, AR_wl, tr_wl, twist_r_wl, lambda_wl, gamma_wl;
                                          twist_tip=twist_t_wl, n=n_wl, r=r_wl)
    O_wl_R = (b_w/2)*[tan(lambda_w*pi/180), 1, tan(gamma_w*pi/180)]
    O_wl_L = [1 0 0; 0 -1 0; 0 0 1]*O_wl_R
    Oaxis_wl_R = vlm.vtk.rotation_matrix(0.0, 0.0, 90.0)
    Oaxis_wl_L = vlm.vtk.rotation_matrix(0.0, 0.0, -90.0)
    vlm.setcoordsystem(winglet_R, O_wl_R, Oaxis_wl_R)
    vlm.setcoordsystem(winglet_L, O_wl_L, Oaxis_wl_L)

    ## Generates propellers on wing (from right to left)
    if verbose; println("\t"^(v_lvl+2)*"Generating main wing propellers..."); end;
    O_prop_w = [ ypos*[tan(lambda_w*pi/180), 1, tan(gamma_w*pi/180)] + [-xoc_offset*AR_w/b_w, 0, 0]
                                              for ypos in y_pos_prop_w]
    props_w = vlm.Rotor[]
    for i in 1:2*np_w
        right = i<=np_w    # Indicates wich side of the wing
        copy_prop = propellers[1+i%2]
        this_prop = deepcopy(copy_prop) # Alternates rotation orientation
        this_O = O_prop_w[ right ? i : np_w-(i-np_w-1)] # Chooses position
        this_O = [1 0 0; 0 (-1)^!right 0; 0 0 1]*this_O   # Places it in correct side

        vlm.setcoordsystem(this_prop, this_O, Float64[1 0 0; 0 1 0; 0 0 1]; user=true)

        # Rotates props to be tip to tip
        vlm.rotate(this_prop, (-1)^(!CW_w) * init_ori_prop)

        # Adds the original polars that don't get copied in deepcopy
        this_prop.airfoils = copy_prop.airfoils
        this_prop._polars = copy_prop._polars
        this_prop._polarroot = copy_prop._polarroot
        this_prop._polartip = copy_prop._polartip

        push!(props_w, this_prop)
    end

    # Assembles moving sections of the wing
    main_wing_moving = vlm.WingSystem()
    vlm.addwing(main_wing_moving, "WingR", wing_R)
    vlm.addwing(main_wing_moving, "WingletR", winglet_R)
    vlm.addwing(main_wing_moving, "WingL", wing_L)
    vlm.addwing(main_wing_moving, "WingletL", winglet_L)
    for (i, prop) in enumerate(props_w)
        vlm.addwing(main_wing_moving, "Prop$i", prop)
    end

    # offset to align with pivot line
    x_off_w = pivot_w*b_w/AR_w

    # Aligns moving and fixed section with their pivot line
    O_off_w = [-x_off_w, 0.0, 0.0]

    vlm.setcoordsystem(wing_md, O_off_w, eye(3))

    for vlmwing in main_wing_moving.wings
        if typeof(vlmwing)==vlm.Rotor
            vlm.setcoordsystem(vlmwing, vlmwing._wingsystem.O + O_off_w,
                                        vlmwing._wingsystem.Oaxis; user=false)
        else
            vlm.setcoordsystem(vlmwing, vlmwing.O + O_off_w, vlmwing.Oaxis)
        end
    end

    # Initial rotation of moving sections
    Oaxis_wmv = vlm.vtk.rotation_matrix(0.0, -init_ori, 0.0)
    vlm.setcoordsystem(main_wing_moving, zeros(3), Oaxis_wmv)

    # Assembles main wing
    main_wing = vlm.WingSystem()
    vlm.addwing(main_wing, "FixedWing", wing_md)
    vlm.addwing(main_wing, "Moving", main_wing_moving)

    # Position of main wing
    O_w = [l_pos_w + x_off_w, 0, h_pos_w]
    Oaxis_w = vlm.vtk.rotation_matrix(0.0, 0.0, 0.0)
    vlm.setcoordsystem(main_wing, O_w, Oaxis_w)

    # ------------ TANDEM WING -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating tandem wing assembly..."); end;

    # Generates tandem wing
    twing = vlm.simpleWing(b_tw, AR_tw, tr_tw, twist_r_tw, lambda_tw,
                                gamma_tw; twist_tip=twist_t_tw, n=n_tw, r=r_tw)

    # Middle section
    pos_md_tw = [-md_tw, 0.0, md_tw]
    clen_md_tw = [md_tw + (1/tr_tw)*(1-md_tw), 1/tr_tw,
                                                   md_tw + (1/tr_tw)*(1-md_tw)]
    twist_md_tw = [twist_r_tw + md_tw*(twist_t_tw-twist_r_tw),
                        twist_r_tw, twist_r_tw + md_tw*(twist_t_tw-twist_r_tw)]
    twing_md = vlm.complexWing(b_tw, AR_tw, ceil(Int, md_tw*n_tw), pos_md_tw,
                                  clen_md_tw, twist_md_tw,
                                  lambda_tw*ones(2), gamma_tw*ones(2);
                                  symmetric=false, chordalign=0.0,
                                  _ign1=true)
    # Left section
    pos_l_tw = [-1, pos_md_tw[1]]
    clen_l_tw = [1, clen_md_tw[1]]
    twist_l_tw = [twist_t_tw, twist_md_tw[1]]
    twing_L = vlm.complexWing(b_tw, AR_tw, ceil(Int, (1-md_tw)*n_tw/2), pos_l_tw,
                                    clen_l_tw, twist_l_tw,
                                    lambda_tw*ones(1), gamma_tw*ones(1);
                                    symmetric=false, chordalign=0.0,
                                    _ign1=true)
    # Right section
    pos_r_tw = [pos_md_tw[end], 1]
    clen_r_tw = [clen_md_tw[end], 1]
    twist_r_tw = [twist_md_tw[end], twist_t_tw]
    twing_R = vlm.complexWing(b_tw, AR_tw, ceil(Int, (1-md_tw)*n_tw/2), pos_r_tw,
                                    clen_r_tw, twist_r_tw,
                                    lambda_tw*ones(1), gamma_tw*ones(1);
                                    symmetric=false, chordalign=0.0,
                                    _ign1=true)

    ## Generates propellers on tandem wing (from right to left)
    if verbose; println("\t"^(v_lvl+2)*"Generating tandem wing propellers..."); end;
    O_prop_tw = [ ypos*[tan(lambda_tw*pi/180), 1, tan(gamma_tw*pi/180)] + [-xoc_offset*AR_tw/b_tw, 0, 0]
                                              for ypos in y_pos_prop_tw]
    props_tw = vlm.Rotor[]
    for i in 1:2*np_tw
        right = i<=np_tw    # Indicates wich side of the wing
        copy_prop = propellers[1+(i+(CW_tw!=CW_w))%2]
        this_prop = deepcopy(copy_prop)       # Alternates rotation orientation
        this_O = O_prop_tw[ right ? i : np_tw-(i-np_tw-1)] # Chooses position
        this_O = [1 0 0; 0 (-1)^!right 0; 0 0 1]*this_O   # Places it in correct side

        vlm.setcoordsystem(this_prop, this_O, Float64[1 0 0; 0 1 0; 0 0 1]; user=true)

        # Rotates props to be tip to tip
        vlm.rotate(this_prop, (-1)^(!CW_tw) * init_ori_prop)

        # Adds the original polars that don't get copied in deepcopy
        this_prop.airfoils = copy_prop.airfoils
        this_prop._polars = copy_prop._polars
        this_prop._polarroot = copy_prop._polarroot
        this_prop._polartip = copy_prop._polartip

        push!(props_tw, this_prop)
    end

    # Assembles moving sections of the wing
    tandem_wing_moving = vlm.WingSystem()
    vlm.addwing(tandem_wing_moving, "WingR", twing_R)
    vlm.addwing(tandem_wing_moving, "WingL", twing_L)
    for (i, prop) in enumerate(props_tw)
        vlm.addwing(tandem_wing_moving, "Prop$i", prop)
    end

    # offset to align with pivot line
    x_off_tw = pivot_tw*b_tw/AR_tw

    # Aligns moving and fixed section with their pivot line
    O_off_tw = [-x_off_tw, 0.0, 0.0]

    vlm.setcoordsystem(twing_md, O_off_tw, eye(3))

    for vlmwing in tandem_wing_moving.wings
        if typeof(vlmwing)==vlm.Rotor
            vlm.setcoordsystem(vlmwing, vlmwing._wingsystem.O + O_off_tw,
                                        vlmwing._wingsystem.Oaxis; user=false)
        else
            vlm.setcoordsystem(vlmwing, vlmwing.O + O_off_tw, vlmwing.Oaxis)
        end
    end

    # Initial rotation of moving sections
    Oaxis_twmv = vlm.vtk.rotation_matrix(0.0, -init_ori, 0.0)
    vlm.setcoordsystem(tandem_wing_moving, zeros(3), Oaxis_twmv)

    # Assembles tandem wing
    tandem_wing = vlm.WingSystem()
    vlm.addwing(tandem_wing, "FixedWing", twing_md)
    vlm.addwing(tandem_wing, "Moving", tandem_wing_moving)

    # Position of tandem wing
    O_tw = [l_pos_tw + x_off_tw, 0, h_pos_tw]
    Oaxis_tw = vlm.vtk.rotation_matrix(0.0, 0.0, 0.0)
    vlm.setcoordsystem(tandem_wing, O_tw, Oaxis_tw)

    # ------------ FUSELAGE ----------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating fuselage..."); end;
    # Generates fuselage
    fuselage = vlm.Wing(x_pos_f1, c_pos_f1, 0.0, c_f1, 0.0)
    vlm.addchord(fuselage, x_pos_f2, c_pos_f2, 0.0, c_f2, 0.0, 1)
    vlm.addchord(fuselage, x_pos_f3, c_pos_f3, 0.0, c_f3, 0.0, 1)
    vlm.addchord(fuselage, x_pos_f4, c_pos_f4, 0.0, c_f4, 0.0, 1)
    Oaxis_f = vlm.vtk.rotation_matrix(0.0, 0.0, -90.0)
    vlm.setcoordsystem(fuselage, zeros(Float64,3), Oaxis_f)

    fuselage = generatefuselage_vahana(l_f)


    # ------------ SYSTEM ------------------------------------------------
    # Creates system assembly
    system = vlm.WingSystem()
    vlm.addwing(system, "MainWing", main_wing)
    vlm.addwing(system, "TandemWing", tandem_wing)
    # vlm.addwing(system, "Fuselage", fuselage)

    rotors = vcat(props_w, props_tw)

    # ------------ GROUND SURFACE ----------------------------------------
    ground1 = generate_ground_vahana(l_f)          # Take-off pad
    ground2 = generate_ground_vahana(l_f)          # Landing pad

    grounds = (ground1, ground2)


    # ------------ OUTPUTS------------------------------------------------

    # Creates string of vtk components
    save_name = run_name
    strn = ""
    strn = strn * save_name * "_MainWing_FixedWing_vlm.vtk;"
    strn = strn * save_name * "_MainWing_Moving_WingR_vlm.vtk;"
    strn = strn * save_name * "_MainWing_Moving_WingletR_vlm.vtk;"
    strn = strn * save_name * "_MainWing_Moving_WingL_vlm.vtk;"
    strn = strn * save_name * "_MainWing_Moving_WingletL_vlm.vtk;"
    strn = strn * save_name * "_TandemWing_FixedWing_vlm.vtk;"
    strn = strn * save_name * "_TandemWing_Moving_WingR_vlm.vtk;"
    strn = strn * save_name * "_TandemWing_Moving_WingL_vlm.vtk;"
    # strn = strn * save_name * "_Fuselage_vlm.vtk;"
    num_blades = propellers[1].B
    for (sys_name, num_props) in [("MainWing_Moving", np_w*2), ("TandemWing_Moving", np_tw*2)]
        for j in 1:num_props
            for i in 1:num_blades
                # strn = strn * save_name * "_" * sys_name * "_Prop$j" * "_Blade$(i)_vlm.vtk;"
            end
            for i in 1:num_blades
                strn = strn * save_name * "_" * sys_name * "_Prop$j" * "_Blade$(i)_loft.vtk;"
            end
        end
    end


    tilting_systems = (main_wing_moving, tandem_wing_moving)
    rotors_tilting_systems = (props_w, props_tw)

    return (system, rotors, tilting_systems, rotors_tilting_systems,
                                                        fuselage, grounds, strn)
end

"""
    Returns a loft of the Vahana fuselage.
"""
function generatefuselage_vahana(fuselage_length; ncells=20)
    sections = [[] for i in 1:6]

    # --------------- SECTION #1------------------------------------------------
    Ps = [
            [0.5, 0.1+0.2],
            [0.0, 1.0],
            [-0.5, 0.1+0.2],
            [0, -0.20+0.2]
         ]

    CPs = [
            [0.4, 1.0],
            [-0.4, 1.0],
            [-0.1, -0.3+0.2],
            [0.1, -0.3+0.2],
         ]

    rhos = [
            0.6,
            0.6,
            0.2,
            0.2
         ]

    scaling = 1.0


    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]

    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[1] = points


    # --------------- SECTION #2------------------------------------------------
    Ps = [
            [0.4, 0.4],
            [0.0, 1.0],
            [-0.4, 0.4],
            [0, -0.05]
         ]

    CPs = [
            [0.3, 1.0],
            [-0.3, 1.0],
            [-0.4, -0.08],
            [0.4, -0.08],
         ]

    rhos = [
            0.7,
            0.7,
            0.5,
            0.5
         ]

    scaling = 2.0

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]

    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[2] = points;


    # --------------- SECTION #3------------------------------------------------
    Ps = [
            [0.8, 1.0],
            [0.0, 2.5],
            [-0.8, 1.0],
            [0, 0.2]
         ]

    CPs = [
            [0.6, 2.5],
            [-0.6, 2.5],
            [-0.8, 0.2],
            [0.8, 0.2],
         ]

    rhos = [
            0.7,
            0.7,
            0.5,
            0.5
         ]

    scaling = 1.0

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]

    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[3] = points;


    # --------------- SECTION #4------------------------------------------------
    Ps = [
            [0.7, 2.0+0.25],
            [0.0, 3.0+0.25],
            [-0.7, 2.0+0.25],
            [0, 1.2]
         ]

    CPs = [
            [0.5, 3.0+0.25],
            [-0.5, 3.0+0.25],
            [-0.8, 1.2],
            [0.8, 1.2],
         ]

    rhos = [
            0.7,
            0.7,
            0.5,
            0.5
         ]

    scaling = 1.0

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]

    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[4] = points;


    # --------------- SECTION #5------------------------------------------------
    Ps = [
            [0.4, 2.75-0.2],
            [0.0, 3.25-0.2],
            [-0.4, 2.75-0.2],
            [0, 1.7]
         ]

    CPs = [
            [0.3, 3.25-0.2],
            [-0.3, 3.25-0.2],
            [-0.4, 1.7],
            [0.4, 1.7],
         ]

    rhos = [
            0.4,
            0.4,
            0.5,
            0.5
         ]

    scaling = 1.0

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]

    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[5] = points;


    # --------------- SECTION #6------------------------------------------------
    Ps = [
            [0.4-0.2, 3.25],
            [0.0, 3.5-0.1],
            [-0.4+0.2, 3.25],
            [0, 2.5+0.5]
         ]

    CPs = [
            [0.3-0.2, 3.5-0.1],
            [-0.3+0.2, 3.5-0.1],
            [-0.4, 2.5+0.5],
            [0.4, 2.5+0.5],
         ]

    rhos = [
            0.4,
            0.4,
            0.3,
            0.3
         ]

    scaling = 1.0

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]
    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[6] = points;

    # --------------- SECTION #0------------------------------------------------
    Ps = [
            [0.5, 0.1+0.2],
            [0.0, 1.0],
            [-0.5, 0.1+0.2],
            [0, -0.20+0.2]
         ]

    CPs = [
            [0.4, 1.0],
            [-0.4, 1.0],
            [-0.1, -0.3+0.2],
            [0.1, -0.3+0.2],
         ]

    rhos = [
            0.6,
            0.6,
            0.2,
            0.2
         ]

    scaling = 0.1

    ss = [
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells)),
            collect(linspace(0, 1, ncells))
         ]
    points = vlm.vtk.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    section0 = points


    # --------------- LOFT -----------------------------------------------------
    # fuselage_length = 5.86              # (m) fuselage length
    bscale = fuselage_length            # Scales the entire geometry by this factor

    crosssections = [ # (l/bscale position , contour)
                        (0.0  , [[2.00*0.8, 0.25][k]*(point[k]) + [0, 0.3][k] for point in section0, k in 1:2]),
                        (0.02, [[0.6*0.8, 0.3][k]*(point[k]) + [0, 0.15][k] for point in sections[1], k in 1:2]),
                        (1/6*0.5, [[1.0*0.8, 0.75][k]*point[k] for point in sections[1], k in 1:2]),
    #                     (1/6*0.5, [[0.75, 0.75][k]*point[k] for point in sections[1], k in 1:2]),
                        (2/6, [[1.00*0.8, 0.75][k]*point[k] for point in sections[2], k in 1:2]),
                        (3/6, [[1.00*0.8, 0.75][k]*(point[k] - [0, 0.1*2][k]) for point in sections[3], k in 1:2]),
                        (4/6, [[0.75, 0.75][k]*(point[k] - [0, 0.4*2][k]) for point in sections[4], k in 1:2]),
                        (5/6, [[0.75, 0.75][k]*(point[k] - [0, 0.6][k]) for point in sections[5], k in 1:2]),
                        (6/6, [[0.75, 0.75][k]*(point[k] - [0, 1.0][k]) for point in sections[6], k in 1:2]),
                    ]

    # Dummy parameters
    b_pos = [sec[1] for sec in crosssections]
    chords = (1/bscale)*ones(b_pos)
    twists = zeros(b_pos)
    LE_x = zeros(b_pos)
    LE_z = zeros(b_pos)

    tilt_y = zeros(b_pos)
    # tilt_y[2] = 45

    grid_fuselage = vlm.vtk.generate_loft(crosssections, bscale,
                                        b_pos, chords, twists, LE_x, LE_z;
                                        # MORE GEOMETRIC OPTIONS
                                        tilt_y=tilt_y,
                                        symmetric=false,
                                        loop_dim=0,
                                    )

    # Aligns the grid with the x-axis
    rotation = vlm.vtk.rotation_matrix2(0, 0, 90)
    vlm.vtk.lintransform!(grid_fuselage, rotation, zeros(3))

    dimsplit = 1
    triangrid_fuselage = vlm.vtk.GridTriangleSurface(grid_fuselage, dimsplit)

    return triangrid_fuselage
end


"""
    Generates ground surface.
"""
function generate_ground_vahana(bscale; verify_spline=false, spl_s=0.001,
                                spl_k="automatic", visualize=false)

    ## Ellipse
    fx, fy = 1.7, 1.05
    lx = bscale*3.5
    ly = bscale*3
    cx = -lx*3/4
    cy = 0.0
    cz = -0.06*bscale
    a, b = lx, ly
    perimeter = [ a*b/sqrt((b*cos(tht))^2 + (a*sin(tht))^2
                    ) * [fx*cos(tht), fy*sin(tht), 0] + [cx, cy, cz] for
                                                tht in linspace(0, 2*pi, 179)]

    # Fluid domain
    NDIVSx = 50              # Cells in the parametric x-direction
    NDIVSy = 50              # Cells in the parametric y-direction
    NDIVSz = 0               # Cells in the geometric z-direction

    z_off = cz               # z-offset for ground position

    if visualize
        opt_args = [(:save_path, "temps/"), (:file_name, "vahanaground"),
                    (:paraview, true)]
    else
        opt_args = []
    end

    ground = generate_perimetergrid(perimeter, NDIVSx, NDIVSy, NDIVSz;
                                       z_min=z_off, z_max=z_off,
                                        verify_spline=verify_spline, spl_s=spl_s,
                                        spl_k=spl_k,
                                        opt_args...
                                        )

    return ground
end


# -------------- END OF VAHANA -------------------------------------------------




################################################################################
# COMMON FUNCTIONS
################################################################################

"""
`generate_perimetergrid(perimeter::Array{Array{T, 1}, 1},
                                  NDIVSx, NDIVSy, NDIVSz;
                                  z_min::Real=0, z_max::Real=0,
                                  # SPLINE OPTIONS
                                  verify_spline::Bool=true,
                                  spl_s=0.001, spl_k="automatic",
                                  # FILE OPTIONS
                                  save_path=nothing, file_name="perimeter",
                                  paraview=true
                                )`

  Generates the perimeter grid with `perimeter` the array of points of the
  contour (must be a closed contour), and `NDIVS_` the number of cells in
  each parametric dimension (give it `NDIVSz=0` for a flat surface, otherwise
  it'll generate a volumetric grid between `z_min` and `z_max`).
"""
function generate_perimetergrid(perimeter::Array{Array{T, 1}, 1},
                                  NDIVSx, NDIVSy, NDIVSz;
                                  z_min::Real=0, z_max::Real=0,
                                  # SPLINE OPTIONS
                                  verify_spline::Bool=true,
                                  spl_s=0.001, spl_k="automatic",
                                  # FILE OPTIONS
                                  save_path=nothing, file_name="perimeter",
                                  paraview=true
                                ) where{T<:Real}

  # Error cases
  multidiscrtype = Array{Tuple{Float64,Int64,Float64,Bool},1}
  if typeof(NDIVSx)==Int64
    nz = NDIVSz
  elseif typeof(NDIVSz)==multidiscrtype
    nz = 0
    for sec in NDIVSz
      nz += sec[2]
    end
  else
    error("Expected `NDIVSz` to be type $(Int64) or $MultiDiscrType,"*
            " got $(typeof(NDIVSz)).")
  end

  # --------- REPARAMETERIZES THE PERIMETER ---------------------------
  org_x = [p[1] for p in perimeter]
  org_y = [p[2] for p in perimeter]
  # Separate upper and lower sides to make the contour injective in x
  upper, lower = gt.splitcontour(org_x, org_y)

  # # Parameterize both sides independently
  # fun_upper = gt.parameterize(upper[1], upper[2], zeros(upper[1]); inj_var=1,
  #                                                     s=spl_s, kspl=spl_k)
  # fun_lower = gt.parameterize(lower[1], lower[2], zeros(lower[1]); inj_var=1,
  #                                                     s=spl_s, kspl=spl_k)
  # # Discretizes both sides
  # if NDIVSx==multidiscrtype
  #   new_upper = gt.multidiscretize(fun_upper, 0, 1, NDIVSx)
  #   new_lower = gt.multidiscretize(fun_lower, 0, 1, NDIVSx)
  # else
  #   new_upper = gt.discretize(fun_upper, 0, 1, NDIVSx, 1.0)
  #   new_lower = gt.discretize(fun_lower, 0, 1, NDIVSx, 1.0)
  # end

  splt_up = Int(ceil((size(upper[1],1)/2)))
  splt_low = Int(ceil((size(lower[1],1)/2)))

  # Splits the perimeter into four faces
  upper1 = [[x for x in upper[1][1:splt_up]], [y for y in upper[2][1:splt_up]]]
  upper2 = [[x for x in upper[1][splt_up:end]], [y for y in upper[2][splt_up:end]]]
  lower1 = [[x for x in lower[1][1:splt_low]], [y for y in lower[2][1:splt_low]]]
  lower2 = [[x for x in lower[1][splt_low:end]], [y for y in lower[2][splt_low:end]]]

  # Parameterize both sides independently
  fun_upper1 = gt.parameterize(upper1[1], upper1[2], zeros(upper1[1]); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_upper2 = gt.parameterize(upper2[1], upper2[2], zeros(upper2[1]); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_lower1 = gt.parameterize(lower1[1], lower1[2], zeros(lower1[1]); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_lower2 = gt.parameterize(lower2[1], lower2[2], zeros(lower2[1]); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  # Discretizes both sides
  if NDIVSx==multidiscrtype
    new_upper1 = gt.multidiscretize(fun_upper1, 0, 1, NDIVSx)
    new_upper2 = gt.multidiscretize(fun_upper2, 0, 1, NDIVSy)
    new_lower1 = gt.multidiscretize(fun_lower1, 0, 1, NDIVSy)
    new_lower2 = gt.multidiscretize(fun_lower2, 0, 1, NDIVSx)
  else
    new_upper1 = gt.discretize(fun_upper1, 0, 1, NDIVSx, 1.0)
    new_upper2 = gt.discretize(fun_upper2, 0, 1, NDIVSy, 1.0)
    new_lower1 = gt.discretize(fun_lower1, 0, 1, NDIVSy, 1.0)
    new_lower2 = gt.discretize(fun_lower2, 0, 1, NDIVSx, 1.0)
  end

  # ----------------- SPLINE VERIFICATION --------------------------------------

  if verify_spline
  #   new_points = vcat(reverse(new_upper), new_lower)
    # new_x = [p[1] for p in new_points]
    # new_y = [p[2] for p in new_points]
    plt.plot(org_x, org_y, "--^k", label="Original", alpha=0.5)
    # plt.plot(new_x, new_y, ":.b", label="Parameterized")

    plt.plot(upper1[1], upper1[2], "*b", label="upper1", alpha=0.75)
    plt.plot(upper2[1], upper2[2], "^b", label="upper2", alpha=0.75)
    plt.plot(lower1[1], lower1[2], "*r", label="lower1", alpha=0.75)
    plt.plot(lower2[1], lower2[2], "^r", label="lower2", alpha=0.75)

    new_points = vcat(new_upper1, new_upper2)
    new_x = [p[1] for p in new_points]
    new_y = [p[2] for p in new_points]
    plt.plot(new_x, new_y, ":.b", label="Parameterized Upper", alpha=0.75)
    new_points = vcat(new_lower1, new_lower2)
    new_x = [p[1] for p in new_points]
    new_y = [p[2] for p in new_points]
    plt.plot(new_x, new_y, ":.r", label="Parameterized Lower", alpha=0.75)
    # plt.xlabel(plt.L"x")
    # plt.ylabel(plt.L"y")
    plt.legend(loc="best")
    plt.grid(true, color="0.8", linestyle="--")
  end


  # --------- GRIDS THE INSIDE OF THE PERIMETER ---------------------
  # Parametric grid
  P_min = zeros(3)
  P_max = [1, 1, 1*(nz!=0)]
  param_grid = gt.Grid(P_min, P_max, [NDIVSx, NDIVSy, NDIVSz])

  # function my_space_transform(X, ind)
  #     i = ind[1]                      # Arc length point
  #     w = X[2]                        # Weight
  #     z = z_min + X[3]*(z_max-z_min)  # z-position
  #
  #     Y = new_lower[i] + w*(new_upper[i]-new_lower[i])
  #     Y[3] = z
  #
  #     return Y
  # end

  rev_new_upper1 = reverse(new_upper1) # Left edge
  rev_new_upper2 = reverse(new_upper2) # Upper edge
  rev_new_lower1 = reverse(new_lower1) # Lower edge
  rev_new_lower2 = reverse(new_lower2) # Right edge

  function my_space_transform(X, ind)
      beta = 1e-2*pi/2                 # Warping parameter

      z = z_min + X[3]*(z_max-z_min)   # z-position

      xw = X[2]                        # x weight
      yw = X[1]                        # y weight

      # Linearly interpolate the edges
      x = new_upper1[ind[1]]*xw + new_lower2[ind[1]]*(1-xw) # Left-right edges
      y = rev_new_upper2[ind[2]]*yw + rev_new_lower1[ind[2]]*(1-yw) # Upper-lower edges
      # Y = x + y

      # A clever way of merging the two interpolations
      # First weight
      yw = abs(X[1]-0.5)/0.5
      yw = tan(yw*pi/2 - beta)/tan(pi/2 - beta)
      # Second weight
      xw = abs(X[2]-0.5)/0.5
      xw = tan(xw*pi/2 - beta)/tan(pi/2 - beta)
      # Merged weight
      w = ( (1-xw) + yw ) /2
      # Merged point
      Y = x*(1-w) + y*w

      Y[3] = z

      return Y
  end

  # Applies the space transformation to the parametric grid
  gt.transform3!(param_grid, my_space_transform)

  if save_path!=nothing
    gt.save(param_grid, file_name; path=save_path)

    if paraview
      strn = file_name*".vtk"
      run(`paraview --data=$save_path/$strn`)
    end

  end

  return param_grid::gt.Grid
end
# -------------- END OF COMMONS ------------------------------------------------
