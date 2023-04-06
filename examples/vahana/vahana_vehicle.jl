#=##############################################################################
# DESCRIPTION
    1) Vehicle definition
=###############################################################################

"""
    Generates the geometry of Vahana aircraft
"""
function generate_vahana_vehicle(;
                                    # VEHICLE OPTIONS
                                    xfoil       = true,             # Whether to run XFOIL
                                    n_factor::Int = 1,              # Discretization factor
                                    add_wings   = true,             # Whether to add the wings
                                    add_rotors  = true,             # Whether to add the rotors
                                    VehicleType = uns.VLMVehicle,   # Type of vehicle to generate (uns.QVLMVehicle for quasi-steady solver)
                                    data_path   = uns.def_data_path,# Database path
                                    # OUTPUT OPTIONS
                                    run_name    = "vahana",
                                    verbose     = true,
                                    v_lvl       = 0
                                    )

    ############################################################################
    # PARAMETERS
    ############################################################################

    if verbose; println("\t"^(v_lvl)*"Defining parameters..."); end;

    # ------------ GEOMETRIC PARAMETERS ------------------------------------
    # Tilt and fixed rotors
    tiltrotor_file  = "VahanaTilt.csv"      # Tilt-rotor file
    R_w             = 0.75                  # (m) main-wing rotor radius (reference)
    R_tw            = R_w                   # (m) tandem-wing rotor radius (reference)
    CW_w            = true                  # Clockwise rotation of main-wing rotor
    CW_tw           = false                 # Clockwise rotation of tandem-wing rotor
    nr_w            = 2                     # Number of rotors per side of main wing
    nr_tw           = 2                     # Number of rotors per side of tandem wing
    pitch           = 0.0                   # (deg) collective pitch for tiltrotors rotors on main and tandem wings
    main_outtilt    = 10                    # (deg) Mount main-wing rotors with this out-tilt angle
    tandem_pitchtilt= 10                    # (deg) Mount tandem-wing rotors with this alternating pitch-tilt angle
    xoc_offset_main = 0.175                 # Axial distance of main-wing rotors from LE, divided by chord
    xoc_offset_tandem = 0.10                # Axial distance of tandem-wing rotors from LE, divided by chord
    soD             = 0.1                   # Tip-to-tip distance of rotors, divided by diameter
    ReD07           = 1.5e6                 # Assumed diameter-based Reynolds number for all rotors at r/R=0.7
    ReD             = ReD07/0.7             # Reynolds number at blade tip (for XFOIL)
    n_rotor         = 7*n_factor            # Number of blade elements per blade
    r_rotor         = 1/20                  # Geometric expansion between blade elements
    tilt_read_polar = vlm.ap.read_polar     # What polar reader to use

    # Stacked rotors
    stackedrotor_file = "VahanaStacked.csv" # Stacked-rotor file
    stacked         = [nr_w, nr_w+1]        # Index of main-wing rotors that will be stacked rotors
    stckd_xoc_offset= -1.40                 # Axial distance of stacked rotors from LE, divided by chord
    stckd_zoc_offset= 0.10                  # Height of stacked rotors above wing, divided by chord
    stckd_zoR_offset= -0.05                 # Stacking distance between stacked rotors, divided by R
    stckd_corotating= true                  # Co-rotating stacked rotors if true, counter-rotating otherwise
    stckd_phase     = -10                   # (deg) initial phase difference of stacked rotors (index angle)
    stckd_pitch_up  = 5.0                   # (deg) collective pitch of upper rotor in stack
    stckd_pitch_low = stckd_pitch_up + 7.5  # (deg) collective pitch of lower rotor in stack
    stacked_read_polar = vlm.ap.read_polar2 # What polar reader to use

    # Main wing
    b_w             = 5.86                  # (m) span
    AR_w            = 7.4                   # Aspect ratio (b/c_tip)
    tr_w            = 1.0                   # Taper ratio (c_tip/c_root)
    twist_r_w       = 14.0                  # (deg) twist at root
    twist_t_w       = twist_r_w             # (deg) twist at tip
    lambda_w        = main_outtilt          # (deg) sweep
    gamma_w         = 5.0                   # (deg) dihedral
    md_w            = 0.9                   # Length of middle section, divided by span
    pivot_w         = 1/4                   # Pivot point along chord of tilt-wing
    n_w             = 24*n_factor           # Number of wing elements per side
    r_w             = 2.0                   # Geometric expansion of wing elements

    # Main-wing winglets
    b_wl            = b_w/4                 # (m) span of winglet from top to bottom
    AR_wl           = 3.0                   # Aspect ratio (b/c_tip)
    tr_wl           = (b_wl/AR_wl)/(b_w/AR_w/tr_w)  # Taper ratio (c_tip/c_root)
    twist_r_wl      = 2.5                   # (deg) twist at root
    twist_t_wl      = 0.0                   # (deg) twist at tip
    lambda_wl       = 40.0                  # (deg) sweep
    gamma_wl        = 15.0                  # (deg) dihedral
    n_wl            = 8*n_factor            # Number of wing elements per side
    r_wl            = 2.0                   # Geometric expansion of wing elements

    # Tandem wing
    b_tw            = b_w*1.0               # (m) span
    AR_tw           = 9.5                   # Aspect ratio (b/c_tip)
    tr_tw           = 1.0                   # Taper ratio (c_tip/c_root)
    twist_r_tw      = 14.0                  # (deg) twist at root
    twist_t_tw      = twist_r_tw            # (deg) twist at tip
    lambda_tw       = 0.0                   # (deg) sweep
    gamma_tw        = 0.0                   # (deg) dihedral
    md_tw           = 0.2                   # Length of middle section, divided by span
    pivot_tw        = pivot_w               # Pivot point along chord of tilt-wing
    n_tw            = 2*n_w                 # Number of wing elements per side
    r_tw            = r_w                   # Geometric expansion of wing elements

    # Fuselage
    l_f             = 5.86                  # (m) length
    h_f             = 2.81*2/3              # (m) height

    # ------------ ASSEMBLY PARAMETERS ------------------------------------
    # Position of wings on fuselage
    h_pos_w         = 0.90*h_f              # (m) height position of wing
    h_pos_tw        = 0.15*h_f              # (m) height position of tandem wing
    l_pos_w         = 0.95*l_f-b_w/AR_w/tr_w# (m) length position of wing
    l_pos_tw        = 0.05*l_f              # (m) length position of tandem wing

    # Position of rotors along main wing
    d_rotor_w       = (1+soD)*(2*R_w)       # Distance between rotors
    y_pos_rotor_w   = Float64[b_w/2 - i*d_rotor_w for i in 0:nr_w-1]    # y-positions

    # Position of rotors along tandem wing
    d_rotor_tw      = (1+soD)*(2*R_tw)      # Distance between rotors on wing
    y_pos_rotor_tw  = Float64[b_tw/2 - i*d_rotor_tw for i in 0:nr_tw-1] # y-positions

    init_ori        = 90.0                  # (deg) initial orientation of wings
    init_ori_rotor  = 0.0                   # (deg) initial orientation of rotors
    init_ori_stackedrotor = 30.0            # (deg) initial orientation of stacked rotors


    ############################################################################
    # GENERATE COMPONENTS
    ############################################################################

    if verbose; println("\t"^(v_lvl)*"Generating components..."); end;

    # ------------ ROTORS ------------------------------------------------
    # Generate base rotors (one for each rotation orientation)
    if add_rotors

        tiltrotors = vlm.Rotor[]                # Tilt rotors

        if verbose; println("\t"^(v_lvl+1)*"Generating first tilt-rotor..."); end;
        push!(tiltrotors, uns.generate_rotor(tiltrotor_file; pitch=pitch,
                                                n=n_rotor, blade_r=r_rotor, CW=!CW_w, ReD=ReD,
                                                verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                read_polar=tilt_read_polar,
                                                data_path=data_path, plot_disc=false))

        if verbose; println("\t"^(v_lvl+1)*"Generating second tilt-rotor..."); end;
        push!(tiltrotors, uns.generate_rotor(tiltrotor_file; pitch=pitch,
                                                n=n_rotor, blade_r=r_rotor, CW=CW_w, ReD=ReD,
                                                verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                read_polar=tilt_read_polar,
                                                data_path=data_path, plot_disc=false))

        stackedrotors = vlm.Rotor[]             # Upper rotor in stacked rotors

        if verbose; println("\t"^(v_lvl+1)*"Generating first stacked-rotor..."); end;
        push!(stackedrotors, uns.generate_rotor(stackedrotor_file; pitch=stckd_pitch_up,
                                                n=n_rotor, blade_r=r_rotor, CW=!CW_w, ReD=ReD,
                                                verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                read_polar=stacked_read_polar,
                                                data_path=data_path, plot_disc=false))

        if verbose; println("\t"^(v_lvl+1)*"Generating second stacked-rotor..."); end;
        push!(stackedrotors, uns.generate_rotor(stackedrotor_file; pitch=stckd_pitch_up,
                                                n=n_rotor, blade_r=r_rotor, CW=CW_w, ReD=ReD,
                                                verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                read_polar=stacked_read_polar,
                                                data_path=data_path, plot_disc=false))

        stackedrotors_low = vlm.Rotor[]         # Lower rotor in stacked rotors

        if stckd_pitch_up != stckd_pitch_low

            if verbose; println("\t"^(v_lvl+1)*"Generating first lower-stacked-rotor..."); end;
            push!(stackedrotors_low, uns.generate_rotor(stackedrotor_file; pitch=stckd_pitch_low,
                                                    n=n_rotor, blade_r=r_rotor, CW=!CW_w, ReD=ReD,
                                                    verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                    read_polar=stacked_read_polar,
                                                    data_path=data_path, plot_disc=false))

            if verbose; println("\t"^(v_lvl+1)*"Generating second lower-stacked-rotor..."); end;
            push!(stackedrotors_low, uns.generate_rotor(stackedrotor_file; pitch=stckd_pitch_low,
                                                    n=n_rotor, blade_r=r_rotor, CW=CW_w, ReD=ReD,
                                                    verbose=verbose, v_lvl=v_lvl+2, xfoil=xfoil,
                                                    read_polar=stacked_read_polar,
                                                    data_path=data_path, plot_disc=false))

        else

            for rotor in stackedrotors
                push!(stackedrotors_low, rotor)
            end

        end
    end


    # ------------ MAIN WING ---------------------------------------------
    # Generate wing
    if verbose; println("\t"^(v_lvl+1)*"Generating main wing assembly..."); end;

    # Middle section
    pos_md_w = [0.0, md_w]
    clen_md_w = [1/tr_w, md_w + (1/tr_w)*(1-md_w)]
    twist_md_w = [twist_r_w, twist_r_w + md_w*(twist_t_w-twist_r_w)]
    wing_md = vlm.complexWing(b_w, AR_w, ceil(Int, md_w*n_w), pos_md_w, clen_md_w, twist_md_w,
                          lambda_w*ones(1), gamma_w*ones(1);
                          symmetric=true, chordalign=0.0,
                          _ign1=true)

    # Left section
    pos_l_w = [0, -(1-md_w)]   # NOTE: Here we define this wing section from right to left
    clen_l_w = [1, clen_md_w[end]]
    twist_l_w = [twist_t_w, twist_md_w[end]]
    wing_L = vlm.complexWing(b_w, AR_w, ceil(Int, (1-md_w)*n_w/2), pos_l_w, clen_l_w,
                            twist_l_w, -lambda_w*ones(1), -gamma_w*ones(1);
                            symmetric=false, chordalign=0.0,
                            _ign1=true)

    # Right section
    pos_r_w = [0, (1-md_w)]
    clen_r_w = [clen_md_w[end], 1]
    twist_r_w = [twist_md_w[end], twist_t_w]
    wing_R = vlm.complexWing(b_w, AR_w, ceil(Int, (1-md_w)*n_w/2), pos_r_w, clen_r_w,
                            twist_r_w, lambda_w*ones(1), gamma_w*ones(1);
                            symmetric=false, chordalign=0.0,
                            _ign1=true)

    # Translate right and left sections to position
    O_w_R = (md_w*b_w/2)*[0, 1, 0]
    O_w_L = [1 0 0; 0 -1 0; 0 0 1]*O_w_R
    vlm.setcoordsystem(wing_R, O_w_R, Im)
    vlm.setcoordsystem(wing_L, O_w_L, Im)

    # Winglets
    winglet_R = vlm.simpleWing(b_wl, AR_wl, tr_wl, twist_r_wl, lambda_wl, gamma_wl;
                                          twist_tip=twist_t_wl, n=n_wl, r=r_wl)

    winglet_L = vlm.simpleWing(b_wl, AR_wl, tr_wl, twist_r_wl, lambda_wl, gamma_wl;
                                          twist_tip=twist_t_wl, n=n_wl, r=r_wl)

    # Translate winglets to position
    O_wl_R = (b_w/2)*[0, 1, 0]
    O_wl_R += ((1-md_w)*b_w/2)*[tan(lambda_w*pi/180), 0, tan(gamma_w*pi/180)]
    O_wl_L = [1 0 0; 0 -1 0; 0 0 1]*O_wl_R
    Oaxis_wl_R = gt.rotation_matrix(0.0, 0.0, 90.0)
    Oaxis_wl_L = gt.rotation_matrix(0.0, 0.0, -90.0)
    vlm.setcoordsystem(winglet_R, O_wl_R, Oaxis_wl_R)
    vlm.setcoordsystem(winglet_L, O_wl_L, Oaxis_wl_L)

    # Generate main-wing rotors (from right to left)
    if add_rotors

        if verbose; println("\t"^(v_lvl+2)*"Generating main-wing rotors..."); end;

        O_rotor_w = [                            # Position of each rotor
                     (ypos - md_w*b_w/2)*[tan(lambda_w*pi/180), 0, -tan(gamma_w*pi/180)] +
                     ypos*[0, 1, 0] +
                     [-(i in stacked ? stckd_xoc_offset : xoc_offset_main)*AR_w/b_w, 0, 0] +
                     [0, 0, (i in stacked ? stckd_zoc_offset*tan(gamma_w*pi/180) : 0)*AR_w/b_w]
                     for (i, ypos) in enumerate(y_pos_rotor_w)]

        rotors_w = vlm.Rotor[]                  # Rotors get stored in these arrays
        rotors_w_stacked_up = vlm.Rotor[]
        rotors_w_stacked_low = vlm.Rotor[]

        for i in 1:2*nr_w
            right = i<=nr_w                     # Indicates which side of the wing

            copy_rotor = i in stacked ? stackedrotors[1+i%2] : tiltrotors[1+i%2]
            this_rotor = deepcopy(copy_rotor)   # Alternates rotation orientation

            this_O = O_rotor_w[ right ? i : nr_w-(i-nr_w-1)]  # Chooses position
            this_O = [1 0 0; 0 (-1)^!right 0; 0 0 1]*this_O   # Places it in correct side
            this_Oaxis = i in stacked ? gt.rotation_matrix(0.0, 90, 0.0) :
                                        gt.rotation_matrix((-1)^(i%2==0)*main_outtilt, 0.0, 0.0)

            # Place rotor in position
            vlm.setcoordsystem(this_rotor, this_O, this_Oaxis; user=true)

            # Rotate rotor to be tip-to-tip with others
            vlm.rotate(this_rotor, (-1)^(!CW_w) * (i in stacked ? init_ori_stackedrotor : init_ori_rotor))

            # Add the original polars that are not copied with deepcopy
            this_rotor.airfoils = copy_rotor.airfoils
            this_rotor._polars = copy_rotor._polars
            this_rotor._polarroot = copy_rotor._polarroot
            this_rotor._polartip = copy_rotor._polartip

            if !(i in stacked)

                push!(rotors_w, this_rotor)

            else
                push!(rotors_w_stacked_up, this_rotor)

                # Generate lower rotor if this is a stacked rotor
                copy_rotor = stackedrotors_low[ 1+(i+1*!stckd_corotating)%2 ]
                this_rotor = deepcopy(copy_rotor)

                # Place rotor in position
                this_O += R_w*[0, 0, stckd_zoR_offset]
                vlm.setcoordsystem(this_rotor, this_O, this_Oaxis; user=true)

                # Rotate rotor to be tip-to-tip with others
                vlm.rotate(this_rotor, (-1)^(!CW_w) * (init_ori_stackedrotor+stckd_phase))

                # Add the original polars that are not copied with deepcopy
                this_rotor.airfoils = copy_rotor.airfoils
                this_rotor._polars = copy_rotor._polars
                this_rotor._polarroot = copy_rotor._polarroot
                this_rotor._polartip = copy_rotor._polartip

                push!(rotors_w_stacked_low, this_rotor)
            end
        end
    end

    # Assemble fixed section of the wing (middle section + stacked rotors)
    main_wing_fixed = vlm.WingSystem()
    vlm.addwing(main_wing_fixed, "WingM", wing_md)

    if add_rotors
        for (i, rotor) in enumerate(rotors_w_stacked_up)
            vlm.addwing(main_wing_fixed, "StackedRotorUp$i", rotor)
        end
        for (i, rotor) in enumerate(rotors_w_stacked_low)
            vlm.addwing(main_wing_fixed, "StackedRotorLow$i", rotor)
        end
    end

    # Assemble wing tip sections
    main_wing_R = vlm.WingSystem()
    vlm.addwing(main_wing_R, "Tip", wing_R)
    vlm.addwing(main_wing_R, "Winglet", winglet_R)

    main_wing_L = vlm.WingSystem()
    vlm.addwing(main_wing_L, "Tip", wing_L)
    vlm.addwing(main_wing_L, "Winglet", winglet_L)

    # Assemble moving sections of the wing
    main_wing_moving = vlm.WingSystem()
    vlm.addwing(main_wing_moving, "WingR", main_wing_R)
    vlm.addwing(main_wing_moving, "WingL", main_wing_L)

    if add_rotors
        for (i, rotor) in enumerate(rotors_w)
            vlm.addwing(main_wing_moving, "Rotor$i", rotor)
        end
    end


    # Align moving and fixed section with their pivot line
    x_off_w = pivot_w*b_w/AR_w                  # offset to align with pivot line
    O_off_w = [-x_off_w, 0.0, 0.0]

    vlm.setcoordsystem(wing_md, O_off_w, Im)

    for vlmwing in main_wing_moving.wings
        if typeof(vlmwing)==vlm.Rotor
            vlm.setcoordsystem(vlmwing, vlmwing._wingsystem.O + O_off_w,
                                        vlmwing._wingsystem.Oaxis; user=false)
        else
            vlm.setcoordsystem(vlmwing, vlmwing.O + O_off_w, vlmwing.Oaxis)
        end
    end

    # Place tilting sections at main-wing tip
    O_mv = (md_w*b_w/2)*[tand(lambda_w), 0, tand(gamma_w)]

    # Initial rotation of moving sections
    Oaxis_wmv = gt.rotation_matrix(0.0, -init_ori, 0.0)
    vlm.setcoordsystem(main_wing_moving, O_mv, Oaxis_wmv)

    # Assemble main wing
    main_wing = vlm.WingSystem()
    vlm.addwing(main_wing, "Fixed", main_wing_fixed)
    vlm.addwing(main_wing, "Moving", main_wing_moving)

    # Position of main wing
    O_w = [l_pos_w + x_off_w, 0, h_pos_w]
    Oaxis_w = gt.rotation_matrix(0.0, 0.0, 0.0)
    vlm.setcoordsystem(main_wing, O_w, Oaxis_w)

    # ------------ TANDEM WING -------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating tandem wing assembly..."); end;

    # Generate tandem wing
    twing = vlm.simpleWing(b_tw, AR_tw, tr_tw, twist_r_tw, lambda_tw,
                                gamma_tw; twist_tip=twist_t_tw, n=n_tw, r=r_tw)

    # Middle section
    pos_md_tw = [-md_tw, 0.0, md_tw]
    clen_md_tw = [md_tw + (1/tr_tw)*(1-md_tw), 1/tr_tw, md_tw + (1/tr_tw)*(1-md_tw)]
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

    ## Generate tandem-wing rotors (from right to left)
    if add_rotors

        if verbose; println("\t"^(v_lvl+2)*"Generating tandem-wing rotors..."); end;

        O_rotor_tw = [ ypos*[tan(lambda_tw*pi/180), 1, tan(gamma_tw*pi/180)] +
                        [-xoc_offset_tandem*AR_tw/b_tw, 0, 0]
                                                  for ypos in y_pos_rotor_tw]

        rotors_tw = vlm.Rotor[]

        for i in 1:2*nr_tw
            right = i<=nr_tw                        # Indicates which side of the wing

            copy_rotor = tiltrotors[1+(i+(CW_tw!=CW_w))%2]
            this_rotor = deepcopy(copy_rotor)       # Alternates rotation orientation

            this_O = O_rotor_tw[ right ? i : nr_tw-(i-nr_tw-1)] # Chooses position
            this_O = [1 0 0; 0 (-1)^!right 0; 0 0 1]*this_O     # Places it in correct side
            this_Oaxis = gt.rotation_matrix(0, (-1)^(i%2==0)*(-1)^right*tandem_pitchtilt, 0)

            # Place rotor in position
            vlm.setcoordsystem(this_rotor, this_O, this_Oaxis; user=true)

            # Rotates rotor to be tip-to-tip with others
            vlm.rotate(this_rotor, (-1)^(!CW_tw) * init_ori_rotor)

            # Add the original polars that are not copied with deepcopy
            this_rotor.airfoils = copy_rotor.airfoils
            this_rotor._polars = copy_rotor._polars
            this_rotor._polarroot = copy_rotor._polarroot
            this_rotor._polartip = copy_rotor._polartip

            push!(rotors_tw, this_rotor)
        end
    end

    # Assemble moving sections of the wing
    tandem_wing_moving = vlm.WingSystem()
    vlm.addwing(tandem_wing_moving, "WingR", twing_R)
    vlm.addwing(tandem_wing_moving, "WingL", twing_L)

    if add_rotors
        for (i, rotor) in enumerate(rotors_tw)
            vlm.addwing(tandem_wing_moving, "Rotor$i", rotor)
        end
    end

    # Align moving and fixed section with their pivot line
    x_off_tw = pivot_tw*b_tw/AR_tw                  # offset to align with pivot line
    O_off_tw = [-x_off_tw, 0.0, 0.0]

    vlm.setcoordsystem(twing_md, O_off_tw, Im)

    for vlmwing in tandem_wing_moving.wings
        if typeof(vlmwing)==vlm.Rotor
            vlm.setcoordsystem(vlmwing, vlmwing._wingsystem.O + O_off_tw,
                                        vlmwing._wingsystem.Oaxis; user=false)
        else
            vlm.setcoordsystem(vlmwing, vlmwing.O + O_off_tw, vlmwing.Oaxis)
        end
    end

    # Initial rotation of moving sections
    Oaxis_twmv = gt.rotation_matrix(0.0, -init_ori, 0.0)
    vlm.setcoordsystem(tandem_wing_moving, zeros(3), Oaxis_twmv)

    # Assemble tandem wing
    tandem_wing = vlm.WingSystem()
    vlm.addwing(tandem_wing, "FixedWing", twing_md)
    vlm.addwing(tandem_wing, "Moving", tandem_wing_moving)

    # Position of tandem wing
    O_tw = [l_pos_tw + x_off_tw, 0, h_pos_tw]
    Oaxis_tw = gt.rotation_matrix(0.0, 0.0, 0.0)
    vlm.setcoordsystem(tandem_wing, O_tw, Oaxis_tw)

    # ------------ FUSELAGE ----------------------------------------------
    if verbose; println("\t"^(v_lvl+1)*"Generating fuselage..."); end;

    # Generate fuselage
    fuselage = generatefuselage_vahana(l_f; ncells=20)

    # All grids get stored here
    body = gt.MultiGrid(3)
    gt.addgrid(body, "Fuselage", fuselage)

    # Generate pylons of stacked rotors
    if add_rotors
        if verbose; println("\t"^(v_lvl+1)*"Generating pylons..."); end;

        pylon_pos = 0.70
        pylon_length = stckd_xoc_offset*AR_w/b_w * pylon_pos

        for i in stacked
            pylon = generatepylon(pylon_length; ncells=5)

            right = i<=nr_w
            this_O = O_rotor_w[ right ? i : nr_w-(i-nr_w-1)]  # Chooses position
            this_O = [1 0 0; 0 (-1)^!right 0; 0 0 1]*this_O   # Places it in correct side
            this_O += main_wing.O                             # Translates it with the wing
                                                              # Offsets it according to length
            this_O += [1/3*0.5*pylon_pos, 0, 3*R_w*stckd_zoR_offset]
            rotation = gt.rotation_matrix2(0, 0, 0)

            gt.lintransform!(pylon, rotation, this_O)

            gt.addgrid(body, "Pylon$i", pylon)
        end
    end


    ############################################################################
    # DEFINE VEHICLE
    ############################################################################

    # System of all FLOWVLM objects
    system = vlm.WingSystem()
    vlm.addwing(system, "MainWing", main_wing)
    vlm.addwing(system, "TandemWing", tandem_wing)

    # Tilting systems
    tilting_systems = (main_wing_moving, tandem_wing_moving)

    # Rotors grouped by systems with shared RPM control
    if add_rotors
        rotor_systems = (rotors_w, rotors_w_stacked_up, rotors_w_stacked_low, rotors_tw)
    else
        rotor_systems = ()
    end

    # System solved through VLM solver
    vlm_system_m = vlm.WingSystem()
    vlm.addwing(vlm_system_m, "middle", wing_md)
    vlm.addwing(vlm_system_m, "R", wing_R)
    vlm.addwing(vlm_system_m, "letR", winglet_R)
    vlm.addwing(vlm_system_m, "L", wing_L)
    vlm.addwing(vlm_system_m, "letL", winglet_L)

    vlm_system_t = vlm.WingSystem()
    vlm.addwing(vlm_system_t, "R", twing_R)
    vlm.addwing(vlm_system_t, "L", twing_L)

    vlm_system = vlm.WingSystem()
    if add_wings
        vlm.addwing(vlm_system, "MWing", vlm_system_m)
        vlm.addwing(vlm_system, "TWing", vlm_system_t)
    end

    # All rotors
    if add_rotors
        rotors = vcat(rotors_w, rotors_w_stacked_up, rotors_w_stacked_low, rotors_tw)
    end

    # System that will shed a VPM wake
    wake_system = vlm.WingSystem()

    if add_wings
        vlm.addwing(wake_system, "SolveVLM", vlm_system)
    end

    if add_rotors
        if VehicleType==uns.VLMVehicle
            for (i, rotor) in enumerate(rotors)
                vlm.addwing(wake_system, "Rotor$i", rotor)
            end
        else
            # Mute warnings regarding potential colinear vortex filaments. This is
            # needed since the quasi-steady solver will probe induced velocities at the
            # lifting line of the blade
            uns.vlm.VLMSolver._mute_warning(true)
        end
    end

    # Visualization grids that are rotated and translated along with the vehicle
    grids = [body]

    # Define vehicle
    vehicle = VehicleType(   system;
                                tilting_systems=tilting_systems,
                                rotor_systems=rotor_systems,
                                vlm_system=vlm_system,
                                wake_system=wake_system,
                                grids=grids
                             )

    return vehicle
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]
    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]
    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    section0 = points


    # --------------- LOFT -----------------------------------------------------
    # fuselage_length = 5.86            # (m) fuselage length
    bscale = fuselage_length            # Scales the entire geometry by this factor

    crosssections = [ # (l/bscale position , contour)
                        (0.0  , [[2.00*0.8, 0.25][k]*(point[k]) + [0, 0.3][k] for point in section0, k in 1:2]),
                        (0.02, [[0.6*0.8, 0.3][k]*(point[k]) + [0, 0.15][k] for point in sections[1], k in 1:2]),
                        (1/6*0.5, [[1.0*0.8, 0.75][k]*point[k] for point in sections[1], k in 1:2]),
                        # (1/6*0.5, [[0.75, 0.75][k]*point[k] for point in sections[1], k in 1:2]),
                        (2/6, [[1.00*0.8, 0.75][k]*point[k] for point in sections[2], k in 1:2]),
                        (3/6, [[1.00*0.8, 0.75][k]*(point[k] - [0, 0.1*2][k]) for point in sections[3], k in 1:2]),
                        (4/6, [[0.75, 0.75][k]*(point[k] - [0, 0.4*2][k]) for point in sections[4], k in 1:2]),
                        (5/6, [[0.75, 0.75][k]*(point[k] - [0, 0.6][k]) for point in sections[5], k in 1:2]),
                        (6/6, [[0.75, 0.75][k]*(point[k] - [0, 1.0][k]) for point in sections[6], k in 1:2]),
                    ]

    # Dummy parameters
    b_pos = [sec[1] for sec in crosssections]
    chords = (1/bscale)*ones(size(b_pos))
    twists = zeros(size(b_pos))
    LE_x = zeros(size(b_pos))
    LE_z = zeros(size(b_pos))

    tilt_y = zeros(size(b_pos))
    # tilt_y[2] = 45

    grid_fuselage = gt.generate_loft(crosssections, bscale,
                                        b_pos, chords, twists, LE_x, LE_z;
                                        # MORE GEOMETRIC OPTIONS
                                        tilt_y=tilt_y,
                                        symmetric=false,
                                        loop_dim=0,
                                    )

    # Aligns the grid with the x-axis
    rotation = gt.rotation_matrix2(0, 0, 90)
    gt.lintransform!(grid_fuselage, rotation, zeros(3))

    dimsplit = 1
    triangrid_fuselage = gt.GridTriangleSurface(grid_fuselage, dimsplit)

    return triangrid_fuselage
end


"""
    Returns a loft of the pylon.
"""
function generatepylon(pylon_length; ncells=20)
    sections = [[] for i in 1:3]

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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    sections[2] = points;


    # --------------- SECTION #3------------------------------------------------
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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]

    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    points = [p .- [0, 1.7] for p in points]
    # points = [p .* [1, 0.75] for p in points]

    sections[3] = points;

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
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells)),
            collect(range(0, 1, length=ncells))
         ]
    points = gt.conic_cross_section(Ps, CPs, rhos, ss)
    points = scaling*points

    section0 = points


    # --------------- LOFT -----------------------------------------------------

    # --------------- LOFT -----------------------------------------------------
    # fuselage_length = 5.86              # (m) fuselage length
    bscale = pylon_length            # Scales the entire geometry by this factor

    crosssections = [ # (l/bscale position , contour)
                    (0.0  , [[2.00*0.8, 0.25][k]*(point[k]) + [0, 0.3][k] for point in section0, k in 1:2]*pylon_length/5.86),
                    (0.02, [[0.6*0.8, 0.3][k]*(point[k]) + [0, 0.15][k] for point in sections[1], k in 1:2]*pylon_length/5.86),
                    (1/3*0.25, [[1.0*0.8, 0.75*0.75][k]*point[k] for point in sections[1], k in 1:2]*pylon_length/5.86),
                    (2/3, [[1.00*0.8, 0.35*0.75][k]*point[k] for point in sections[2], k in 1:2]*pylon_length/5.86),
                    (3/3, [[1.00*0.8, 0.75*0.75][k]*(point[k] - [0, 0.1*2][k]) for point in sections[3], k in 1:2]*pylon_length/5.86),
                    ]

    # Dummy parameters
    b_pos = [sec[1] for sec in crosssections]
    chords = (1/bscale)*ones(size(b_pos))
    twists = zeros(size(b_pos))
    LE_x = zeros(size(b_pos))
    LE_z = zeros(size(b_pos))

    tilt_y = zeros(size(b_pos))
    # tilt_y[2] = 45

    grid_pylon = gt.generate_loft(crosssections, bscale,
                                        b_pos, chords, twists, LE_x, LE_z;
                                        # MORE GEOMETRIC OPTIONS
                                        tilt_y=tilt_y,
                                        symmetric=false,
                                        loop_dim=0,
                                    )

    # Aligns the grid with the x-axis
    rotation = gt.rotation_matrix2(0, 0, 90)
    gt.lintransform!(grid_pylon, rotation, zeros(3))

    dimsplit = 1
    triangrid_pylon = gt.GridTriangleSurface(grid_pylon, dimsplit)

    return triangrid_pylon
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
                                            tht in range(0, 2*pi, length=179)]

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
# UTILITIES
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
  fun_upper1 = gt.parameterize(upper1[1], upper1[2], zeros(size(upper1[1])); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_upper2 = gt.parameterize(upper2[1], upper2[2], zeros(size(upper2[1])); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_lower1 = gt.parameterize(lower1[1], lower1[2], zeros(size(lower1[1])); inj_var=1,
                                                      s=spl_s, kspl=spl_k)
  fun_lower2 = gt.parameterize(lower2[1], lower2[2], zeros(size(lower2[1])); inj_var=1,
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
# ------------------------------------------------------------------------------
