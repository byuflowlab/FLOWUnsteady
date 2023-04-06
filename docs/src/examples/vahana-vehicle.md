# [Vehicle Definition](@id vahanavehicle)

In this example we simulate the eVTOL transition maneuver of a tandem
tilt-wing multirotor aircraft.
The aircraft configuration resembles the Vahana eVTOL aircraft
but with tilt and stacked rotors:

```@raw html
<table>
    <tr>
        <td>
            <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//vahana2-vehicle00-cropped-small.png" alt="Pic here" style="width:90%;"/>
            <br>
            <center><b>Takeoff and landing</b></center>
        </td>
        <td>
            <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//vahana2-vehicle02-cropped-small.png" alt="Pic here" style="width:90%;"/>
            <br>
            <center><b>Cruise</b></center>
        </td>
    </tr>
</table>
```

Due to the complexity of this simulation, each step of the simulation setup
is quite involved.
Hence, we have broken down each step into a function that can be call
when we setup the simulation.

Below is the function that defines the vehicle for the simulation
([`uns.UVLMVehicle`](@ref)).
Along with defining the vehicle geometry, it also defines two tilting
systems (a tilting system is a set of components that tilt together) and
three rotor systems (a rotor system is a set of rotors with a common RPM).
Later in the next section we will define the control inputs for these
tilting and rotor systems.

```@raw html
<table>
    <tr>
        <td>
            <center>
                <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//vahana-tiltingsystems00.png" alt="Pic here" style="width:70%;"/>
            </center>
        </td>
        <td>
            <center>
                <img src="https://edoalvar2.groups.et.byu.net/public/FLOWUnsteady//vahana-rotorsystems00.png" alt="Pic here" style="width:70%;"/>
            </center>
        </td>
    </tr>
</table>
```


```julia
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
```
!!! info "Full example"
    The function that defines the fuselage is given in the full example
    under
    [examples/vahana/vahana_vehicle.jl](https://github.com/byuflowlab/FLOWUnsteady/blob/master/examples/vahana/vahana_vehicle.jl)

