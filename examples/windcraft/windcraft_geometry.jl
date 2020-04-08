#=##############################################################################
# DESCRIPTION
    Windcraft vehicle geometry generation.

# AUTHORSHIP
  * Author    : Judd Mehr and Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

"""
    Generates the geometry of M600 windcraft
"""
function generate_geometry_windcraft(;  # INCLUSION OPTIONS:
                                        circlepath      = true,
                                        includewing     = true,
                                        wingalpha       = 4.0,
                                        includetail     = true,
                                        includepylons   = true,
                                        includerotors   = true,
                                        numrotors       = 8,
                                        counterrotate   = true,
                                        includecontrols = true,
                                        inlinerotors    = false,
                                        # REFINEMENT OPTIONS
                                        numbladeelements= 15,
                                        latticepermeter = 1,
                                        # AIRCRAFT OPTIONS
                                        rotor_file      = "propturbo-M600.csv",
                                        data_path       = uns.def_data_path,
                                        xfoil           = true,
                                        n_factor::Int   = 1,
                                        # OUTPUT OPTIONS
                                        run_name        = "windcraft",
                                        verbose         = true,
                                        v_lvl           = 0)

    ############################################################################
    # PARAMETERS: This block just defines parameters needed to create geometry.
    ############################################################################

    #Print What's Happening as we go
    gt.verbalize("Defining parameters: ", v_lvl, verbose)

    # ------------ GEOMETRIC PARAMETERS (meters) -------------------------------
    # ----- Main Wing (wing system)----- #
    #Set Global Origin and Axis
    vehicleorigin = [0.0; 0.0; 0.0]
    vehicleaxis = [1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]

    #TODO Need to figure out how to best define lattices.
    # latticepermeter = 1.25

    rootspan         = 12.5 #INPUT span of flap (root section span)
    tipspan          = 13.5 #INPUT span of aileron (tip section span)
    if includewing == true
        if includecontrols == true
            # --- Control Surface Parameters (simple wing) --- #
            # Flaps
            flapchord           = 0.4*1.19 #chord length of flaps
            flapAR              = rootspan/flapchord #INPUT flap aspect ratio
            flapchordlength     = [flapchord; flapchord]
            flapchordpos        = [0.0; 1.0]
            leftflapchordpos    = [-1.0; 0.0]
            flaptwist           = zeros(2) #INPUT flap root section twist
            flapsweep           = [0.0] #INPUT flap span sweep
            flapdihedral        = [0.0] #INPUT flap span dihedral
            flapnumlattice      = round(Int,latticepermeter*rootspan) #TODO figure out how many needed.
            # Ailerons
            aileronchord         = flapchord #chord length of ailerons

            aileronAR            = tipspan/aileronchord #INPUT aileron aspect ratio
            aileronchordlength   = [aileronchord; aileronchord]
            rightaileronchordpos = [0.0; 1.0]
            leftaileronchordpos  = [-1.0; 0.0]
            ailerontwist         = zeros(2) #INPUT aileron root section twist
            ailerondihedral      = [0.0] #INPUT aileron span dihedral
            aileronnumlattice    = round(Int,latticepermeter*tipspan) #TODO figure out how many needed.


            # --- Main Wing Parameters (complex wing) --- #
            mainwingspan        = 26.0 #INPUT main wing total span
            #note that chords of main wing are without contributions from ailerons/flaps
            mainwingrootchord   = 1.666 - flapchord #chord length of root, minus flap
            mainwingtipchord    = 0.8925 - flapchord #chord length of tip, minus flap
            mainwingAR          = mainwingspan/mainwingtipchord #INPUT 'Aspect Ratio'
            mainwingchordlength = [mainwingrootchord; mainwingrootchord; mainwingtipchord]./mainwingtipchord #INPUT section chord lengths along semi-span (from center)
            mainwingchordpos    = [0.0; rootspan; mainwingspan]./mainwingspan #INPUT spanwise locations of chord sections (from center)
            mainwingtwist       = zeros(3) #INPUT chord section twist
            mainwingsweep       = [0.0; 1.5] #INPUT span section sweep
            mainwingdihedral    = [0.0; 2.5] #INPUT span section dihedral
            mainwingnumlattice  = round(Int,latticepermeter*mainwingspan) #TODO figure out how many needed.

            #Moving Control Surfaces to correct positions
            y1                  = tipspan/2*tand(mainwingsweep[2])
            d1                  = mainwingrootchord - y1
            y2                  = d1 - mainwingtipchord
            aileronsweep        = [0.0]
            aileronroll         = -2.58
            aileronyaw          = -(180/pi * atan( y2,tipspan/2 ) ) #INPUT aileron span sweep
            flappos             = [mainwingrootchord; 0.0; 0.0]
            rightaileronpos     = [mainwingrootchord; rootspan/2; 0.0]
            leftaileronpos      = [mainwingrootchord; -rootspan/2; 0.0]
            rightaileronaxis    = rotate("x",aileronroll)*rotate("z",aileronyaw)*vehicleaxis
            leftaileronaxis     = rotate("x",-aileronroll)*rotate("z",-aileronyaw)*vehicleaxis

        else
            # --- Main Wing Parameters [INCLUDING CONTROL SURFACE CHORD LENGTHS (complex wing) --- #
            mainwingspan        = 26.0 #INPUT main wing total span
            #note that chords of main wing are without contributions from ailerons/flaps
            mainwingrootchord   = 1.666 #- flapchord #chord length of root, minus flap
            mainwingtipchord    = 0.8925 #- flapchord #chord length of tip, minus flap
            mainwingAR          = mainwingspan/mainwingtipchord #INPUT 'Aspect Ratio'
            mainwingchordlength = [mainwingrootchord; mainwingrootchord; mainwingtipchord]./mainwingtipchord #INPUT section chord lengths along semi-span (from center)
            mainwingchordpos    = [0.0; rootspan; mainwingspan]./mainwingspan #INPUT spanwise locations of chord sections (from center)
            mainwingtwist       = zeros(3) #INPUT chord section twist
            mainwingsweep       = [0.0; 1.5] #INPUT span section sweep
            mainwingdihedral    = [0.0; 2.5] #INPUT span section dihedral
            mainwingnumlattice  = round(Int,latticepermeter*mainwingspan) #TODO figure out how many needed.
        end

    else
        mainwing            = []
        mainwingspan        = 26.0
        mainwingtipchord    = 0.8925
        mainwingAR          = mainwingspan/mainwingtipchord
    end

    # If we want to add the Empennage, here are the parameters for it
    if includetail == true
        # --- Horizontal Stabilizer (simple wing) --- #
        hstabspan           = 4.9
        hstabrootchord      = 1.0
        hstabtipchord       = 0.4
        hstabAR             = hstabspan/hstabtipchord
        hstabtaperratio     = 0.4
        hstabchordlength    = [hstabrootchord; hstabtipchord]./hstabtipchord
        hstabtwist          = 0.0
        hstabsweep          = 14.0
        hstabdihedral       = 0.0
        hstabpos            = [6.6; 0.0; -0.875]
        hstabrotation       = 3.0*pi/180
        hstabaxis           = [cos(hstabrotation) -sin(hstabrotation) 0; sin(hstabrotation) cos(hstabrotation) 0; 0 0 1]

        # --- Verticatal Stabilizer (complex wing) --- #
        vstabspan           = 4.0
        vstabrootchord      = vstabspan/(3.3*2) #1.3/2
        vstabtipchord       = vstabrootchord*0.8
        vstabmidchord       = vstabrootchord*1.125
        vstabAR             = vstabspan/vstabtipchord
        vstablchordlength   = [vstabrootchord; vstabmidchord; vstabtipchord]./vstabtipchord
        vstabchordpos       = [0.0; 0.25; 1.0]
        vstabtwist          = [0.0; 0.0; 0.0]
        vstabsweep          = [0.0; 0.0]
        vstabdihedral       = [0.0; 0.0]
        vstabnumlattice     = round(Int,latticepermeter*2*vstabspan)
        vstabsymmetry       = false
        vstabxpos           = 7.0
        vstabzpos           = -0.875
        vstabpos            = [vstabxpos; 0.0; vstabzpos]
        vstabaxis           = [1.0 0.0 0.0; 0.0 0.0 1.0; 0.0 1.0 0.0]

        # --- Rudder (complex wing) --- #
        rudderspan          = vstabspan
        rudderrootchord     = vstabrootchord*.3
        ruddertipchord      = rudderrootchord*0.8
        ruddermidchord      = rudderrootchord*1.25
        rudderAR            = rudderspan/ruddertipchord
        rudderlchordlength  = [rudderrootchord; ruddermidchord; ruddertipchord]./ruddertipchord
        rudderchordpos      = [0.0; 0.25; 1.0]
        ruddertwist         = [0.0; 0.0; 0.0]
        ruddersweep         = [0.0; 0.0]
        rudderdihedral      = [0.0; 0.0]
        ruddernumlattice    = round(Int,latticepermeter*2*rudderspan)
        ruddersymmetry      = false
        rudderpos           = [vstabxpos+vstabrootchord; 0.0; vstabzpos] #7.59
    end #if we want the empennage

    #If we want to include the rotors, do it here.
    # --- Rotors --- #
    rotorspacing    = 2.4
    if includerotors == true
        toprotorxpos    = -1.4
        bottomrotorxpos = toprotorxpos #-1.35

        #if we have inline rotors
        if inlinerotors == true
            if numrotors == 1
                n_rotors    = 1
                rotorpos1   = [toprotorxpos; 0.0; 0.0]
                rotorposs   = rotorpos1
                rotorccw    = [1]
            elseif numrotors == 2
                n_rotors = 2
                rotorpos1   = [toprotorxpos; 0.0; rotorspacing/2.0]
                rotorpos2   = [toprotorxpos; 0.0; -rotorspacing/2.0]
                rotorposs   = [rotorpos1 rotorpos2]
                if counterrotate == true
                    rotorccw        = [1,2]
                else
                    rotorccw        = [1,1]
                end
            else
                gt.verbalize("number specified rotors: $numrotors.  4 being defined...", v_lvl, verbose)
                n_rotors        = 4
                rotorpos1  = [toprotorxpos; 0.5*rotorspacing; rotorspacing/2.0]
                rotorpos2  = [toprotorxpos; 0.5*rotorspacing; -rotorspacing/2.0]
                rotorpos3  = [toprotorxpos; -0.5*rotorspacing; rotorspacing/2.0]
                rotorpos4  = [toprotorxpos; -0.5*rotorspacing; -rotorspacing/2.0]
                rotorposs  = [rotorpos1 rotorpos2 rotorpos3 rotorpos4]
                rotorccw        = [1,2,1,2]
            end
        #if we have offset rotors
        else
            n_rotors        = 8
            toprotorxpos    = -1.4
            bottomrotorxpos = -1.35
            rotorspacing    = 2.4
            # Position of rotors on main wing numbering starts on top right (lookingforward from windcraft) and reads counterclockwise
            rotorpos1 = [toprotorxpos; 1.5*rotorspacing; rotorspacing/2.0]
            rotorpos2 = [toprotorxpos; 0.5*rotorspacing; rotorspacing/2.0]
            rotorpos3 = [toprotorxpos; -0.5*rotorspacing; rotorspacing/2.0]
            rotorpos4 = [toprotorxpos; -1.5*rotorspacing; rotorspacing/2.0]
            rotorpos5 = [bottomrotorxpos; -1.5*rotorspacing; -rotorspacing/2.0]
            rotorpos6 = [bottomrotorxpos; -0.5*rotorspacing; -rotorspacing/2.0]
            rotorpos7 = [bottomrotorxpos; 0.5*rotorspacing; -rotorspacing/2.0]
            rotorpos8 = [bottomrotorxpos; 1.5*rotorspacing; -rotorspacing/2.0]
            rotorposs = [rotorpos1 rotorpos2 rotorpos3 rotorpos4 rotorpos5 rotorpos6 rotorpos7 rotorpos8]
            rotorccw  = [1,1,2,2,1,1,2,2]
        end

        rotorpitch       = 0.0
        # numbladeelements = 15 #todo how many?

    end #if we want rotors

    # if we want to add the pylons, do that here.
    if includepylons == true
        # --- Pylons (wing systems) --- #
        #due to odd geometry, need to make pylons 2-wing (complex) systems.
        # Top section
        pylontoprootchord   = 0.7
        pylontoptipchord    = 0.9
        pylontopspan        = rotorspacing
        pylontopAR          = pylontopspan/pylontoptipchord
        pylontopchordlength = [pylontoprootchord; pylontoptipchord]
        pylontopchordpos    = [0.0; 1.0]
        pylontoptwist       = [0.0; 0.0]
        pylontopsweep       = [-6.0]
        pylontopdihedral    = [0.0]
        pylontopnumlattice  = round(Int,latticepermeter*2*pylontopspan)
        pylontopsymmetry    = false
        pylonxshift         = -0.9
        pylonspacing        = 2.4
        pylontoppos1        = [pylonxshift; 1.5*pylonspacing; 0]
        pylontoppos2        = [pylonxshift; 0.5*pylonspacing; 0]
        pylontoppos3        = [pylonxshift; -0.5*pylonspacing; 0]
        pylontoppos4        = [pylonxshift; -1.5*pylonspacing; 0]
        pylontopaxis        = [1.0 0.0 0.0; 0.0 0.0 1.0; 0.0 1.0 0.0]

        # Bottom Section
        pylonbottomspan         = rotorspacing
        pylonbottomrootchord    = 1.7
        pylonbottomtipchord     = 1.0
        pylonbottomAR           = pylonbottomspan/pylonbottomtipchord
        pylonbottomchordlength  = [pylonbottomrootchord; pylonbottomtipchord]
        pylonbottomchordpos     = [0.0; 1.0]
        pylonbottomtwist        = [0.0; 0.0]
        pylonbottomsweep        = [3.0]
        pylonbottomdihedral     = [0.0]
        pylonbottomnumlattice   = round(Int,latticepermeter*2*pylonbottomspan)
        pylonbottomsymmetry     = false
        pylonbottompos1         = [pylonxshift; 1.5*pylonspacing; 0]
        pylonbottompos2         = [pylonxshift; 0.5*pylonspacing; 0]
        pylonbottompos3         = [pylonxshift; -0.5*pylonspacing; 0]
        pylonbottompos4         = [pylonxshift; -1.5*pylonspacing; 0]
        pylonbottomaxis         = [1.0 0.0 0.0; 0.0 0.0 -1.0; 0.0 1.0 0.0]
    end #if we want pylons



    ############################################################################
    # ASSEMBLY: This block actually builds the geometry using the above parameters
    ############################################################################

    gt.verbalize("Generating geometry...", v_lvl, verbose)

    # --- Generate MAIN WING --- #
    gt.verbalize("Generating Main Ming...", v_lvl, verbose)

    if includewing == true
        mainwing = vlm.complexWing(mainwingspan, mainwingAR, mainwingnumlattice, mainwingchordpos, mainwingchordlength, mainwingtwist, mainwingsweep, mainwingdihedral; symmetric=true, chordalign=0.0,_ign1=true) #? what is _ign1=true
        #set wing an angle of attack
        vlm.setcoordsystem(mainwing, vehicleorigin, rotate("y",-wingalpha,unit="deg")*vehicleaxis)
    end

    if includewing ==true && includecontrols == true
        gt.verbalize("Generating Control Surfaces...", v_lvl, verbose)
        # --- Generate Control surfaces --- #
        rightaileron = vlm.complexWing(tipspan, aileronAR, aileronnumlattice, rightaileronchordpos, aileronchordlength, ailerontwist,
        aileronsweep, ailerondihedral; symmetric=false, chordalign=0.0,_ign1=true)
        vlm.setcoordsystem(rightaileron, rightaileronpos,rightaileronaxis)

        rightflap = vlm.complexWing(rootspan, flapAR, flapnumlattice, flapchordpos, flapchordlength, flaptwist, flapsweep, flapdihedral; symmetric=false, chordalign=0.0,_ign1=true)
        vlm.setcoordsystem(rightflap, flappos, vehicleaxis)

        leftflap = vlm.complexWing(rootspan, flapAR, flapnumlattice, leftflapchordpos, flapchordlength, flaptwist, flapsweep, flapdihedral; symmetric=false, chordalign=0.0,_ign1=true)
        vlm.setcoordsystem(leftflap, flappos, vehicleaxis)

        leftaileron = vlm.complexWing(tipspan, aileronAR, aileronnumlattice, leftaileronchordpos, aileronchordlength, ailerontwist,
        aileronsweep, -ailerondihedral; symmetric=false, chordalign=0.0,_ign1=true)
        vlm.setcoordsystem(leftaileron, leftaileronpos, leftaileronaxis)
    end

    # --- Assemble Main Wing (with control surfaces) --- #
    # Assembles moving sections of the wing
    mainwingsystem = vlm.WingSystem()
    if includewing == true
        vlm.addwing(mainwingsystem, "mainwing", mainwing)
    end

    if includewing == true && includecontrols == true
        vlm.addwing(mainwingsystem, "rightaileron", rightaileron)
        vlm.addwing(mainwingsystem, "rightflap", rightflap)
        vlm.addwing(mainwingsystem, "leftflap", leftflap)
        vlm.addwing(mainwingsystem, "leftaileron", leftaileron)
    end
    # --- Start Whole System Assembly --- #
    # Create system assembly
    gt.verbalize("Initializing System Assembly...", v_lvl, verbose)
    system = vlm.WingSystem()
    vlm.addwing(system, "MainWing", mainwingsystem)


    gt.verbalize("Initializing VLM System...", v_lvl, verbose)
    # System to solve through the VLM solver
    vlm_system = vlm.WingSystem()
    if includewing == true
        vlm.addwing(vlm_system, "mainwing", mainwing)
    end

    if includewing == true && includecontrols == true
        vlm.addwing(vlm_system, "rightaileron", rightaileron)
        vlm.addwing(vlm_system, "rightflap", rightflap)
        vlm.addwing(vlm_system, "leftflap", leftflap)
        vlm.addwing(vlm_system, "leftaileron", leftaileron)
    end

    #make dummy rotor in case there aren't any
    rotors = vlm.Rotor[]
    rotor_systems = ()

    #if we want rotors, generate them and add them to the wing system here
    if includerotors == true
        gt.verbalize("Generating Rotors...", v_lvl, verbose)
        # --- Generate ROTORS --- #
        R, B = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]]
        ##more rotor definition stuff:
        n = 200.0 #target RPS
        Vinf = 40.0 #freestream velocity
        vind = sqrt( Vinf^2 + (n*0.84)^2 ) #velocity at 70% blade span
        # Simulation parameters
        J = Vinf/(n*2.0*R)                      # Advance ratio Vinf/(nD)
        rho = 1.225                         # (kg/m^3) air density
        mu = 1.81e-5                        # (kg/ms) air dynamic viscosity
        ReD07 = rho*2.0*R*0.7*vind/mu            # Diameter-based Reynolds at 70% span
        ReD = ReD07/0.7                     # Diameter-based Reynolds

        # Generates base rotors (one on each rotation orientation)
        props = vlm.Rotor[]
        gt.verbalize("Generating first propeller...", v_lvl, verbose)
        @time push!(props, uns.generate_rotor(rotor_file; pitch=rotorpitch,
        n=numbladeelements, CW=true, ReD=ReD,
        verbose=verbose, xfoil=xfoil,
        data_path=data_path,
        # plot_disc=plot_disc,
        verbose=verbose, v_lvl=v_lvl+2))

        gt.verbalize("Generating second propeller...", v_lvl, verbose)
        @time push!(props, vlm.Rotor(!props[1].CW, props[1].r,
                                props[1].chord, props[1].theta,
                                props[1].LE_x, props[1].LE_z,
                                props[1].B, props[1].airfoils))
        @time vlm.initialize(props[2], props[1].m)

        # --- Assemble Rotors --- #
        gt.verbalize("Adding Rotors to Main Wing...", v_lvl, verbose)
        rotors = vlm.Rotor[]
        for i = 1:n_rotors
            copy_prop = props[rotorccw[i]]
            this_prop = deepcopy(copy_prop) # Alternates rotation orientation
            this_O = rotorposs[:,i]
            vlm.setcoordsystem(this_prop, this_O, vehicleaxis; user=true)

            # Rotates props to be tip to tip #?what does this do?
            # vlm.rotate(this_prop, (-1)^(!CW_w) * init_ori_prop)

            # Adds the original polars that don't get copied in deepcopy
            this_prop.airfoils = copy_prop.airfoils
            this_prop._polars = copy_prop._polars
            this_prop._polarroot = copy_prop._polarroot
            this_prop._polartip = copy_prop._polartip

            push!(rotors, this_prop)
        end

        for (i, rotor) in enumerate(rotors)
            vlm.addwing(mainwingsystem, "rotor$i", rotor)
        end

        # --- Define rotor system --- #
        gt.verbalize("Creating Rotor System...", v_lvl, verbose)
        # Rotors grouped by systems of the same RPM
        rotor_systems = (rotors,)
    end #if adding rotors



    # if we want an empennage, generate it and add it here.
    if includetail == true
        # --- Generate Empennage --- #
        gt.verbalize("Generating Empennage...", v_lvl, verbose)

        gt.verbalize("Generating Horizontal Stabilizer...", v_lvl, verbose)
        #Hstab:
        horizontalstabilizer = vlm.simpleWing(hstabspan, hstabAR, hstabtaperratio, hstabtwist, hstabsweep, hstabdihedral)
        vlm.setcoordsystem(horizontalstabilizer,hstabpos,hstabaxis)

        gt.verbalize("Generating Vertical Stabilizer...", v_lvl, verbose)
        #Vstab:
        verticalstabilizer = vlm.complexWing(vstabspan,vstabAR,vstabnumlattice,vstabchordpos,vstablchordlength,vstabtwist,vstabsweep,vstabdihedral,symmetric=false, chordalign=1.0, _ign1=true)
        vlm.setcoordsystem(verticalstabilizer,vstabpos,vstabaxis)
        rudder = vlm.complexWing(rudderspan,rudderAR,ruddernumlattice,rudderchordpos,rudderlchordlength,ruddertwist,ruddersweep,rudderdihedral,symmetric=false, chordalign=0, _ign1=true)
        vlm.setcoordsystem(rudder,rudderpos,vstabaxis)

        # --- Assemble Empennage --- #
        empennagesystem = vlm.WingSystem()
        vlm.addwing(empennagesystem, "hstab", horizontalstabilizer)
        vlm.addwing(empennagesystem, "vstab", verticalstabilizer)
        vlm.addwing(empennagesystem, "rudder", rudder)

        # --- Add Empennage to System Assembly --- #
        gt.verbalize("Adding Empennage to System Assembly...", v_lvl, verbose)
        vlm.addwing(system, "Empennage", empennagesystem)

        # --- Add Empennage to VLM Assembly --- #
        gt.verbalize("Adding Empennage to VLM Assembly...", v_lvl, verbose)
        vlm.addwing(vlm_system, "vstab", verticalstabilizer)
        vlm.addwing(vlm_system, "rudder", rudder)
        vlm.addwing(vlm_system, "hstab", horizontalstabilizer)

    end #if adding empennage

    #if we wanted the pylons, generate and add here
    if includepylons == true
        gt.verbalize("Generating Pylons...", v_lvl, verbose)
        # --- Generate Pylons --- #
        pylon1t = vlm.complexWing(pylontopspan,pylontopAR,pylontopnumlattice,pylontopchordpos,pylontopchordlength,pylontoptwist,pylontopsweep,pylontopdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon1t,pylontoppos1,pylontopaxis)
        pylon1b = vlm.complexWing(pylonbottomspan,pylonbottomAR,pylonbottomnumlattice,pylonbottomchordpos,pylonbottomchordlength,pylonbottomtwist,pylonbottomsweep,pylonbottomdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon1b,pylonbottompos1,pylonbottomaxis)

        pylon2t = vlm.complexWing(pylontopspan,pylontopAR,pylontopnumlattice,pylontopchordpos,pylontopchordlength,pylontoptwist,pylontopsweep,pylontopdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon2t,pylontoppos2,pylontopaxis)
        pylon2b = vlm.complexWing(pylonbottomspan,pylonbottomAR,pylonbottomnumlattice,pylonbottomchordpos,pylonbottomchordlength,pylonbottomtwist,pylonbottomsweep,pylonbottomdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon2b,pylonbottompos2,pylonbottomaxis)

        pylon3t = vlm.complexWing(pylontopspan,pylontopAR,pylontopnumlattice,pylontopchordpos,pylontopchordlength,pylontoptwist,pylontopsweep,pylontopdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon3t,pylontoppos3,pylontopaxis)
        pylon3b = vlm.complexWing(pylonbottomspan,pylonbottomAR,pylonbottomnumlattice,pylonbottomchordpos,pylonbottomchordlength,pylonbottomtwist,pylonbottomsweep,pylonbottomdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon3b,pylonbottompos3,pylonbottomaxis)

        pylon4t = vlm.complexWing(pylontopspan,pylontopAR,pylontopnumlattice,pylontopchordpos,pylontopchordlength,pylontoptwist,pylontopsweep,pylontopdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon4t,pylontoppos4,pylontopaxis)
        pylon4b = vlm.complexWing(pylonbottomspan,pylonbottomAR,pylonbottomnumlattice,pylonbottomchordpos,pylonbottomchordlength,pylonbottomtwist,pylonbottomsweep,pylonbottomdihedral,symmetric=false, chordalign=0.0, _ign1=true)
        vlm.setcoordsystem(pylon4b,pylonbottompos4,pylonbottomaxis)

        # --- Assemble Pylons --- #
        pylon1 = vlm.WingSystem()
        vlm.addwing(pylon1, "pylon1top", pylon1t)
        vlm.addwing(pylon1, "pylon1bottom", pylon1b)

        pylon2 = vlm.WingSystem()
        vlm.addwing(pylon2, "pylon2top", pylon2t)
        vlm.addwing(pylon2, "pylon2bottom", pylon2b)

        pylon3 = vlm.WingSystem()
        vlm.addwing(pylon3, "pylon3top", pylon3t)
        vlm.addwing(pylon3, "pylon3bottom", pylon3b)

        pylon4 = vlm.WingSystem()
        vlm.addwing(pylon4, "pylon4top", pylon4t)
        vlm.addwing(pylon4, "pylon4bottom", pylon4b)

        pylonsystem = vlm.WingSystem()
        vlm.addwing(pylonsystem, "pylon1", pylon1)
        vlm.addwing(pylonsystem, "pylon2", pylon2)
        vlm.addwing(pylonsystem, "pylon3", pylon3)
        vlm.addwing(pylonsystem, "pylon4", pylon4)

        # --- Add Pylons to System Assembly --- #
        gt.verbalize("Adding Pylons to System Assembly...", v_lvl, verbose)
        vlm.addwing(system, "Pylons", pylonsystem)

        # --- Add Pylons to VLM Assembly --- #
        gt.verbalize("Adding Pylons to VLM Assembly...", v_lvl, verbose)
        vlm.addwing(vlm_system, "pylon1", pylon1)
        vlm.addwing(vlm_system, "pylon2", pylon2)
        vlm.addwing(vlm_system, "pylon3", pylon3)
        vlm.addwing(vlm_system, "pylon4", pylon4)

    end #if adding pylons




    # ------------ OUTPUTS------------------------------------------------

    #Rotate to be in correct starting pos.
    O = zeros(3)
    if circlepath == true
        Oaxis = [0 -1 0; 0 0 1; 1 0 0]
    else
        Oaxis = [1 0 0; 0 1 0; 0 0 1]
    end
    vlm.setcoordsystem(system, O, Oaxis)

    # Tilting systems
    gt.verbalize("Creating Tilting System...", v_lvl, verbose)
    if includetail == true
        tilting_wings = (rightaileron,rightflap,leftflap,leftaileron,rudder,horizontalstabilizer)
    elseif includewing == true && includecontrols == true
        tilting_wings = (rightaileron,rightflap,leftflap,leftaileron)
    else
        tilting_wings = ()
    end

    # Convert the Wings into WingSystems
    tilting_systems = []
    for (wi, wing) in enumerate(tilting_wings)
        sys = vlm.WingSystem()
        vlm.addwing(sys, "TiltWing$wi", wing)
        push!(tilting_systems, sys)
    end
    tilting_systems = Tuple(tilting_systems)


    gt.verbalize("Creating Wake System...", v_lvl, verbose)
    # Wake-shedding system (`vlm_system`+`rotors`)
    wake_system = vlm.WingSystem()
    vlm.addwing(wake_system, "SolveVLM", vlm_system)
    if includerotors == true
        for (i, rotor) in enumerate(rotors)
            vlm.addwing(wake_system, "Rotor$i", rotor)
        end
    end

    gt.verbalize("Completed Geometry Generation and Assembly...", v_lvl, verbose)

    # OTHER MONITOR INPUTS
    # get y position of horseshoes for monitor purposes
    if includewing == true
        mainwinghorshoeypos = mainwing._ym
    else
        mainwinghorshoeypos = []
    end

    return (system, vlm_system, rotors, tilting_systems, rotor_systems, wake_system)

end



"""
CONVENIENT ROTATION FUNCTION
"""
function rotate(axis,theta;unit="deg")
    if unit=="deg"; theta*=pi/180; end; #convert to radians

    if axis == "x"
        return [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)]
    elseif axis == "y"
        return [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]
    elseif axis == "z"
        return [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]
    else
        error("That's not an axis...")
    end
end
