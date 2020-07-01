
function postprocessing_noise(
                                    save_path::String;
                                    # ---------- AERO SIMULATION INFO ----------
                                    num0=600,                       # First time step to analyze
                                    nrevs=3,                        # Number of revs to analyze
                                    nsteps_per_rev=72,              # Time steps per revolution
                                    dt=0.195516/2160,               # Time step size
                                    Vvehicle=23.384*[-1, 0, 0],     # Velocity of vehicle
                                    rotorsystems=[[2, 2]],          # rotorsystems[si][ri] is the number of blades of the ri-th rotor in the si-th system
                                    rho=1.225,                      # (kg/m^3) air density
                                    speedofsound=343,               # (m/s) speed of sound
                                    # ---------- OBSERVERS -------------------------
                                    loading=true,                   # Include loading pressure
                                    sph_R=1.5*1.693, sph_nR=0, sph_ntht=24, # Sphere definition
                                    sph_nphi=24, sph_phimax=180,
                                    sph_rotation=[0, 90, 0],
                                    sph_C=zeros(3),
                                    microphoneX=nothing,            # If given, replaces sphere with one observer at this position
                                    ww_nummin=0,                    # Starts calculating acoustics after this time steps
                                    ww_numless=0,                   # It doesn't calculate acoustic on this last time steps
                                    # ---------- INPUT OPTIONS ---------------------
                                    read_path=extdrive_path = "/Users/randerson/Box/BYU/FLOWResearch/data/fvs_blownwing11_alloutputs_noise/aero_rawoutputs/",
                                    wopwopbin="/home/edoalvar/Dropbox/FLOWResearch/OtherCodes/PSU-WOPWOP_v3.4.3_B3385/wopwop3_serial", # WW binary
                                    # ---------- OUTPUT OPTIONS --------------------
                                    verbose=true, v_lvl=0,
                                    prompt=true, debug_paraview=false,
                                    debuglvl=1,                     # WW debug level
                                    observerf_name="observergrid",  # .xyz file with observer grid
                                    rotor_name="blownwing",       # Rotor name for vtks
                                    case_name="runcase",            # Name of case to create and run
                                    )

    numf = ceil(Int, num0 + nrevs*nsteps_per_rev)  # Final step to analyze
    tmax = dt*(numf-num0)               # Final time in aero simulation

    # ww_nt = numf-num0+1             # Number of time steps in PSU-WOPWOP
    # ww_tMax = tmax                  # (s) end time in PSU-WOPWOP
    # ww_dt = ww_tMax/ww_nt           # (s) time step size in PSU-WOPWOP

    ww_tMin = dt*ww_nummin            # (s) start time in PSU-WOPWOP
    ww_nt = (numf-ww_numless)-(num0+ww_nummin)       # Number of time steps in PSU-WOPWOP
    ww_dt = dt                        # (s) time step size in PSU-WOPWOP
    ww_tMax = ww_tMin + dt*ww_nt      # (s) end time in PSU-WOPWOP

    org_pwd = pwd()
    cd(org_pwd)

    def_wopwopbin = "wopwop3"       # Copy wopwopbin with this name


    if verbose; println("\t"^(v_lvl)*"Aero time steps range: $(num0) to $(numf)"); end;
    if verbose; println("\t"^(v_lvl)*"WOPWOP time steps range: $(num0+ww_nummin) to $(numf-ww_numless)"); end;

    if ww_nt<=0
        error("Insufficient time steps in WOPWOP: ww_nt=$(ww_nt)")
    end

    ############################################################################
    # WORK-DIRECTORY SETUP
    ############################################################################
    if verbose; println("\t"^(v_lvl)*"Creating work directory..."); end;

    # Create path where to run this case
    gt.create_path(save_path, prompt)

    # Copy PSU-WOPWOP binary
    cp(wopwopbin, joinpath(save_path, def_wopwopbin); remove_destination=true)

    # Make that path the work directory
    cd(save_path)

    # Create file with cases to run
    f = open("cases.nam", "w")
    print(f,
    """
    &casename
    globalFolderName='./$(case_name)/'
    caseNameFile='$(case_name).nam'
    /
    """
    )
    close(f)

    # Create directory of case to run
    if !ispath(case_name)
        mkdir(case_name)
    end

    # String of VTKs
    prv_str = case_name*"/"


    # Create file case driver
    f = open(joinpath(case_name, "$(case_name).nam"), "w")

    # Environment
    str =
    """
    &EnvironmentIn

      nbContainer=1
      ASCIIOutputFlag=.true.
      audioFlag=.$(microphoneX!=nothing).

      OASPLdBFlag = .true.
      OASPLdBAFlag = .true.
      spectrumFlag = .true.
      SPLdBFlag = .true.
      SPLdBAFlag = .true.
      acousticPressureFlag=.true.
      pressureGradient1AFlag = .false.

      debugLevel = $debuglvl

      thicknessNoiseFlag=.true.
      loadingNoiseFlag=.$(loading).
      totalNoiseFlag=.$(loading).

    /
    &EnvironmentConstants
      rho   = $rho
      c     = $speedofsound
    /
    """
    print(f, str)

    ############################################################################
    # SPHERE/MICROPHONE OBSERVER SETUP
    ############################################################################
    if microphoneX != nothing
        # Declare observer in case file
        str =
        """
         &ObserverIn
           nbBase = $(Vvehicle!=nothing ? 2 : 0)
           nt   = $(ww_nt)
           xLoc = $(microphoneX[1])
           yLoc = $(microphoneX[2])
           zLoc = $(microphoneX[3])
           tMin = $(ww_tMin)
           tMax = $(ww_tMax)
         /
        """
        print(f, str)

    else
        if verbose; println("\t"^(v_lvl)*"Creating spherical observer grid..."); end;
        grid = noise.observer_sphere(sph_R, sph_nR, sph_ntht, sph_nphi;
                                    phimax=sph_phimax,
                                    C=sph_C,
                                    save_path=case_name,
                                    file_name=observerf_name,
                                    rotation=sph_rotation,
                                    fmt="plot3d")

        # Declare observer in case file
        str =
        """
         &ObserverIn
           nbBase = $(Vvehicle!=nothing ? 1 : 0)
           nt   = $(ww_nt)
           fileName = "$(observerf_name).xyz"
           tMin = $(ww_tMin)
           tMax = $(ww_tMax)
         /
        """
        print(f, str)

        if debug_paraview

            gt.save(grid, observerf_name; path=case_name*"/")
            prv_str *= observerf_name*".vtk;"

            # run(`paraview --data=$prv_str`)
        end
    end

    # Translates the observer with the velocity of the vehicle
    if Vvehicle != nothing
        str =
        """
         &CB
           Title='ObserverTranslation'
           iB=1
           TranslationType='KnownFunction'
           Y0=$(dt*num0*Vvehicle[1]),$(dt*num0*Vvehicle[2]),$(dt*num0*Vvehicle[3])
           VH=$(Vvehicle[1]),$(Vvehicle[2]),$(Vvehicle[3])
         /
        """
        print(f, str)
    end


    ############################################################################
    # GEOMETRY PRE-PROCESSING
    ############################################################################
    if verbose; println("\t"^(v_lvl)*"Generating WOPWOP geometry files..."); end;

    # Collect all loft VTKs into one WOPWOP file per blade
    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
            for bi in 1:nBlades

                # NOTE: Here I am hardcoding the name suffix of the rotor without including the system
                # You'll have to fix this to make it more general
                fname = rotor_name*"_Rotor$(ri)_Blade$(bi)_loft"
                fnameout = rotor_name*"_Sys$(si)_Rotor$(ri)_Blade$(bi)_loft"

                if verbose; println("\t"^(v_lvl+1)*"file $(fname)..."); end;

                noise.vtk2wopwop(fname, fnameout*".wop";
                                    read_path=read_path, save_path=case_name,
                                    nums=num0:numf,
                                    t0=0.0, tf=tmax, period=nothing)
            end
        end
    end


    # Collect all lifting-line VTKs into one WOPWOP file per blade
    if loading
        for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
            for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
                for bi in 1:nBlades

                    fname = rotor_name*"_Sys$(si)_Rotor$(ri)_Blade$(bi)_compact"
                    fnameout = rotor_name*"_Sys$(si)_Rotor$(ri)_Blade$(bi)_compact"

                    if verbose; println("\t"^(v_lvl+1)*"file $(fname)..."); end;

                    noise.vtk2wopwop(fname, fnameout*".wop";
                                        read_path=read_path, save_path=case_name,
                                        nums=num0:numf,
                                        t0=0.0, tf=tmax, period=nothing,
                                        compact=true)
                end
            end
        end
    end

    if verbose; println("\t"^(v_lvl)*"Generating WOPWOP loading files..."); end;

    # Collect all loading files into one WOPWOP loading file per blade
    if loading
        for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
            uns.generate_wopwoploading(read_path, case_name, num0:numf;
                        # INPUT OPTIONS
                        filename="loading_Sys$(si)", fieldname="Ftot",
                        filenameout=rotor_name*"_Sys$(si)",
                        # PROCESSING OPTIONS
                        period=nothing,
                        t0=0.0, dt=dt,
                        structured=false
                       )
        end
    end


    ############################################################################
    # SPECIFY ROTOR FILES IN PSU-WOPWOP
    ############################################################################

    # Count total number of blades
    totnblades = 0
    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
            totnblades += nBlades
        end
    end

    # Build rotor description for PSU-WOPWOP
    str =
    """
    &ContainerIn
      dTau        = $(ww_dt)
      nbBase      = 0
      nbContainer = $((2^loading)*totnblades)
    /
    """
    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
            for bi in 1:nBlades
                # Loft surface patch for thickness
                str *=
                """
                &ContainerIn
                  patchGeometryFile="$(rotor_name)_Sys$(si)_Rotor$(ri)_Blade$(bi)_loft.wop"
                  nbBase=0
                /
                """
                if loading
                    str *=
                    """
                    &ContainerIn
                      patchGeometryFile="$(rotor_name)_Sys$(si)_Rotor$(ri)_Blade$(bi)_compact.wop"
                      patchLoadingFile="$(rotor_name)_Sys$(si)_Rotor$(ri)_Blade$(bi)_loading_aperiodic.wop"
                      nbBase=0
                    /
                    """
                end
            end
        end
    end

    print(f, str)


    if debug_paraview && microphoneX==nothing
        if verbose
            println("\t"^(v_lvl)*"Showing Grid and rotor before running WOPWOP...")
            println("")
            println("\t"^(v_lvl+1)*"CLOSE PARAVIEW TO CONTINUE")
            println("")
        end
        run(`paraview --data=$prv_str`)
    end


    #######################################################################
    # Run PSU-WOPWOP
    #######################################################################
    if verbose; println("\t"^(v_lvl)*"Running PSU-WOPWOP (see log.txt)..."); end;

    # Close case file
    close(f)

    # Run PSU-WOPWOP
    cmd2 = "> log.txt"
    run(`chmod +x $(def_wopwopbin)`)
    run(`./$(def_wopwopbin) $cmd2`)

    cd(org_pwd)

    return microphoneX == nothing ? grid : nothing
end
