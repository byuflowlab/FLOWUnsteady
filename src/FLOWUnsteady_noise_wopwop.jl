#=##############################################################################
# DESCRIPTION
    Coupling of FLOWUnsteady with PSU-WOPWOP v3.4.2 for aeroacoustic tonal
    noise.

# ABOUT
  * Created   : Jan 2020
  * License   : MIT
=###############################################################################

"""
Given the path of a FLOWUnsteady simulation `read_path`, it runs the noise
analysis on the rotors of the simulation. It uses PSU-WOPWOP to calculate the
tonal noise from thickness and loading sources on the geometry and aerodynamic
loading.

```julia
run_noise_wopwop(
    read_path::String,              # Path from where to read aerodynamic solution (FLOWUnsteady simulation)
    run_name,                       # Run name (prefix of rotor files to read)
    RPM::Real,                      # Reference RPM to convert `nrevs` to simulation time
    rho::Real, speedofsound::Real,  # Air density and speed of sound
    rotorsystems,                   # `rotorsystems[i][j]` is the number of blades of the j-th rotor in the i-th system
    ww_nrevs,                       # Run PSU-WOPWOP for this many revolutions
    ww_nsteps_per_rev,              # Number of steps per revolution to use in WW
    save_path::String,              # Where to save PSU-WOPWOP results
    wopwopbin;                      # Path to PSU-WOPWOP binary
    nrevs           = nothing,      # Number of revolutions to read (defaults to `ww_nrevs`)
    nsteps_per_rev  = nothing,      # Number of steps per revolution to read (default to `ww_nsteps_per_rev`)

    # ---------- OBSERVERS ---------------------------------------
    Vobserver   = nothing,          # (m/s) velocity of observer (vector)
    sph_R       = 1.5*6.5,          # (m) sphere radius
    sph_nR      = 0,                # Number of cells in radial direction
    sph_ntht = 24, sph_nphi = 24,   # Number of cells in polar and azimuthal directions
    sph_thtmin = 5, sph_thtmax = 175,   # (deg) Bounds of polar direction
    sph_phimax  = 180,              # (deg) maximum azimuthal angle (use 180 to make a hemisphere)
    sph_rotation= [0, 90, 0],       # (degs) rotate the sphere by these angles
    sph_C       = zeros(3),         # (m) center of sphere
    microphoneX = nothing,          # If given, replaces sphere with one observer at this position
    highpass = nothing, lowpass = nothing, # Low and high pass filters
    windowing   = nothing,          # Windowing method
    output_octaves = true,          # Whether to output octave bands
    Noctave     = 3,                # Number of octave bands

    # ---------- SIMULATION OPTIONS -----------------------------
    periodic    = true,             # Whether rotor loading and translation in aerodynamic solution is periodic

    # ---------- INPUT OPTIONS ----------------------------------
    num_min     = 0,                # Start reading loading files from this number
    const_geometry = false,         # Whether to run PSW on a constant geometry, obtained from num_min
    axisrot     = "automatic",      # Axis of rotation to use for constant geometry (defaults to [1,0,0])
    CW          = true,             # Clockwise or counter-clockwise rotation of constant geometry

    # ---------- OUTPUT OPTIONS ---------------------------------
    prompt      = true,             # Whether to prompt the user
    verbose     = true,             # Whether to verbose
    v_lvl       = 0,                # Indentation level when printing verbose
    debug_paraview = false,         # Whether to visualize the grid of observers in Paraview before running
    debuglvl    = 1,                # PSU-WOPWOP debug level
    observerf_name = "observergrid",# .xyz file with observer grid
    case_name   = "runcase",        # Name of case to create and run
)
```

**NOTE:** This function will call the PSU-WOPWOP binary indicated through
`wopwopbin`. This binary is not included with FLOWUnsteady and must be provided
by the user. This method has been tested on PSU-WOPWOP v3.4.2.

**NOTE 2:** Make sure that the simulation was run with `nsteps_save=1`,
otherwise the time in PSU-WOPWOP will get messed up.
"""
function run_noise_wopwop(read_path::String,                        # Path from where to read aerodynamic solution (FLOWUnsteady simulation)
                                    run_name,                       # Run name (prefix of rotor files to read)
                                    RPM::Real,                      # Reference RPM to convert `nrevs` to simulation time
                                    rho::Real, speedofsound::Real,  # Air density and speed of sound
                                    rotorsystems,                   # `rotorsystems[i][j]` is the number of blades of the j-th rotor in the i-th system
                                    ww_nrevs,                       # Run PSU-WOPWOP for this many revolutions
                                    ww_nsteps_per_rev,              # Number of steps per revolution to use in WW
                                    save_path::String,              # Where to save PSU-WOPWOP results
                                    wopwopbin;                      # Path to PSU-WOPWOP binary
                                    nrevs           = nothing,      # Number of revolutions to read (defaults to `ww_nrevs`)
                                    nsteps_per_rev  = nothing,      # Number of steps per revolution to read (default to `ww_nsteps_per_rev`)

                                    # ---------- OBSERVERS --------------------------------------
                                    Vobserver   = nothing,          # (m/s) velocity of observer (vector)
                                    sph_R       = 1.5*6.5,          # (m) sphere radius (for a spherical grid of observers)
                                    sph_nR      = 0,                # Number of cells in radial direction
                                    sph_ntht = 24, sph_nphi = 24,   # Number of cells in polar and azimuthal directions
                                    sph_thtmin = 5, sph_thtmax = 175,   # (deg) Bounds of polar direction
                                    sph_phimax  = 180,              # (deg) maximum azimuthal angle (use 180 to make a hemisphere)
                                    sph_rotation= [0, 90, 0],       # (degs) rotate the sphere by these angles
                                    sph_C       = zeros(3),         # (m) center of sphere
                                    microphoneX = nothing,          # If given, replaces sphere with one observer at this position
                                    highpass = nothing, lowpass = nothing, # Low and high pass filters
                                    windowing   = nothing,          # Windowing method
                                    output_octaves = true,          # Whether to output octave bands
                                    Noctave     = 3,                # Number of octave bands

                                    # ---------- SIMULATION OPTIONS -----------------------------
                                    periodic    = true,             # Whether rotor loading and translation in aerodynamic solution is periodic

                                    # ---------- INPUT OPTIONS ----------------------------------
                                    num_min     = 0,                # Start reading loading files from this number
                                    const_geometry = false,         # Whether to run PSW on a constant geometry, obtained from num_min
                                    axisrot     = "automatic",      # Axis of rotation to use for constant geometry (defaults to [1,0,0])
                                    CW          = true,             # Clockwise or counter-clockwise rotation of constant geometry

                                    # ---------- OUTPUT OPTIONS ---------------------------------
                                    prompt      = true,             # Whether to prompt the user
                                    verbose     = true,             # Whether to verbose
                                    v_lvl       = 0,                # Indentation level when printing verbose
                                    debug_paraview = false,         # Whether to visualize the grid of observers in Paraview before running
                                    debuglvl    = 1,                # PSU-WOPWOP debug level
                                    observerf_name = "observergrid",# .xyz file with observer grid
                                    case_name   = "runcase",        # Name of case to create and run
                                    )

    if num_min==0
        @warn("Got `num_min=0`, and the first step of a simulation is typically under-resolved!"*
                " It is recommended that you start from any other step instead"*
                " (this is specially important in periodic solutions).")
    end

    _axisrot = nothing
    if const_geometry

        _axisrot = (-1)^(!CW) * Float64.( axisrot=="automatic" ? [1, 0, 0] : axisrot )
        # _axisrot = (-1)^(CW) * Float64.( axisrot=="automatic" ? [1, 0, 0] : axisrot )

        if axisrot=="automatic"
            @warn("axisrot set to \"automatic\"; defaulting to $(_axisrot).")
        end
    elseif periodic
        @warn("Running periodic solution."*
              " Make sure that loading is indeed periodic or close to,"*
              " otherwise, prediction might be dominated by FFT numerical noise.")
    end

    ww_nt = ceil(Int, ww_nrevs *  ww_nsteps_per_rev)  # Number of time steps in PSU-WOPWOP
    ww_tMax = ww_nrevs/(RPM/60)     # (s) end time in PSU-WOPWOP
    ww_dt = ww_tMax/ww_nt           # (s) time step size in PSU-WOPWOP
    ww_samplerate = ww_nsteps_per_rev * RPM/60

    _nsteps_per_rev = nsteps_per_rev!=nothing ? nsteps_per_rev : ww_nsteps_per_rev
    _nrevs = nrevs!=nothing ? nrevs : ww_nrevs
    nt = ceil(Int, _nrevs *  _nsteps_per_rev)     # Number of time steps to read
    tMax = _nrevs/(RPM/60)                       # (s) end time to read
    dt = tMax/nt                                # (s) time step size to read

    # num_max = num_min + (ww_nt-1)
    # num_max = num_min + (ww_nt-1) + 1
    num_max = num_min + (nt-1) + 1

    org_pwd = pwd()
    cd(org_pwd)

    f = nothing
    grid = nothing

try

    def_wopwopbin = "wopwop3"       # Copy wopwopbin with this name


    ############################################################################
    # WORK-DIRECTORY SETUP
    ############################################################################
    if verbose; println("\t"^(v_lvl)*"Creating work directory..."); end;

    # Create path where to run this case
    gt.create_path(save_path, prompt)

    # Copy PSU-WOPWOP binary
    cp(wopwopbin, joinpath(save_path, def_wopwopbin); force=true)

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
    mkdir(case_name)

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
      audioFlag=.false.

      OASPLdBFlag = .true.
      OASPLdBAFlag = .true.
      spectrumFlag = .true.
      SPLdBFlag = .true.
      SPLdBAFlag = .true.
      acousticPressureFlag=.true.
      pressureGradient1AFlag = .false.

      debugLevel = $debuglvl

      thicknessNoiseFlag=.true.
      loadingNoiseFlag=.true.
      totalNoiseFlag=.true.

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
    hilopass = ""
    if highpass != nothing
        hilopass *=
        """
            highPassFrequency = $highpass
        """
    end
    if lowpass != nothing
        hilopass *=
        """
            lowPassFrequency = $lowpass
        """
    end
    if windowing != nothing
        hilopass *=
        """
            windowFunction = '$windowing'
        """
    end
    if output_octaves
        hilopass *=
        """
           octaveFlag = .$(output_octaves).
           octaveNumber = $(Noctave)
           octaveApproxFlag = .false.
        """
    end

    if microphoneX != nothing
        # Declare observer in case file
        str =
        """
         &ObserverIn
           nbBase = $(Vobserver!=nothing ? 2 : 0)
           nt   = $(ww_nt)
           xLoc = $(microphoneX[1])
           yLoc = $(microphoneX[2])
           zLoc = $(microphoneX[3])
           tMin = 0.0
           tMax = $(ww_tMax)
           $(hilopass)
         /
        """
        print(f, str)

    else
        if verbose; println("\t"^(v_lvl)*"Creating spherical observer grid..."); end;
        grid = noise.observer_sphere(sph_R, sph_nR, sph_ntht, sph_nphi;
                                    phimax=sph_phimax,
                                    save_path=case_name,
                                    file_name=observerf_name,
                                    rotation=sph_rotation,
                                    thtmin=sph_thtmin, thtmax=sph_thtmax,
                                    C=sph_C,
                                    fmt="plot3d")

        # Declare observer in case file
        str =
        """
         &ObserverIn
           nbBase = $(Vobserver!=nothing ? 1 : 0)
           nt   = $(ww_nt)
           fileName = "$(observerf_name).xyz"
           tMin = 0.0
           tMax = $(ww_tMax)
           $(hilopass)
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
    if Vobserver != nothing
        str =
        """
         &CB
           Title='ObserverTranslation'
           iB=1
           TranslationType='KnownFunction'
           Y0=0,0,0
           VH=$(Vobserver[1]),$(Vobserver[2]),$(Vobserver[3])
         /
        """
        print(f, str)
    end


    ############################################################################
    # GEOMETRY PRE-PROCESSING
    ############################################################################
    nums = const_geometry ? nothing : num_min:num_max

    if verbose; println("\t"^(v_lvl)*"Generating WOPWOP geometry files..."*
                        " (reading num range $(num_min):$(const_geometry ? num_min : nums[end]))"); end;

    # Unique prefixes of blade loft files (one string per blade)
    loftfiles = unique([fn[1:findfirst('.', fn)-1]
                         for fn in readdir(read_path)
                         if occursin("_Blade", fn) && occursin("_loft", fn)])

    nlofts = size(loftfiles, 1)       # Number of lofts

    # Iterate over loft files converting into WOPWOP thickness files
    for fname in loftfiles
        if verbose; println("\t"^(v_lvl+1)*"file $(fname)..."); end;

        fnamein = const_geometry ? fname*".$(num_min)" : fname

        noise.vtk2wopwop(fnamein, fname*".wop";
                            read_path=read_path, save_path=case_name*"/",
                            # nums=vcat(0:(ww_nt-1), 0),
                            nums=nums,
                            t0=0.0, tf=tMax, period=periodic ? tMax : nothing)
    end

    # Collect all lifting-line VTKs into one WOPWOP file per blade
    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
            for bi in 1:nBlades

                fname    = run_name*"_Sys$(si)_Rotor$(ri)_Blade$(bi)_compact"
                fnamein  = const_geometry ? fname*".$(num_min)" : fname

                if verbose; println("\t"^(v_lvl+1)*"file $(fname)..."); end;

                noise.vtk2wopwop(fnamein, fname*".wop";
                                    read_path=read_path, save_path=case_name,
                                    # nums=vcat(0:(ww_nt-1), 0),
                                    nums=nums,
                                    t0=0.0, tf=tMax, period=periodic ? tMax : nothing,
                                    compact=true)
            end
        end
    end

    if verbose; println("\t"^(v_lvl)*"Generating WOPWOP loading files..."); end;

    # Collect all loading files into one WOPWOP loading file per blade
    nums = const_geometry ? [num_min] : num_min:num_max

    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        generate_wopwoploading(read_path, case_name, nums;
                                    # INPUT OPTIONS
                                    filename="loading_Sys$(si)", fieldname="Ftot",
                                    filenameout=run_name*"_Sys$(si)",
                                    # PROCESSING OPTIONS
                                    period=periodic ? tMax : nothing,
                                    t0=0.0, dt=tMax/nt,
                                    structured=false
                                   )
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
    ncompacts = totnblades
    nbcontainer = nlofts + ncompacts

    # NOTE: should this be ww_dt or dt?
    str =
    """
    &ContainerIn
      dTau        = $(ww_dt)
      nbBase      = $(const_geometry ? 1 : 0)
      nbContainer = $(nbcontainer)
    /
    """

    if const_geometry
        # Give RPM for PSU-WOPWOP to rotate the constant geometry
        str *=
        """
        &CB
          Title="Rotor angular velocity"
          rotation=.true.
          AngleType="KnownFunction"
          Omega=$(2*pi*RPM/60)
          AxisValue=$(_axisrot[1]),$(_axisrot[2]),$(_axisrot[3])
        /
        """
    end

    for fname in loftfiles
        # Loft surface patch for thickness
        str *=
        """
        &ContainerIn
          patchGeometryFile="$(fname).wop"
          nbBase=0
        /
        """
    end

    loading_suf = const_geometry ? "constant" : (periodic ? "" : "a")*"periodic"

    for (si, rotors) in enumerate(rotorsystems)     # Iterate over systems
        for (ri, nBlades) in enumerate(rotors)      # Iterate over rotors
            for bi in 1:nBlades
                # Loft surface patch for thickness
                str *=
                """
                &ContainerIn
                  patchGeometryFile="$(run_name)_Sys$(si)_Rotor$(ri)_Blade$(bi)_compact.wop"
                  patchLoadingFile="$(run_name)_Sys$(si)_Rotor$(ri)_Blade$(bi)_loading_$(loading_suf).wop"
                  nbBase=0
                /
                """
            end
        end
    end
    print(f, str)


    if debug_paraview
        if verbose
            println("\t"^(v_lvl)*"Showing Grid before running WOPWOP...")
            println("")
            println("\t"^(v_lvl+1)*"CLOSE PARAVIEW TO CONTINUE")
            println("")
        end
        run(`paraview --data=$prv_str`)
    end


    #######################################################################
    # Run PSU-WOPWOP
    #######################################################################
    if verbose; println("\t"^(v_lvl)*"Running PSU-WOPWOP..."); end;

    # Close case file
    close(f)

    # Run PSU-WOPWOP
    run(`chmod +x $(def_wopwopbin)`)
    run(`./$(def_wopwopbin)`)

catch e
    for (exc, bt) in Base.catch_stack()
       showerror(stdout, exc, bt)
       println()
    end

    # f = open(joinpath(save_path, case_name, "$(case_name).nam"), "w")
    # print(f, "asd")
    if f != nothing; close(f); end;
    cd(org_pwd)
    # println(joinpath(save_path, case_name, "$(case_name).nam"))
    throw(e)
end

    cd(org_pwd)
    if verbose; println("\t"^(v_lvl)*"PSU-WOPWOP is done!"); end;

    return microphoneX == nothing ? grid : nothing
end




"""
    Generates loading files for PSU-WOPWOP from the indicated simulation in
`read_path` using the steps `nums`. If `woptype=Constant`, only one step must
be specified; if `woptype=Periodic`, it is assumed that the specified steps make
one and only one full revolution; if `woptype=Aperiodic`, it is assumed that the
specified steps make one and only one full revolution. This function outputs the
load per unit length in the global coordinate system if periodic, or in the
blade coordinate system if constant.

NOTE: If periodic, there should be steps from 0 degrees up to 360 degrees (or up
to whether the periodic angle is) where `nums[1]` matches the same conditions
than `nums[end]` (rotor simulations with this module will do this by default).
The period of one revolution should be indicated under `period`.

NOTE2: `dt` must be the same time step than specified in the PSU-WOPWOP driver.
"""
function generate_wopwoploading(read_path, save_path, nums;
                                # INPUT OPTIONS
                                # nums=collect(0:72*6-1),
                                filename="loading_Sys1", fieldname="Ftot",
                                filenameout="singlerotor_Sys1",
                                # OUTPUT OPTIONS
                                # save_path=extdrive_path*"val_piv_single16_cont3_wopwop01/",
                                v_lvl=0, verbose=true, prompt=true,
                                wopext="wop", wopbin=true, wopv=1.0,
                                # PROCESSING OPTIONS
                                period=nothing,
                                t0=0.0, dt=nothing,
                                structured=false
                               )

    # 1==Constant, 2==Periodic, 3==Aperiodic
    if length(nums)==1
        Tflag = 1
        woptype = "constant"
    elseif period != nothing
        Tflag = 2
        woptype = "periodic"
    else
        Tflag = 3
        woptype = "aperiodic"
    end

    # ERROR CASES
    if Tflag != 1 && dt == nothing
        error("Received non-constant load without a time step dt.")
    end

    # Binary / ASCII printing
    prnt(f, x) = wopbin ? write(f, x) : print(f, x)
    prntln(f, x) = wopbin ? write(f, x) : print(f, x, "\n")

    # Convertion to 4-bytes numbers
    # NOTE: 4 bytes = 4*8 bites = 32 bites
    fl(x) = Float32(x)
    nt(x) = Int32(x)
    # Convertion to n-bytes string
    st(x::String, n) = x * " "^(n-length(x))

    f = []             # f[i][j] is the file of the j-th blade in the i-th rotor

    for (i,num) in enumerate(nums)                      # Iterate over steps

        # Loading of every rotor in this step
        ftot = JLD.load(joinpath(read_path, filename*".$(num).jld"), fieldname)

        for ri in 1:size(ftot, 1)                     # Iterate over rotors
            for bi in 1:size(ftot[ri], 1)             # Iterate over blades

                if i==1                               # Initialize files

                    if bi==1; push!(f, []); end;

                    # Create file
                    fname = filenameout*"_Rotor$(ri)_Blade$(bi)_loading_"*woptype*"."*wopext
                    push!(f[ri], open(joinpath(save_path, fname), "w"))

                    if wopv==1.0
                        # Magic number
                        prntln(f[ri][bi], nt(42))
                        # Version number
                        prnt(f[ri][bi], nt(1))
                        prntln(f[ri][bi], nt(0))
                        # Comments
                        prntln(f[ri][bi],
                               st("Compact patch loading data file for PSU-WOPWOP (Format v1.0)\n"*
                                  "------------------------------------------------\n"*
                                  "Created for noise-prop-on-prop project (written by Eduardo Alvarez)\n"*
                                  "https://github.com/byuflowlab/alvarezcritchfield2020-noise-prop-on-prop\n"*
                                  "Creation date: $(Dates.now())\n"*
                                  "Units: SI\n"*
                                  "Format: $(Tflag !=1 ? "Global" : "Blade") coordinate system, $(structured ? "" : "un")"*"structured", 1024))

                        # Format string
                        prntln(f[ri][bi], nt(2))               # Functional file flag
                        prntln(f[ri][bi], nt(1))               # Number of zones
                        prntln(f[ri][bi], nt(2^!structured))   # 1==structured, 2==unstructured
                        prntln(f[ri][bi], nt(Tflag))           # 1==Constant, 2==Periodic, 3==Aperiodic
                        prntln(f[ri][bi], nt(2))               # Data centered on 1==node, 2==face
                        prntln(f[ri][bi], nt(2))               # Data 1==pressure, 2==loading, 3==flow
                        if Tflag==1
                            # NOTE: If constant loading, patch-fixed frame is assumed
                            prntln(f[ri][bi], nt(3))           # Frame 1==stationary, 2==rotating, 3==patch-fixed
                        else
                            # NOTE: If non-constant loading, stationary ground frame is assumed
                            prntln(f[ri][bi], nt(1))
                        end
                        prntln(f[ri][bi], nt(1))               # Floating point 1==single, 2==double
                        prntln(f[ri][bi], nt(0))               # WOPWOP secret conspiracy
                        prntln(f[ri][bi], nt(0))               # WOPWOP other secret conspiracy

                        # Zone specificiation
                        prnt(f[ri][bi], nt(1))                 # Number of zones with data
                        prntln(f[ri][bi], nt(-1))              # Zone with data (negative for no thickness)

                        # Patch header
                        # Name
                        prntln(f[ri][bi], st("liftinglineloading", 32))

                        if Tflag == 2
                            # period
                            prntln(f[ri][bi], fl(period) )
                            # nKey
                            # prntln(f[ri][bi], nt(size(nums, 1) + 1) )
                            prntln(f[ri][bi], nt(size(nums, 1)) )

                        elseif Tflag == 3
                            # nTimes
                            prntln(f[ri][bi], nt(size(nums, 1)) )
                        end

                        if !structured
                            # NOTE: Here I assume face-centered data
                            # nbFaces
                            prntln(f[ri][bi], nt( size(ftot[ri][bi], 1) ))
                        else
                            # iMax
                            prntln(f[ri][bi], nt( 1 ))
                            # jMax
                            prntln(f[ri][bi], nt( size(ftot[ri][bi], 1) ))
                        end

                    else
                        error("Got invalid WOPWOP version $wopv")
                    end
                end

                # Keyi or Timei
                if Tflag != 1
                    prntln(f[ri][bi], fl(t0 + (i-1)*dt) )
                end


                # imax × jmax floating point x coordinates
                # imax × jmax floating point y coordinates
                # imax × jmax floating point z coordinates
                for k in 1:3
                    for j in 1:size(ftot[ri][bi], 1)
                        prntln(f[ri][bi], fl(ftot[ri][bi][j][k]) )
                    end
                end


                if i==size(nums, 1)


                    # # Repeat the first step if periodic
                    # if Tflag==2
                    #     ftot0 = JLD.load(joinpath(read_path,"gammas.$(nums[1]).jld"), "ftot")
                    #
                    #     # Keyi
                    #     prntln(f[ri][bi], fl(t0 + i*dt) )
                    #
                    #     # imax × jmax floating point x coordinates
                    #     # imax × jmax floating point y coordinates
                    #     # imax × jmax floating point z coordinates
                    #     for k in 1:3
                    #         for j in 1:size(ftot[ri][bi], 1)
                    #             prntln(f[ri][bi], fl(ftot0[ri][bi][j][k]) )
                    #         end
                    #     end
                    # end

                    close(f[ri][bi])                # Close files
                end

            end
        end

    end
end


"""
Generate lifting lines for acoustic compact patches of rotors
"""
function generate_vtkliftinglines(rotors, rotor_name, save_path;
                                                num=nothing, suf="_compact")

    for (ri, rotor) in enumerate(rotors)

        nHS = vlm.get_mBlade(rotor)

        for bi in 1:rotor.B

            blade = vlm.get_blade(rotor, bi)
            points = [vlm.getHorseshoe(blade, k)[2] for k in 1:nHS]
            push!(points, vlm.getHorseshoe(blade, nHS)[3])
            vtk_line = [i-1 for i in 1:nHS+1]

            gt.generateVTK(rotor_name*"_Rotor$(ri)_Blade$(bi)$(suf)", points;
                                lines=[vtk_line], num=num, path=save_path)

        end
    end

end
