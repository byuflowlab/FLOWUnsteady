#=##############################################################################
# DESCRIPTION
    Coupling of FLOWUnsteady with BPM code aeroacoustic broadband noise.

# AUTHORSHIP
  * Author    : Tyler Critchfield and Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Jul 2020
  * License   : MIT
=###############################################################################


"""
Calculates broadband noise from the Brooks, Pope and Marcolini method using
the BPM.jl package.

The function writes the data to a file that can be read in postprocessing
    with FLOWNoise.readwopwopoutput(). It writes three files: SPL
    (non-weighted), A-weighted SPL, and OASPL.
"""
function run_noise_bpm(rotors::Array{vlm.Rotor, 1},
                                RPM::Real, Vinf::Function,
                                rho::Real, mu::Real, speedofsound::Real,
                                save_path::String;
                                # ---------- OBSERVERS -------------------------
                                sph_R=1.5*6.5, sph_nR=0, sph_ntht=24, # Sphere definition
                                sph_nphi=24, sph_phimax=180,
                                sph_rotation=[0, 90, 0],
                                sph_thtmin=5, sph_thtmax=175,
                                sph_C=zeros(3),
                                microphoneX=nothing,            # If given, replaces sphere with one observer at this position
                                # ---------- BPM OPTIONS -----------------------
                                noise_correction=0.5,           # broadband calibration
                                TE_thickness::Union{Float64, Array{Float64, 1}}=16.15,  # TE thickness (in degrees)
                                freq_bins=BPM.default_f,        # Frequency bins
                                db_offset=BPM.default_AdB,      # dB offset of every frequency for A-weighting
                                # ---------- OUTPUT OPTIONS --------------------
                                prompt=true,
                                verbose=true, v_lvl=0,
                                )


    # Create save path
    gt.create_path(save_path, prompt)

    grid = nothing

    # Create observer
    if microphoneX == nothing

        grid = noise.observer_sphere(sph_R, sph_nR, sph_ntht, sph_nphi;
                                        phimax=sph_phimax,
                                        rotation=sph_rotation,
                                        thtmin=sph_thtmin, thtmax=sph_thtmax,
                                        C=sph_C)

    end

    observer = microphoneX != nothing ?  microphoneX : grid


    ############################################################################
    # BPM PARAMETERS
    ############################################################################

    x = [rotor._wingsystem.O[1] for rotor in rotors]
    y = [rotor._wingsystem.O[2] for rotor in rotors]
    rpm = [RPM for rotor in rotors]

    nrotors = length(x)

    magVinf = norm(Vinf(0.0, 0.0))
    winddir = fill(0, nrotors) #south #? Does it matter?
    windvel = fill(magVinf, nrotors)
    B = [rotor.B for rotor in rotors] # number of blades, 2 for now
    h = 10.0 # height of rotor, not really applicable in this case but I add an arbitrary height just in case and then offset the observers this much in the z direction as well
    hs = fill(h, nrotors)
    rad = [rotor.r for rotor in rotors]
    c = [rotor.chord for rotor in rotors]
    c1 = c * 0.25
    alpha = [rotor.theta for rotor in rotors]
    nu = mu/rho
    c0 = speedofsound
    # psi = 16.15 # for now - from 3856, webplotdigitizer
    AR = [rotor.rotorR/calc_cbar(rotor) for rotor in rotors]
    psi = typeof(TE_thickness)==Float64 ? fill(TE_thickness, nrotors) : TE_thickness
    nf = length(freq_bins)

    if nf != length(db_offset)
        error("Invalid dB offset. Expected $nf elements, got $(length(db_offset)).")
    end



    ############################################################################
    # RUN BPM CODE
    ############################################################################
    if verbose; println("\t"^v_lvl*"Calculating BPM noise..."); end;
    if typeof(observer) == GeometricTools.Grid

        OASPL = zeros(grid.nnodes, 1)
        OASPLA = zeros(grid.nnodes, 1)
        SPLf = zeros(grid.nnodes, nf)
        SPLfA = zeros(grid.nnodes, nf)

        for i = 1:grid.nnodes
            if verbose && ((i-1)%10==0 || i==grid.nnodes); println("\t"^(v_lvl+1)*"Observer $i out of $(grid.nnodes)"); end;

            obs = grid.nodes[:,i] + [0.0, 0.0, h]
            obs = rotate_observers(obs, 90)

            OASPL[i], OASPLA[i], SPLf[i,:], SPLfA[i,:] = BPM.turbinepos_multi(x, y, obs,
                                    winddir, windvel,
                                    rpm, B, hs,
                                    rad, c, c1, alpha,
                                    nu, c0,
                                    psi, AR,
                                    noise_correction; f=freq_bins, AdB=db_offset)
        end

    else
        obs = observer + [0.0, 0.0, h] # offsetting observers to match height
        obs = rotate_observers(obs, 90)

        OASPL, OASPLA, SPLf, SPLfA = BPM.turbinepos_multi(x, y, obs,
                                                    winddir, windvel,
                                                    rpm, B, hs,
                                                    rad, c, c1, alpha,
                                                    nu, c0,
                                                    psi, AR,
                                                    noise_correction; f=freq_bins, AdB=db_offset)
    end



    ############################################################################
    # OUTPUT BPM RESULTS
    ############################################################################

    # write OASPL
    filename = "OASPLdB.tec"
    f1 = open(joinpath(save_path, filename), "w") #.tec file similar to wopwop outputs
    header = " TITLE='BPM OASPL' " # Title
    header = string(header, "\n", "VARIABLES= 'OASPL at each observer node'")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    write(f1, header)

    # write A-OASPL
    filename = "OASPLdBA.tec"
    f2 = open(joinpath(save_path, filename), "w") #.tec file similar to wopwop outputs
    header = " TITLE='BPM A-weighted OASPL' " # Title
    header = string(header, "\n", "VARIABLES= 'A-OASPL at each observer node'")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    write(f2, header)

    # write SPLf (non-weighted)
    filename = "spl_spectrum.tec"
    f3 = open(joinpath(save_path, filename), "w") #.tec file similar to wopwop outputs
    header = " TITLE='BPM SPL' " # Title
    header = string(header, "\n", "VARIABLES= 'SPL at each observer node (nfrequency rows and nobserver columns)'")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    write(f3, header)

    # write SPLfA (A-weighted)
    filename = "splA_spectrum.tec"
    f4 = open(joinpath(save_path, filename), "w") #.tec file similar to wopwop outputs
    header = " TITLE='BPM A-weighted SPL' " # Title
    header = string(header, "\n", "VARIABLES= 'SPLA at each observer node (nfrequency rows and nobserver columns)'")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    write(f4, header)

    # write frequencies
    filename = "frequencies.tec"
    f5 = open(joinpath(save_path, filename), "w") #.tec file similar to wopwop outputs
    header = " TITLE='BPM frequencies' " # Title
    header = string(header, "\n", "VARIABLES= 'frequencies'")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    header = string(header, "\n", "")
    write(f5, header)

    for i = 1:length(OASPL) # number of observers
        data1 = OASPL[i]
        data2 = OASPLA[i]
        write(f1, string(" ", data1, "\n")) #adding a space at the beginning
        write(f2, string(" ", data2, "\n")) #adding a space at the beginning
    end

    for j = 1:nf
        for i = 1:length(OASPL) # this loop ordering makes it easier to write so that each observer is its own column instead of row
            # flocal = f[j] #TODO: add frequency values to these files
            data3 = SPLf[i,j]
            data4 = SPLfA[i,j]
            write(f3, string(" ", data3)) # space at beginning and between each spl
            write(f4, string(" ", data4))
        end
        write(f5, string(" ", freq_bins[j]))

        write(f3, string("\n")) # go to next line for next frequency
        write(f4, string("\n"))
        write(f5, string("\n"))
    end

    close(f1)
    close(f2)
    close(f3)
    close(f4)
    close(f5)
    if verbose; println("\t"^v_lvl*"BPM calculation is done!"); end;

    return observer
end


"""
    Returns the mean chord of the blade of the given rotor
"""
function calc_cbar(rotor::vlm.Rotor)

    c = rotor.chord
    r = rotor.r

    cbar = [(c[i+1]+c[i])/2 * (r[i+1]-r[i]) for i in 1:(size(r, 1)-1)]
    cbar = sum(cbar)/(r[end]-r[1])

    return cbar
end


"
Rotate observer(s) as the BPM code assumes wind is in the x-y plane.
(So for a rotor in hover, the rotors are offset by 90 deg. We can
instead rotate the observers to get the same results.)

θ is in degrees.

"
function rotate_observers(obs, θ) # θ in degrees

    θ = θ * pi / 180

    R = [cos(θ) -sin(θ) 0;
        sin(θ) cos(θ) 0;
        0 0 1]

    new_obs = R * obs

    return new_obs
end
