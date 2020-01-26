#=##############################################################################
# DESCRIPTION
Testing rotor-on-wing and wing-on-rotor modeling using a scaled-down Bertin's
wing (45-deg, swept-back, planar wing in Example 7.2, pp. 343 of Bertin's
Aerodynamics for Engineers) with two APC 10x7 propellers (one on each side).

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Dec 2019
  * License   : MIT
=###############################################################################

# ------------ MODULES ---------------------------------------------------------
# Load simulation engine
# import FLOWUnsteady
reload("FLOWUnsteady")
uns = FLOWUnsteady
vlm = uns.vlm

import GeometricTools
gt = GeometricTools

using PyPlot

noise = nothing
try
    # FLOWNoise https://github.com/byuflowlab/FLOWNoise (it requires PSU-WOPWOP)
    import FLOWNoise
    noise = FLOWNoise
catch e
    warn("FLOWNoise not loaded: $e")
    println("FLOWNoise not loaded: $e")
end


# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"
# extdrive_path = "temps/"


# ------------ HEADERS ---------------------------------------------------------
for header_name in ["singleprop", "isolatedwing", "blownwing", "postprocessing",
                                                                        "noise"]
    include("blownwing_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------
function run_singleprop(; xfoil=true, prompt=true)
    singleprop(; xfoil=xfoil, save_path=extdrive_path*"fvs_singleprop04/",
                                                                  prompt=prompt)
end

function run_isolatedwing(; prompt=true)
    isolatedwing(; save_path=extdrive_path*"fvs_isolatedwing10/", prompt=prompt)
end

function run_blownwing(; xfoil=true, prompt=true)
    blownwing(; xfoil=xfoil, save_path=extdrive_path*"fvs_blownwing00/",
                                                                  prompt=prompt)
end

function run_noise(; )

    read_path = "/media/edoalvar/MyExtDrive/simulationdata5/fvs_blownwing11_alloutputs_noise/aero_rawoutputs/"
    save_path = "/media/edoalvar/MyExtDrive/simulationdata5/fvs_blownwing11_alloutputs_noise/test_noise06/"
    postprocessing_noise(  save_path;
                            # ---------- AERO SIMULATION INFO ----------
                            num0=2,                         # First time step to analyze
                            nrevs=29,                       # Number of revs to analyze
                            # ---------- OBSERVERS -------------------------
                            loading=true,                   # Include loading pressure
                            sph_R=1.2*1.693/2, sph_nR=24, sph_ntht=24, # Sphere definition
                            sph_nphi=24, sph_phimax=360,
                            sph_rotation=[90, 0, 0],
                            sph_C=[1.693/4, 0, 0],
                            microphoneX=nothing,
                            ww_nummin=72*15,
                            ww_numless=72*11-2,
                            # ---------- INPUT OPTIONS ---------------------
                            read_path=read_path,
                            # ---------- OUTPUT OPTIONS --------------------
                            prompt=true, debug_paraview=true,
                            debuglvl=1,                     # WW debug level
                        )
end
