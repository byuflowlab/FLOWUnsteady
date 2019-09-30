#=##############################################################################
# DESCRIPTION
    High-fidelity simulation engine of fully-unsteady flight vehicle.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################


# ------------ FLOW CODES ------------------------------------------------------
# FLOWVLM https://github.com/byuflowlab/FLOWVLM
# flowvlm_path = "/home/edoalvar/Dropbox/FLOWResearch/FLOWCodes/FLOWVLM/"
# include(flowvlm_path*"src/FLOWVLM.jl")
import FLOWVLM
vlm = FLOWVLM

# MyVPM https://github.com/EdoAlvarezR/MyVPM
# myvpm_path = "/home/edoalvar/Dropbox/FLOWResearch/MyCodes/MyVPM/"
# include(myvpm_path*"src/MyVPM.jl")
import MyVPM
vpm = MyVPM

# GeometricTools https://github.com/byuflowlab/GeometricTools.jl
gt = vlm.vtk

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
using PyPlot


# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["rotor", "geometry", "kinematics"]#, "simulation", "run"]
    include("fvs_"*module_name*".jl")
end


# ------------ GLOBAL VARIABLES ------------------------------------------------
global module_path; module_path,_ = splitdir(@__FILE__)    # Path to this module
global data_path = joinpath(module_path, "../data/")       # Path to data folder
                                                           # Path to external dr
global extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"



# ------------------------------------------------------------------------------
function main(; # save_path="temps/vahanasimulation01",
                save_path=extdrive_path*"vahanasimulation01",
                                    prompt=true,
                                    run_name="vahana",
                                    verbose=true, v_lvl=1)

    # # Maneuver to perform
    # maneuver = maneuver_vahana1
    # Vcruise = 0.125 * 125*0.44704            # Cruise speed
    # RPMh_w = 600                            # RPM of main wing rotors in hover
    # telapsed = 60.0                         # Total time to perform maneuver
    # nsteps = 9000                           # Time steps
    # dt = telapsed/nsteps

    # Maneuver to perform
    maneuver = maneuver_vahana1
    Vcruise = 0.25 * 125*0.44704            # Cruise speed
    Vinf(x,t) = 1e-5*[1,0,-1]               # (m/s) freestream velocity, if 0 the simulation will crash
    RPMh_w = 200                            # RPM of main wing rotors in hover
    telapsed = 30.0                         # Total time to perform maneuver
    nsteps = 1500                           # Time steps
    dt = telapsed/nsteps

    # Generate geometry
    (system, rotors, tilting_systems, rotors_tilting_systems,
        fuselage, grounds, strn) = generategeometry_vahana(; n_factor=1,
                                                             xfoil=false,
                                                             data_path=data_path,
                                                             run_name=run_name)

     run_simulation(maneuver, system, rotors, moving_main,
                                  moving_tandem, fuselage, props_w, props_tw;
                                  # SIMULATION OPTIONS
                                  Vcruise=Vcruise,
                                  RPMh_w=RPMh_w,
                                  telapsed=telapsed,
                                  nsteps=nsteps,
                                  Vinf=Vinf,
                                  # OUTPUT OPTIONS
                                  save_path=save_path,
                                  run_name=run_name,
                                  prompt=prompt,
                                  verbose=verbose, v_lvl=v_lvl,
                                  paraview=false)


      # Move landing pad to landing area
      gt.lintransform!(grounds[2], eye(3), Vcruise*telapsed*[-0.25, 0, -0.0025])

      # Save ground
      strn *= run_name*"_FuselageGrid.vtk;"
      strn = replace(strn, ".", "...")

      for (i, ground) in enumerate(grounds)
        gt.save(ground, run_name*"_Ground$i"; path=save_path)
        strn *= run_name*"_Ground$i.vtk;"
      end
      # println(strn)

      # Call paraview
      run(`paraview --data="$save_path/$strn"`)
end
