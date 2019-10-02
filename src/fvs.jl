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

# ------------ GLOBAL VARIABLES ------------------------------------------------
global module_path; module_path,_ = splitdir(@__FILE__)    # Path to this module
global data_path = joinpath(module_path, "../data/")       # Path to data folder
# Path to external dr
global extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"


# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["rotor", "geometry", "kinematics", "simulation"]
    include("fvs_"*module_name*".jl")
end
