"""
    An interactional aerodynamics and acoustics solver for multirotor aircraft
    and wind energy.

    * Main developer  : Eduardo J. Alvarez (edoalvarez.com)
    * Email           : Edo.AlvarezR@gmail.com
    * Repo            : github.com/byuflowlab/FLOWUnsteady
    * Created         : Oct 2019
    * License         : MIT
"""
module FLOWUnsteady

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
import DataFrames
import JLD
import Dates
using PyPlot
using LinearAlgebra: I

# ------------ FLOW CODES ------------------------------------------------------
import GeometricTools

# NOTE: Unregistered packages available at https://github.com/byuflowlab
import FLOWVPM
import FLOWVLM
import FLOWNoise
import BPM

# Aliases
const gt    = GeometricTools
const vpm   = FLOWVPM
const vlm   = FLOWVLM
const noise = FLOWNoise

# ------------ GLOBAL VARIABLES ------------------------------------------------
const module_path    = splitdir(@__FILE__)[1]              # Path to this module
const def_data_path  = joinpath(module_path, "../data/")   # Default path to data folder


# ------------ HEADERS ---------------------------------------------------------
for header_name in ["vehicle", "vehicle_vlm",
                    "maneuver", "rotor",
                    "simulation_types", "simulation", "utils",
                    "processing", "processing_force", "monitors",
                    "noise_wopwop", "noise_bpm"]

    include("FLOWUnsteady_"*header_name*".jl")

end

end # END OF MODULE
