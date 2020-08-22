"""
Mixed-fidelity unsteady aerodynamics simulation engine.

    # AUTHORSHIP
      * Author    : Eduardo J. Alvarez
      * Email     : Edo.AlvarezR@gmail.com
      * Created   : Oct 2019
      * License   : MIT
"""
module FLOWUnsteady

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
import JLD
import Dates
using PyPlot
using LinearAlgebra: norm, dot, cross, I

# ------------ FLOW CODES ------------------------------------------------------
# FLOWVLM https://github.com/byuflowlab/FLOWVLM
import FLOWVLM
const vlm = FLOWVLM

# FLOWVPM https://github.com/byuflowlab/FLOWVPM.jl
try                     # Load FLOWVPM if available
    import FLOWVPM
catch e                 # Otherwise load a dummy version of FLOWVPM
    @warn("FLOWVPM module not found. Using dummy module instead.")
    include("FLOWUnsteady_dummy_FLOWVPM.jl")
end
const vpm = FLOWVPM

# GeometricTools https://github.com/byuflowlab/GeometricTools.jl
import GeometricTools
const gt = GeometricTools

import FLOWNoise
const noise = FLOWNoise

# BPM https://github.com/byuflowlab/BPM.jl
import BPM

# ------------ GLOBAL VARIABLES ------------------------------------------------
const module_path = splitdir(@__FILE__)[1]                # Path to this module
const def_data_path = joinpath(module_path, "../data/")   # Default path to data folder


# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["vehicle", "vehicle_vlm",
                    "maneuver", "rotor",
                    "simulation_types", "simulation", "utils",
                    "processing", "monitors",
                    "noise_wopwop", "noise_bpm"]
    include("FLOWUnsteady_"*module_name*".jl")
end

end # END OF MODULE
