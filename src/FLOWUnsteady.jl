"""
    An interactional aerodynamics and acoustics solver for multirotor aircraft
    and wind energy.

    * Main developer  : Eduardo J. Alvarez (edoalvarez.com)
    * Email           : Edo.AlvarezR@gmail.com
    * Repo            : github.com/byuflowlab/FLOWUnsteady
    * Created         : Sep 2017
    * License         : MIT
"""
module FLOWUnsteady

#= TODO
    * [ ] Change name UVLMVehicle -> UVehicle
=#

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
import DataFrames
import JLD
import Dates
import PyPlot as plt

using PyPlot: @L_str
using LinearAlgebra: I
using Printf: @printf

# ------------ FLOW CODES ------------------------------------------------------
import GeometricTools

# NOTE: Unregistered packages available at https://github.com/byuflowlab
import FLOWVPM
import FLOWVLM
import VSPGeom
import FLOWNoise
import BPM

# Aliases
const gt    = GeometricTools
const vpm   = FLOWVPM
const vlm   = FLOWVLM
const vsp   = VSPGeom
const noise = FLOWNoise

# ------------ GLOBAL VARIABLES ------------------------------------------------
const module_path    = splitdir(@__FILE__)[1]              # Path to this module
const default_database  = joinpath(module_path, "..", "database") # Default path to database
const def_data_path  = default_database
const examples_path  = joinpath(module_path, "..", "examples") # Path to examples

# Identity matrix
const Im = Array(1.0I, 3, 3)

# ------------ HEADERS ---------------------------------------------------------
for header_name in ["vehicle", "vehicle_vlm",
                    "maneuver", "rotor",
                    "simulation_types", "simulation", "utils",
                    "processing", "processing_force", "monitors",
                    "noise_wopwop", "noise_bpm", "postprocessing",
                    "openvsp"]

    include("FLOWUnsteady_"*header_name*".jl")

end

# Format PyPlot
formatpyplot()

# VPM utilities
include(joinpath(vpm.utilities_path, "utilities_fluiddomain.jl"))

end # END OF MODULE
