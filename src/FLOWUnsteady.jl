"""
    An interactional aerodynamics and acoustics solver for multirotor aircraft
    and wind energy.

    * Main developers : Eduardo J. Alvarez (edoalvarez.com) and Ryan Anderson (rymanderson@gmail.com)
    * Email           : Edo.AlvarezR@gmail.com
    * Repo            : github.com/byuflowlab/FLOWUnsteady
    * Created         : Sep 2017
    * License         : MIT
"""
module FLOWUnsteady

# ------------ GENERIC MODULES -------------------------------------------------

using StaticArrays

# ------------ FLOW CODES ------------------------------------------------------

# NOTE: Unregistered packages available at https://github.com/byuflowlab
using FastMultipole
import VortexLattice as vlm
import FLOWVPM as vpm
import VSPGeom
# import FLOWPanel

# Aliases
const vpm   = FLOWVPM
const vsp   = VSPGeom
# const pnl   = FLOWPanel

# ------------ GLOBAL VARIABLES ------------------------------------------------
const module_path    = splitdir(@__FILE__)[1]              # Path to this module
const default_database  = joinpath(module_path, "..", "database") # Default path to database
const def_data_path  = default_database
const examples_path  = joinpath(module_path, "..", "examples") # Path to examples

# ------------ HEADERS ---------------------------------------------------------
# for header_name in ["vehicle", "vehicle_vlm",
#                     "maneuver", "rotor",
#                     "simulation_types", "simulation", "utils",
#                     "processing", "processing_force", "monitors",
#                     "noise_wopwop", "noise_bpm", "postprocessing",
#                     "openvsp"]#,"panel"]
# 
#     include("FLOWUnsteady_"*header_name*".jl")
# 
# end
include("dummy_sixdof.jl")
include("dummy_flowtrajectories.jl")

# ------------ EXPORTS ---------------------------------------------------------
export RigidBodyState

# VPM utilities
include(joinpath(vpm.utilities_path, "utilities_fluiddomain.jl"))

end # END OF MODULE
