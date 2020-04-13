"""
Mixed-fidelity unsteady aerodynamics simulation engine.

    # AUTHORSHIP
      * Author    : Eduardo J. Alvarez
      * Email     : Edo.AlvarezR@gmail.com
      * Created   : Oct 2019
      * License   : MIT
"""
module FLOWUnsteady

export Vehicle, KinematicManeuver, DynamicManeuver,
        generate_rotor, save_vtk

#=
    NOTES
    * If calculating force coefficients through FLOWVLM, remember to specify
        qinf manually, otherwise it will include the wake-induced velocity in
        the normalizing static pressure.
    * Calculating force coefficients through FLOWVLM doesn't include the
        VPM-induced velocity, but the semi-infinite singular filaments.
    * I would recommend avoid using FLOWVLM's force calculators. Instead use the
        tools under `fvs_processing.jl`.
    * FLOWVLM has a a rough regularization scheme that tends to make the solver
        oscillate if the smoothing radius overlaps any control point. Just FYI.
=#
#=
    TODO
    * Incorporate VPM-induced velocity in VLM force calculations.
=#

# ------------ FLOW CODES ------------------------------------------------------
# FLOWVLM https://github.com/byuflowlab/FLOWVLM
import FLOWVLM
vlm = FLOWVLM


# MyVPM https://github.com/EdoAlvarezR/MyVPM
import MyVPM
vpm = MyVPM

# GeometricTools https://github.com/byuflowlab/GeometricTools.jl
import GeometricTools
gt = GeometricTools

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
import JLD
using PyPlot

# ------------ GLOBAL VARIABLES ------------------------------------------------
module_path = splitdir(@__FILE__)[1]                # Path to this module
def_data_path = joinpath(module_path, "../data/")   # Default path to data folder


# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["vehicle", "vehicle_vlm",
                    "maneuver", "rotor",
                    "simulation_types", "simulation", "utils",
                    "processing", "noise", "monitors"]
    include("FLOWUnsteady_"*module_name*".jl")
end

end # END OF MODULE
