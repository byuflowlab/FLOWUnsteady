"""
Flight Vehicle Simulator; a high-fidelity simulation engine of fully-unsteady
flight vehicle.

    # AUTHORSHIP
      * Author    : Eduardo J. Alvarez
      * Email     : Edo.AlvarezR@gmail.com
      * Created   : Oct 2019
      * License   : MIT
"""
module FLOWFVS

export Vehicle

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
gt = vlm.vtk

# ------------ GENERIC MODULES -------------------------------------------------
import Dierckx
import CSV
using PyPlot

# ------------ GLOBAL VARIABLES ------------------------------------------------
module_path = splitdir(@__FILE__)[1]                # Path to this module
data_path = joinpath(module_path, "../data/")       # Path to data folder


# ------------ HEADERS ---------------------------------------------------------
# Load modules
for module_name in ["vehicle", "maneuver"]
    include("FLOWFVS_"*module_name*".jl")
end

end # END OF MODULE
