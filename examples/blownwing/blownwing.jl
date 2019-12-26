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
# import FLOWFVS
reload("FLOWFVS")
fvs = FLOWFVS
vlm = fvs.vlm

import GeometricTools
gt = GeometricTools

using PyPlot


# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata5/"
# extdrive_path = "temps/"


# ------------ HEADERS ---------------------------------------------------------
for header_name in ["singleprop"]
    include("blownwing_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------
function run_singleprop(; xfoil=true, prompt=true)
    singleprop(; xfoil=xfoil, save_path=extdrive_path*"fvs_singleprop01/",
                  prompt=prompt)
end
