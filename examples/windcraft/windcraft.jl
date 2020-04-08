#=##############################################################################
# DESCRIPTION
Testing models, dynamics, and control of a wind-harvesting aircraft (a.k.a.,
windcraft). This vehicle is a multirotor aircraft tethered to the ground in
similitud to the Makani M600 Project, whose rotors operate both as propellers
and turbines along sections of a circular path in the presence of cross wind.

This example is a snippet of the study published under J. Mehr, E. Alvarez, A.
Cardoza, and A. Ning, "Mixed-fidelity Aerodynamic Analysis of Wind Harvesting
Aircraft," AIAA AVIATION Forum, 2020. See the paper for further details on
geometry, rotor design, and analysis.

# AUTHORSHIP
  * Author    : Judd Mehr and Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

# ------------ MODULES ---------------------------------------------------------
# Load simulation engine
# import FLOWUnsteady
reload("FLOWUnsteady")
uns = FLOWUnsteady
vlm = uns.vlm
gt = uns.gt

using PyPlot

# ------------ GLOBAL VARIABLES ------------------------------------------------
# Default path where to save data
extdrive_path = "/media/edoalvar/MyExtDrive/simulationdata6/"
# extdrive_path = "temps/"


# ------------ HEADERS ---------------------------------------------------------
for header_name in ["geometry"]
    include("windcraft_"*header_name*".jl")
end


# ------------ DRIVERS ---------------------------------------------------------


"""
    Generates geometry of the windcraft, saves it as vtk files, and calls
    Paraview visualizing the VTK geometry.
"""
function visualize_geometry_windcraft(; save_path   = extdrive_path*"windcraft_geometry00/",
                                        run_name    = "windcraft",
                                        prompt      = true,
                                        verbose     = true,
                                        v_lvl       = 0,
                                        paraview    = true,
                                        xfoil       = false,
                                        optargs...)

    gt.verbalize("GEOMETRY VISUALIZATION", v_lvl, verbose)

    # Generate the Geometry in order to visualize
    system, rotors = generate_geometry_windcraft(; xfoil=xfoil,
                                                  data_path=uns.def_data_path,
                                                  run_name=run_name,
                                                  v_lvl=v_lvl+1,
                                                  verbose=verbose, optargs...)

    # Set up dummy values of RPM and freestream
    Vinf(x,t) = [1,0,0]
    vlm.setVinf(system, Vinf)
    for rotor in rotors
        vlm.setRPM(rotor, 6000)
    end

    # Save visualization of geometry
    gt.create_path(save_path, prompt)
    strn = vlm.save(system, run_name; save_horseshoes=false, path=save_path)

    # Call Paraview
    if paraview
        run(`paraview --data="$save_path/$strn"`)
    end

end
