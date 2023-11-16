#=##############################################################################
# DESCRIPTION
    Import of OpenVSP geometry into FLOWUnsteady

# AUTHORSHIP
  * Author          : Cibin Joseph
  * Email           : cibinjoseph92@gmail.com
  * Created         : Aug 2023
  * Last updated    : Aug 2023
  * License         : MIT
=###############################################################################

import FLOWUnsteady as uns

run_name        = "aircraft-vsp"            # Name of this simulation
save_path       = run_name                  # Where to save this simulation


Vinf(X, t)      = [1.0, 0.0, 0.0]  # Freestream function

# ----------------- 1) VEHICLE DEFINITION --------------------------------------
println("Importing geometry...")

comp = uns.read_degengeom("aircraft.csv")

fuselage = uns.import_vsp(comp[1])
wingL = uns.import_vsp(comp[2])
wingR = uns.import_vsp(comp[2]; flip_y=true)
basefuse = uns.import_vsp(comp[4])
horstabL = uns.import_vsp(comp[5])
horstabR = uns.import_vsp(comp[5]; flip_y=true)
verstab = uns.import_vsp(comp[7])

println("Generating vehicle...")

# Generate vehicle
system = uns.vlm.WingSystem()                   # System of all FLOWVLM objects
uns.vlm.addwing(system, "WingL", wingL)
uns.vlm.addwing(system, "WingR", wingR)
uns.vlm.addwing(system, "HorStabL", horstabL)
uns.vlm.addwing(system, "HorStabR", horstabR)
uns.vlm.addwing(system, "VerStab", verstab)

fuse_grid = uns.gt.MultiGrid(3)
uns.gt.addgrid(fuse_grid, "Fuselage", fuselage)

basefuse_grid = uns.gt.MultiGrid(3)
uns.gt.addgrid(basefuse_grid, "BaseFuse", basefuse)

grids = [fuse_grid, basefuse_grid]

vlm_system = system                         # System solved through VLM solver
wake_system = system                        # System that will shed a VPM wake

vehicle = uns.VLMVehicle(   system;
                            vlm_system=vlm_system,
                            wake_system=wake_system,
                            grids=grids
                         )

# ----------------- 2) GEOMETRY EXPORT -----------------------------------------
rm(save_path, recursive=true, force=true)
mkdir(save_path)

uns.vlm.setVinf(system, Vinf)
uns.save_vtk(vehicle, run_name; path=save_path)
