# Regression: a root-only output failure must terminate the job, not strand peers.
import FLOWUnsteady as uns
uns.MPI.Init()
wing = uns.vlm.simpleWing(2.0, 6.0, 1.0, 0.0, 20.0, 0.0; n=2)
system = uns.vlm.WingSystem()
uns.vlm.addwing(system, "Wing", wing)
vehicle = uns.UVLMVehicle(system; vlm_system=system, wake_system=system)
maneuver = uns.KinematicManeuver((), (), t -> zeros(3), t -> zeros(3))
simulation = uns.Simulation(vehicle, maneuver, 0.0, 0.0, 0.01)
uns.run_simulation(simulation, 1; parallel=uns.ReplicatedMPI(),
    sigma_vlm_surf=0.1, sigma_rotor_surf=0.1, prompt=false, verbose=false,
    save_path="/proc/flowunsteady-mpi-io-failure")
error("Expected an output failure")
