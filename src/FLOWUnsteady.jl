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
import FLOWMath
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

# import DelimitedFiles as DF
# import ForwardDiff as FD
# import LinearAlgebra as LA
# import FLOWTrajectories as FT

# Aliases
const gt    = GeometricTools
const vpm   = FLOWVPM
const vlm   = FLOWVLM
const vsp   = VSPGeom
const noise = FLOWNoise
const fm    = FLOWMath

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






# plots
# data = readdlm("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/output_file.txt", ' ')
# dt = 0.0006666666666666666
# dt = 0.0009092559960466243
# #dt = 0.0015715833880154161
# dt = 0.0017453292519943294
# ts = range(0.0, dt*length(data[:,1]), length = length(data[:,1]))
# dx_data = data[:,1:3]
# x_data = data[:,4:10]
# u_data = data[:,11:13]
# dx_filter = data[:,14:16]
# x_filter = data[:,17:end]
# p1 = plot(ts, x_data[:,1], title = "Velocity (m/s)", label = "Actual")
# # plot!(ts_ref, vs_ref, label = "Commanded")
# p2 = plot(ts, x_data[:,6], title = "Pitch (deg)", label = "Actual")
# # plot!(ts_ref, ps_ref, label = "Commanded")
# p3 = plot(ts, u_data[:,1], title = "Front Lifters (rad/s)", label = false, ylims = [0.0, 400.0])
# p4 = plot(ts, u_data[:,2], title = "Back Lifters (rad/s)", label = false, ylims = [0.0, 400.0])
# p5 = plot(ts, u_data[:,3], xlabel = "Time (s)", title = "Pusher (rad/s)", label = false, ylims = [0.0, 500.0])

# # p5 = plot(ts, dx_data[:,3], xlabel = "Time (s)", ylabel = "")
# # p6 = plot(ts, x_data[:,6], xlabel = "Time (s)")
# p_x = plot(p1,p2, layout = (2,1))
# p_u = plot(p3,p4,p5, layout = (3,1))
# p_all = plot(p1,p2,p3,p4,p5)


# pu = plot(ts, u_data[:,1], xlabel = "Time (s)", title = "Rotor Velocity (rad/s)", label = "Front Lifters", ylims = [0.0, 500.0], legend = :topleft)
# plot!(ts, u_data[:,2], label = "Back Lifters")
# plot!(ts, u_data[:,3], label = "Pusher")

# px = plot(p1,p2)

# p_final = plot(px, pu, layout = (2,1))

# # window_dist = 1000*ones(Int64, length(data[:,1]))
# window_dist[1] = 100
# window_dist[5300:6000] .= 50



# window = 400
# half_window = Int(window/2)
# u_data = zeros(3,window)
# for i in 1:length(u[:,1])
#     for j in 1:length(u_data[1,:])-1
#         u_data[:,j] = u_data[:,j+1]
#     end
#     u_data[:,end] = u[i,:]
#     if i < window
#         u_filter[i,:] = u[i,:]
#     else
#         for j in 1:length(u_filter[1,:])
#             u_filter[i-half_window,j] = sum(u_data[j,1:window])/window
#         end
#     end
# end

# for i in 1:half_window
#     for j in 1:length(u_filter[1,:])
#         u_filter[end-half_window+i,j] = sum(u_data[j,end-window+i:end])/(window-i)
#     end
# end

# # plot!(u_filter[:,1])

# for i in 1:length(u_filter[:,1])
#     f = open("/media/flowlab/ExternalStorage1/atagg/workstation/FLOWTrajectories/examples/aurora/ctrl_inputs.txt", "a")
#     print(f, u_filter[i,1], " ", u_filter[i,2], " ", u_filter[i,3], "\n")
#     close(f)
# end
