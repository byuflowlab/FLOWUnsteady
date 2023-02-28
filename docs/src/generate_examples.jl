import FLOWUnsteady as uns

module_path = splitdir(@__FILE__)[1]      # Path to this module
output_path = joinpath(module_path, "examples") # Path where to store  examples as markdown

include("generate_examples_wing.jl")
include("generate_examples_tethered.jl")
