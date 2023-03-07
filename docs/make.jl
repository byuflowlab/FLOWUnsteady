using Documenter
using DocumenterTools: Themes

using FLOWUnsteady
const uns = FLOWUnsteady

import FLOWUnsteady: FLOWVPM, FLOWVLM

include("src/generate_examples.jl")


# Themes.compile(joinpath(@__DIR__,"src/assets/light.scss"), joinpath(@__DIR__,"src/assets/themes/documenter-light.css"))
# Themes.compile(joinpath(@__DIR__,"src/assets/dark.scss"), joinpath(@__DIR__,"src/assets/themes/documenter-dark.css"))


makedocs(
    sitename = "FLOWUnsteady",
    format = Documenter.HTML(;
                                sidebar_sitename = false,
                                assets = ["assets/favicon.ico"],
                                collapselevel = 1
                            ),
    pages = [
                "Home"          => "index.md",
                "Installation"  => "installation.md",
                "Tutorials"     => [
                                    "Simple Wing" => [
                                                        "Basics" => "examples/wing-4p2aoa.md",
                                                        "examples/wing-aoasweep.md"
                                                      ],
                                    "examples/tetheredwing.md",
                                    "examples/heavingwing.md",
                                   ],
                "API Guide"     => [
                                    "(1) Vehicle Definition" => [
                                                                "api/flowunsteady-vehicle-types.md",
                                                                "api/flowunsteady-vehicle-components.md",
                                                                "api/flowunsteady-vehicle-asm.md"
                                                                ],
                                    "api/flowunsteady-maneuver.md",
                                    "api/flowunsteady-simulation.md",
                                    "api/flowunsteady-monitor.md",
                                    "api/flowunsteady-run.md",
                                    "(6) Viz and Postprocessing" => [
                                                                "api/flowunsteady-postprocessing-fdom.md",
                                                                "api/flowunsteady-postprocessing-noise.md",
                                                                ],
                                    "Extras" => [
                                                "FLOWVPM"  => [
                                                                "api/flowvpm-particle.md",
                                                                "api/flowvpm-uj.md",
                                                                "api/flowvpm-viscous.md",
                                                                "api/flowvpm-relaxation.md",
                                                                "api/flowvpm-sfs.md",
                                                                "api/flowvpm-time.md",
                                                                "api/flowvpm-utils.md",
                                                               ]
                                                ]
                                   ],
            ]
)



# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
deploydocs(
    repo = "github.com/byuflowlab/FLOWUnsteady.git",
    target = "build",
    deps = nothing,
    make = nothing,
    # devbranch = "main"
)
