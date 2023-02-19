using Documenter
using FLOWUnsteady
const uns = FLOWUnsteady

# include("src/generate_examples.jl")

makedocs(
    sitename = "FLOWUnsteady",
    format = Documenter.HTML(;
                                sidebar_sitename = false,
                                assets = ["assets/favicon.ico"],
                            ),
    pages = [
                "Home"              => "index.md",
                "Examples"          => [
                                        "Simple Wing" => [
                                                            "Basics" => "examples/sweptwing-4p2aoa.md",
                                                            "examples/sweptwing-aoasweep.md"
                                                            ],
                                       ],
                # "Potential Flow"    => "potentialflow.md",
                # "Elements"          => [
                #                         "elements/paneldefinition.md",
                #                         "Constant Source"  => "elements/constantsource.md",
                #                         "Constant Doublet" => "elements/constantdoublet.md",
                #                         "Semi-Infinite Doublet" => "elements/semiinfdoublet.md",
                #                         "Non-Planar Semi-Infinite Doublet" => "elements/semiinfnonplanardoublet.md",
                #                         "Constant Vortex Sheet" => "elements/constantvortexsheet.md",
                #                        ],
                # "Geometry Engine"   => [
                #                         "Grid Generation" => [
                #                                                 "geometry/gridgeneration.md",
                #                                                 "geometry/gridgeneration-loft.md",
                #                                                 "geometry/gridgeneration-rev.md",
                #                                                 "geometry/gridgeneration-transf.md",
                #                                                 "geometry/gridgeneration-triang.md",
                #                                             ]
                #                         "Advanced" => [
                #                                                 "geometry/basics.md",
                #                                                 "geometry/basics-grid.md",
                #                                                 "geometry/basics-transformations.md",
                #                                                 "geometry/basics-loopedgrid.md",
                #                                                 "geometry/basics-surfacegrid.md",
                #                                             ]
                #                       ],
                # "API Reference"     => ["api.md",
                #                         "api-elements.md",
                #                         "api-abstractbody.md"
                #                        ]
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
