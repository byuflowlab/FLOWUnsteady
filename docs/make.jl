using Documenter
# using FLOWUnsteady

makedocs(
    sitename = "FLOWUnsteady",
    format = Documenter.HTML(),
    # modules = [FLOWUnsteady],
    pages = ["Home" => "index.md",
            "Tutorials" => Any[
                "tutorials/getting-started.md",
                "tutorials/first-steps.md",
                ],
            "How to " => Any[
                "how-to-guide/paraview-visualization.md",
                "how-to-guide/define-complex-wings.md",
                "how-to-guide/define-complex-systems.md",
                "how-to-guide/define-a-rotor.md",
                "how-to-guide/define-kinematic-maneuvers.md",
                "how-to-guide/set-up-runtime-functions.md",
                "how-to-guide/select-a-solver.md",
                "how-to-guide/wake-stability.md",],
            "Reference" => Any[
                "reference/FLOWVLMfunctions.md",
                "reference/VPMfunctions.md",
                "reference/FLOWUnsteadyfunctions.md",],

            "Theory" => Any[
                "theory/FLOWVLMtheory.md",
                "theory/VPMtheory.md",
                "theory/FLOWUnsteadytheory.md",],
            ]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
deploydocs(
    repo   = "github.com/byuflowlab/FLOWUnsteady",
)
