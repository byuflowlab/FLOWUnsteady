using Documenter
# using FLOWUnsteady

makedocs(
    sitename = "FLOWUnsteady",
    format = Documenter.HTML(),
    # modules = [FLOWUnsteady],
    pages = ["Home" => "index.md",
            "Reference" => Any[
                "reference/FLOWVLM.md",
                "reference/VPM.md",
                "reference/FLOWUnsteady.md",],
            "Tutorials" => Any[
                "tutorials/getting-started.md",
                "tutorials/first-steps.md",
                "tutorials/define-complex-wings.md",
                "tutorials/define-complex-systems.md",
                "tutorials/define-a-rotor.md",
                "tutorials/define-kinematic-maneuvers.md",
                "tutorials/set-up-runtime-functions.md",],
            "How-to-Guide" => Any[
                "how-to-guide/paraview-visualization.md",
                "how-to-guide/select-a-solver.md",
                "how-to-guide/calculate-aerodynamic-forces.md",
                "how-to-guide/wake-stability.md"],
            ]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
deploydocs(
    repo   = "github.com/byuflowlab/FLOWUnsteady",
)
