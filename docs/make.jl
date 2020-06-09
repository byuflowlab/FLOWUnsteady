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
                "tutorials/first-steps.md",],
            "How-to-Guide" => Any[
                "how-to-guide/getting-started.md",
                "how-to-guide/paraview-visualization.md",
                "how-to-guide/defining-systems.md",
                "how-to-guide/define-a-rotor.md",
                "how-to-guide/define-kinematic-maneuvers.md",
                "how-to-guide/select-a-solver.md",
                "how-to-guide/calculate-aerodynamic-forces.md",
                "how-to-guide/set-up-runtime-functions.md",
                "how-to-guide/set-up-monitors.md",],
            ]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
deploydocs(
    repo   = "github.com/byuflowlab/FLOWUnsteady",
)
