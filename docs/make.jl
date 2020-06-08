using Documenter, FLOWUnsteady

makedocs(
            root    = ".",
            source  = "./src",
            build   = "./build",
            authors = "Eduardo J. Alvarez, Ryan Anderson, Adam Cardoza, Tyler Critchfield, Judd Mehr",
            sitename="FLOWUnsteady",
            pages = ["Home" => "index.md",
                    "Reference" => "reference/theory.md",
                        "reference/FLOWVLM.md",
                        "reference/VPM.md",
                        "reference/FLOWUnsteady.md",
                    "Tutorials" => "tutorials/tutorials.md",
                        "tutorials/first-steps.md",
                    "How-to-Guide" => "how-to-guide/how-to.md",
                        "how-to-guide/getting-started.md",
                        "how-to-guide/paraview-visualization.md",
                        "how-to-guide/defining-systems.md",
                        "how-to-guide/define-a-rotor.md",
                        "how-to-guide/define-kinematic-maneuvers.md",
                        "how-to-guide/select-a-solver.md",
                        "how-to-guide/calculate-aerodynamic-forces.md",
                        "how-to-guide/set-up-runtime-functions.md",
                        "how-to-guide/set-up-monitors.md",
                    ]
        )

deploydocs(
            repo   = "github.com/byuflowlab/FLOWUnsteady",
            root   = ".",
            target = "./build",
            branch = "gh-pages",
            # devbranch = "master",
            julia = "0.6"
        )