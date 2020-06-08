using Documenter, FLOWUnsteady

makedocs(
            root    = ".",
            source  = "./src",
            build   = "./build",
            authors = "Eduardo J. Alvarez, Ryan Anderson, Adam Cardoza, Tyler Critchfield, Judd Mehr",
            sitename="FLOWUnsteady",
            pages = ["Home" => "index.md",
                    "Theory" => "ref/theory.md",
                    "ref/FLOWVLM.md",
                    "ref/VPM.md",
                    "ref/FLOWUnsteady.md",
                    "Tutorials" => "tutorial/tutorials.md",
                    "tutorial/geometry-basics.md",
                    "How-to Guide" => "howto/how-to.md",
                    "howto/getting-started.md",
                    "howto/paraview-visualization.md",
                    "howto/defining-systems.md",
                    "howto/define-a-rotor.md",
                    "howto/define-kinematic-maneuvers.md",
                    "howto/select-a-solver.md",
                    "howto/calculate-aerodynamic-forces.md",
                    "howto/set-up-runtime-functions.md",
                    "howto/set-up-monitors.md",
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