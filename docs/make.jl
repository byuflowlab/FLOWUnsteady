using Documenter, FLOWUnsteady

makedocs(
            root    = ".",
            source  = "./src",
            build   = "./build",
            authors = "Eduardo J. Alvarez, Ryan Anderson, Adam Cardoza, Tyler Critchfield, Judd Mehr",
            sitename="FLOWUnsteady",
            pages = ["Home" => "index.md",
                    "Theory" => "theory.md",
                    "Tutorials" => "tutorials.md",
                    "How-to Guides" => "howto.md"
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