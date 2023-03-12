import FLOWUnsteady as uns

module_path = splitdir(@__FILE__)[1]      # Path to this module
output_path = module_path                 # Path where to store markdown outputs

readmefile = joinpath(uns.examples_path, "..", "README.md")

htmlflags = [ # Trigger on, trigger off, on/off status
                "<p" "</p" false;
                "<img" ">" false;
                "<div" "</div>" false;
            ]


open(joinpath(output_path, "index.md"), "w") do fout

    open(readmefile, "r") do fin

        for l in eachline(fin)

            for (i, triggeron) in enumerate(htmlflags[:, 1])
                if htmlflags[i, 3]==false && contains(l, triggeron) && sum(htmlflags[:, 3])==0

                    println(fout, "```@raw html")
                    htmlflags[i, 3] = true

                end
            end

            println(fout, l)

            for (i, triggeroff) in enumerate(htmlflags[:, 2])
                if htmlflags[i, 3]==true && contains(l, triggeroff)

                    println(fout, "```")
                    htmlflags[i, 3] = false

                end
            end

        end
    end

end
