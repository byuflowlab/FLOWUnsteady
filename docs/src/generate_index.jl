import FLOWUnsteady as uns

module_path = splitdir(@__FILE__)[1]      # Path to this module
output_path = module_path                 # Path where to store markdown outputs

readmefile = joinpath(uns.examples_path, "..", "README.md")

htmlflags = [ # Trigger on, trigger off, on/off status
                "<p" "</p" false;
                "<img" ">" false;
                # "<div" "</div>" false;
            ]

youtubeflag = "href=\"https://www.youtube.com/watch?v="

str_youtube(url) = """
    ```@raw html
    <div style="position:relative;padding-top:50%;">
        <iframe style="position:absolute;left:0;top:0;height:80%;width:72%;"
            src="https://www.youtube.com/embed/$(url)?hd=1"
            title="YouTube video player" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowfullscreen></iframe>
    </div>
    ```
    """


open(joinpath(output_path, "index.md"), "w") do fout

    open(readmefile, "r") do fin

        for l in eachline(fin)

            for (i, triggeron) in enumerate(htmlflags[:, 1])
                if htmlflags[i, 3]==false && contains(l, triggeron) &&
                    sum(htmlflags[:, 3])==0 && !contains(l, youtubeflag)

                    println(fout, "```@raw html")
                    htmlflags[i, 3] = true

                end
            end

            if contains(l, youtubeflag)

                url = l[ ( last(findfirst(youtubeflag, l))+1 ):( first(findfirst("&hd=1", l))-1 ) ]
                println(fout, str_youtube(url))
            else
                println(fout, l)
            end

            for (i, triggeroff) in enumerate(htmlflags[:, 2])
                if htmlflags[i, 3]==true && contains(l, triggeroff)

                    println(fout, "```")
                    htmlflags[i, 3] = false

                end
            end

        end
    end

end
