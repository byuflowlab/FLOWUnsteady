#!/usr/bin/env julia
# A separate application environment keeps the solver dependencies reproducible.
using Pkg

root = normpath(joinpath(@__DIR__, ".."))
environment = joinpath(root, "environments", "mpi")
Pkg.activate(environment)

pins = [
    ("AirfoilPrep.jl", "16db0d24f82b041382c70c7cb73ba6f0bd67df52"),
    ("FLOWVLM", "f7c437250f25c8919c30a4088852e19fdfb360d6"),
    ("FLOWNoise", "4e0334c67f6cca570d564a8ba0fe674beb071215"),
    ("BPM.jl", "b1440522d1f93e414d51d36207f9587ea6ed59e4"),
    ("FLOWVPM.jl", "4f433fb09f6baad25db65c9905e0d9cbb09663ce"),
    ("FastMultipole.jl", "21dcc2f2376ab67e0b2030d2f213d47dbae727ad"),
]
if !isfile(joinpath(environment, "Manifest.toml"))
    Pkg.add([PackageSpec(url="https://github.com/byuflowlab/" * name, rev=rev)
             for (name, rev) in pins])
end
cd(environment) do
    Pkg.develop(path="../..")
end
Pkg.instantiate()
if haskey(ENV, "PYTHON")
    Pkg.build("PyCall")
end
