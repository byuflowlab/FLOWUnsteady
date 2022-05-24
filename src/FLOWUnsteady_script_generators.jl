# script generator for use with the BYU supercomputer

function write_script_example(script_path, example_file, call_example_functions...)
    open(script_path, "w") do io
        write(io, "#"^50, "\n")
        write(io, "# AUTOMATED SCRIPT GENERATED ON $(Dates.now()) BY FLOWUnsteady", "\n")
        write(io, "# Script Generator by Ryan Anderson", "\n")
        write(io, "#"^50, "\n")
        write(io, "", "\n")

        # include example file
        write(io, "include(\"$(joinpath(module_path, "..", "examples", example_file))\")", "\n", "\n")

        # call specified function
        if typeof(call_example_functions) <: String
            write(io, call_example_functions, "\n")
        elseif eltype(call_example_functions) <: String
            for call_example_function in call_example_functions
                write(io, call_example_function, "\n")
            end
        else
            @error "call_example_functions are the wrong type! Got $call_example_functions"
        end
    end
end

function write_batch(batch_path, file_names::Vector{String}, FLOWExaFMM_dir::String;
    test=true, email_notifications=false, email_address="",
    time="01:00:00", ntasks=24, mem_per_cpu="1024M"
)

    open(batch_path, "w") do io
        write(io, "#!/bin/bash", "\n")
        write(io, "\n")
        write(io, "#SBATCH --time=$time   # walltime", "\n")
        write(io, "#SBATCH --ntasks=$ntasks   # number of processor cores (i.e. tasks)", "\n")
        write(io, "#SBATCH --nodes=1   # number of nodes", "\n")
        write(io, "#SBATCH --mem-per-cpu=$mem_per_cpu   # memory per CPU core", "\n")
        write(io, "#SBATCH -J \"$batch_path\"   # job name", "\n")
        if email_notifications
            write(io, "#SBATCH --mail-user=", email_address, "\n")
            write(io, "#SBATCH --mail-type=BEGIN", "\n")
            write(io, "#SBATCH --mail-type=END", "\n")
            write(io, "#SBATCH --mail-type=FAIL", "\n")
        end
        if test; write(io, "#SBATCH --qos=test"); end
        write(io, "\n")
        write(io, "\n")
        write(io, "# Set the max number of threads to use for programs using OpenMP. Should be <= ppn. Does nothing if the program doesn't use OpenMP.", "\n")
        write(io, "export OMP_NUM_THREADS=\$SLURM_CPUS_ON_NODE", "\n")
        write(io, "\n")

        write(io, "# LOAD MODULES, INSERT CODE, AND RUN YOUR PROGRAMS HERE\n")
        write(io, "module load julia/1.6\n")
        write(io, "\n")
        write(io, "# build fmm\n")
        build_tmp_path = joinpath(FLOWExaFMM_dir, "build_tmp.sh\n")
        write(io, "source $build_tmp_path\n")
        write(io, "\n")

        write(io, "# include julia files\n")
        for file_name in file_names
            write(io, "julia -t auto $file_name\n")
        end

        write(io, "\n")
    end

    return nothing
end
