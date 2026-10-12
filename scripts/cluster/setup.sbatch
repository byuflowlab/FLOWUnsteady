#!/bin/bash
#SBATCH --job-name=flow-mpi-setup
#SBATCH --partition=compute
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=16
#SBATCH --exclusive
#SBATCH --time=00:45:00
set -euo pipefail
task_root=${1:?Pass the shared task directory}
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
hostname
lscpu | grep -E 'Model name|Socket|Core.s. per socket|Thread.s. per core|CPU.s.:'
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot"
export JULIA_NUM_THREADS=8 JULIA_NUM_PRECOMPILE_TASKS=8
export JULIA_PKG_PRECOMPILE_AUTO=0 CONDA_JL_HOME="$task_root/depot/conda/3"
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPAL_PREFIX=/opt/amazon/openmpi5
cd "$task_root/code"
"$task_root/runtime/uv" venv --allow-existing --python /usr/bin/python3 environments/mpi/.venv
"$task_root/runtime/uv" pip install --python environments/mpi/.venv/bin/python \
    -r environments/mpi/requirements.txt
export PYTHON="$PWD/environments/mpi/.venv/bin/python"
# MPIPreferences eagerly loads an existing system MPI on import. Clear just
# that task-local selection before switching installations in a fresh process.
julia --startup-file=no -e '
    using TOML
    file = "environments/mpi/LocalPreferences.toml"
    if isfile(file)
        preferences = TOML.parsefile(file)
        delete!(preferences, "MPIPreferences")
        open(io -> TOML.print(io, preferences), file, "w")
    end'
julia --startup-file=no --project=environments/mpi -e '
    using MPIPreferences
    prefix = "/opt/amazon/openmpi5"
    MPIPreferences.use_system_binary(library_names=[joinpath(prefix, "lib", "libmpi.so")],
        mpiexec=joinpath(prefix, "bin", "mpiexec"), abi="OpenMPI")
    # HDF5_jll also imports OpenMPI_jll. Keep all of its library products on
    # the same system installation to avoid mixing Open MPI 4/5 or two runtimes.
    MPIPreferences.Preferences.set_preferences!(
        (Base.UUID("fe0851c0-eecd-5654-98d4-656369965a5c"), "OpenMPI_jll"),
        (name * "_path" => joinpath(prefix, "lib", name * ".so")
            for name in ("libmpi", "libmpi_mpifh", "libmpi_usempi_ignore_tkr", "libmpi_usempif08"))...,
        "mpiexec_path" => joinpath(prefix, "bin", "mpiexec"); force=true)'
# Copied package downloads can contain generated paths from the source machine.
# Rebuild both Python bridges before precompiling at the cluster path.
julia --startup-file=no --project=environments/mpi -e 'using Pkg; Pkg.build("Conda")'
julia --startup-file=no scripts/setup_mpi.jl
julia --startup-file=no --project=environments/mpi -e 'using Pkg; Pkg.precompile()'
julia --startup-file=no --project=environments/mpi -e '
    import FLOWUnsteady as uns
    println("Julia ", VERSION)
    println("MPI ", uns.MPI.Get_library_version())
    println("FLOWVPM ", pathof(uns.vpm))
    println("FastMultipole ", pathof(uns.vpm.fmm))'
