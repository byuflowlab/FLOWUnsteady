#!/bin/bash
# Separate sampled diagnostics; the ordinary benchmark remains uninstrumented.
# Submit: sbatch scripts/cluster/profile.sbatch /home/ubuntu/flowunsteady-mpi-20261013
#SBATCH --job-name=flow-mpi-profile
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=16
#SBATCH --exclusive
#SBATCH --time=00:30:00
set -euo pipefail
task_root=${1:?Pass the shared task directory}
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot" JULIA_NUM_THREADS=8
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
cd "$task_root/code"
mapfile -t hosts < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
printf 'PROFILE JOB %s HOSTS %s\n' "$SLURM_JOB_ID" "${hosts[*]}"
declare -a outputs=()
for layout in serial-1 mpi-3 mpi-1; do
    mode=${layout%-*}
    nodes=${layout##*-}
    host_list=$(IFS=,; echo "${hosts[*]:0:nodes}")
    output="$task_root/results/profile-${SLURM_JOB_ID}-${layout}"
    outputs+=("$output")
    mpirun --host "$host_list" -np "$nodes" --map-by ppr:1:node:PE=8 --bind-to core \
        julia --startup-file=no --project=environments/mpi scripts/profile_mpi.jl \
        --mode "$mode" --n 8 --revolutions 4 --steps-per-rev 36 --p-per-step 2 \
        --check-state false --gc-policy deferred --profile-delay 0.003 \
        --profile-buffer 2000000 --output "$output"
done
python3 scripts/compare_mpi_rotorhover.py "${outputs[0]}" "${outputs[@]:1}" \
    --output "$task_root/results/profile-${SLURM_JOB_ID}-comparison.json"
