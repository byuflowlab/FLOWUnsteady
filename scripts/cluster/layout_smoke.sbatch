#!/bin/bash
#SBATCH --job-name=flow-mpi-layout
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=4
#SBATCH --cpus-per-task=4
#SBATCH --exclusive
#SBATCH --time=00:05:00
set -euo pipefail
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="/opt/slurm/bin:$PATH"
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
mapfile -t hosts < <(/opt/slurm/bin/scontrol show hostnames "$SLURM_JOB_NODELIST")
host_list=$(printf '%s:8,' "${hosts[@]}")
host_list=${host_list%,}
for ranks_per_node in 1 2 4; do
    threads=$((8 / ranks_per_node))
    printf 'LAYOUT RANKS_PER_NODE=%s THREADS=%s\n' "$ranks_per_node" "$threads"
    mpirun --host "$host_list" -np "$((3 * ranks_per_node))" \
        --map-by "ppr:$ranks_per_node:node:PE=$threads" --bind-to core --report-bindings \
        bash -c 'printf "rank=%s host=%s " "$OMPI_COMM_WORLD_RANK" "$(hostname)"; sed -n "/^Cpus_allowed_list:/p" /proc/self/status'
done
