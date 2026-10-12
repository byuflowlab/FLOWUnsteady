#!/bin/bash
#SBATCH --job-name=flow-mpi-validate
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=16
#SBATCH --exclusive
#SBATCH --time=00:45:00
set -euo pipefail
task_root=${1:?Pass the shared task directory}
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot" JULIA_NUM_THREADS=8
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
# These c6a nodes have Ethernet. Explicit TCP transport makes the comparison
# reproducible and avoids confusing an installed EFA stack with an EFA device.
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
cd "$task_root/code"
mapfile -t hosts < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
printf 'JOB %s HOSTS %s\n' "$SLURM_JOB_ID" "${hosts[*]}"
srun --ntasks=3 --ntasks-per-node=1 --cpu-bind=none bash -c '
    hostname
    lscpu | grep -E "Model name|Socket|Core.s. per socket|Thread.s. per core|CPU.s.:"
    grep Cpus_allowed_list /proc/self/status
    ls /dev/infiniband 2>/dev/null || true'
mpirun -np 3 --map-by ppr:1:node:PE=8 --bind-to core --report-bindings \
    julia --startup-file=no --project=environments/mpi scripts/cluster/mpi_smoke.jl
# Run the original automatic-GC regressions on one thread; the complete hover
# comparisons below exercise eight threads/rank with explicit deferred GC.
JULIA_NUM_THREADS=1 mpirun -np 3 --map-by ppr:1:node:PE=1 --bind-to core \
    julia --startup-file=no --project=environments/mpi test/mpi/runtests.jl
for nodes in 1 2 3; do
    host_list=$(IFS=,; echo "${hosts[*]:0:nodes}")
    for mode in serial mpi; do
        [[ $mode == serial && $nodes != 1 ]] && continue
        output="$task_root/results/check-${SLURM_JOB_ID}-${mode}-${nodes}node"
        mpirun --host "$host_list" -np "$nodes" --map-by ppr:1:node:PE=8 --bind-to core \
            julia --startup-file=no --project=environments/mpi examples/mpi/rotorhover.jl \
            --mode "$mode" --n 8 --revolutions 4 --steps-per-rev 36 --p-per-step 2 \
            --check-state true --gc-policy deferred --output "$output"
    done
done
python3 scripts/compare_mpi_rotorhover.py \
    "$task_root/results/check-${SLURM_JOB_ID}-serial-1node" \
    "$task_root/results/check-${SLURM_JOB_ID}-mpi-1node" \
    "$task_root/results/check-${SLURM_JOB_ID}-mpi-2node" \
    "$task_root/results/check-${SLURM_JOB_ID}-mpi-3node" \
    --output "$task_root/results/check-${SLURM_JOB_ID}-comparison.json"
