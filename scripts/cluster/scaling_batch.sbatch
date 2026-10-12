#!/bin/bash
#SBATCH --job-name=flow-mpi-scaling-batch
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=4
#SBATCH --cpus-per-task=4
#SBATCH --exclusive
#SBATCH --time=01:00:00
set -euo pipefail
task_root=${1:?Pass the shared task directory}
repeats=${2:-3}
threads=${3:-8}
ranks_per_node=${4:-1}
partitions=${5:-both}
schedulers=${6:-static}
callback_sync=${7:-full}
rotor_parallel=${8:-false}
preflight=${9:-true}
blade_elements=${10:-8}
pin_threads=${11:-false}
source_order=${12:-contiguous}
[[ $repeats =~ ^[1-9][0-9]*$ ]] || exit 2
[[ $threads =~ ^(1|2|4|8)$ ]] || exit 2
[[ $ranks_per_node =~ ^(1|2|4)$ ]] || exit 2
(( threads * ranks_per_node == 8 )) || { echo 'Use all eight physical cores per MPI node'; exit 2; }
[[ $partitions == both || $partitions == all || $partitions == targets || $partitions == sources || $partitions == hybrid ]] || exit 2
[[ $schedulers == both || $schedulers == static || $schedulers == dynamic ]] || exit 2
[[ $callback_sync == full || $callback_sync == readonly ]] || exit 2
[[ $rotor_parallel == true || $rotor_parallel == false ]] || exit 2
[[ $preflight == true || $preflight == false ]] || exit 2
[[ $blade_elements =~ ^[0-9]+$ ]] && (( blade_elements >= 2 )) || exit 2
[[ $pin_threads == true || $pin_threads == false ]] || exit 2
[[ $source_order == contiguous || $source_order == cyclic ]] || exit 2
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot" JULIA_NUM_THREADS=$threads
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
cd "$task_root/code"
mapfile -t hosts < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
host_list=$(printf '%s:8,' "${hosts[@]}")
host_list=${host_list%,}
ranks=$((3 * ranks_per_node))
output="$task_root/results/scaling-batch-${SLURM_JOB_ID}-t${threads}-rpn${ranks_per_node}"
printf 'JOB %s HOSTS %s REPEATS %s THREADS %s RANKS_PER_NODE %s PARTITIONS %s SCHEDULERS %s CALLBACK_SYNC %s\n' \
    "$SLURM_JOB_ID" "${hosts[*]}" "$repeats" "$threads" "$ranks_per_node" "$partitions" "$schedulers" "$callback_sync"
# Processes persist through all layouts; each active communicator still runs a
# complete warmup and then constructs a fresh measured simulation. Lifetime RSS
# includes previous layouts. Recheck promising timings with scaling.sbatch.
if [[ $preflight == true ]]; then
JULIA_NUM_THREADS=1 mpirun --host "$host_list" -np 3 \
    --map-by ppr:1:node:PE=1 --bind-to core \
    julia --startup-file=no --project=environments/mpi test/mpi/nearfield_scheduler.jl
JULIA_NUM_THREADS=1 mpirun --host "$host_list" -np 3 \
    --map-by ppr:1:node:PE=1 --bind-to core \
    julia --startup-file=no --project=environments/mpi test/mpi/callback_readonly.jl --skip-pilot
if [[ $partitions == hybrid || $partitions == all ]]; then
    JULIA_NUM_THREADS=1 mpirun --host "$host_list" -np 6 \
        --map-by ppr:2:node:PE=1 --bind-to core \
        julia --startup-file=no --project=environments/mpi test/mpi/hybrid.jl
fi
if [[ $rotor_parallel == true ]]; then
    JULIA_NUM_THREADS=1 mpirun --host "$host_list" -np 3 \
        --map-by ppr:1:node:PE=1 --bind-to core \
        julia --startup-file=no --project=environments/mpi test/mpi/rotor_parallel.jl
fi
fi
mpirun --host "$host_list" -np "$ranks" \
    --map-by "ppr:$ranks_per_node:node:PE=$threads" --bind-to core \
    julia --startup-file=no --project=environments/mpi scripts/benchmark_scaling.jl \
    --repeats "$repeats" --partitions "$partitions" --schedulers "$schedulers" \
    --ranks-per-node "$ranks_per_node" --n "$blade_elements" --revolutions 4 \
    --steps-per-rev 36 --p-per-step 2 --check-state false \
    --callback-sync "$callback_sync" --rotor-parallel "$rotor_parallel" \
    --pin-threads "$pin_threads" \
    --source-order "$source_order" \
    --gc-policy deferred --output "$output"
