#!/bin/bash
#SBATCH --job-name=flow-mpi-scaling
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=4
#SBATCH --cpus-per-task=4
#SBATCH --exclusive
#SBATCH --time=01:00:00
set -euo pipefail
task_root=${1:?Pass the shared task directory}
repeats=${2:-1}
threads=${3:-8}
ranks_per_node=${4:-1}
partitions=${5:-all}
nearfield=${6:-static}
callback_sync=${7:-full}
rotor_parallel=${8:-false}
blade_elements=${9:-8}
pin_threads=${10:-false}
source_order=${11:-contiguous}
[[ $repeats =~ ^[1-9][0-9]*$ ]] || exit 2
[[ $threads =~ ^(1|2|4|8)$ ]] || exit 2
[[ $ranks_per_node =~ ^(1|2|4)$ ]] || exit 2
(( threads * ranks_per_node == 8 )) || { echo 'Use all eight physical cores per MPI node'; exit 2; }
[[ $partitions == all || $partitions == targets || $partitions == sources || $partitions == hybrid ]] || exit 2
[[ $nearfield == static || $nearfield == dynamic ]] || exit 2
[[ $callback_sync == full || $callback_sync == readonly ]] || exit 2
[[ $rotor_parallel == true || $rotor_parallel == false ]] || exit 2
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
printf 'JOB %s HOSTS %s REPEATS %s THREADS %s RANKS_PER_NODE %s\n' \
    "$SLURM_JOB_ID" "${hosts[*]}" "$repeats" "$threads" "$ranks_per_node"
declare -a outputs=()
for ((repeat=1; repeat<=repeats; repeat++)); do
    if (( repeat % 2 == 1 )); then
        layouts=(serial-targets-1 mpi-sources-3 mpi-targets-3 mpi-hybrid-3 mpi-sources-1 mpi-targets-1 mpi-hybrid-1 mpi-sources-2 mpi-targets-2 mpi-hybrid-2)
    else
        layouts=(serial-targets-1 mpi-hybrid-2 mpi-targets-2 mpi-sources-2 mpi-hybrid-1 mpi-targets-1 mpi-sources-1 mpi-hybrid-3 mpi-targets-3 mpi-sources-3)
    fi
    for layout in "${layouts[@]}"; do
        IFS=- read -r mode partition nodes <<< "$layout"
        [[ $mode == serial || $partitions == all || $partition == "$partitions" ]] || continue
        run_threads=$threads
        ranks=$((nodes * ranks_per_node))
        if [[ $mode == serial ]]; then run_threads=8; ranks=1; fi
        host_list=$(printf '%s:8,' "${hosts[@]:0:nodes}")
        host_list=${host_list%,}
        output="$task_root/results/scaling-${SLURM_JOB_ID}-${layout}-${nearfield}-${callback_sync}-t${run_threads}-r${ranks}-rep${repeat}"
        target_groups=1
        [[ $partition != hybrid ]] || target_groups=$ranks_per_node
        run_source_order=$source_order
        [[ $mode != serial && $partition != targets ]] || run_source_order=contiguous
        outputs+=("$output")
        JULIA_NUM_THREADS=$run_threads mpirun --host "$host_list" -np "$ranks" \
            --map-by "ppr:$((ranks / nodes)):node:PE=$run_threads" --bind-to core \
            julia --startup-file=no --project=environments/mpi examples/mpi/rotorhover.jl \
            --mode "$mode" --partition "$partition" --nearfield "$nearfield" \
            --target-groups "$target_groups" \
            --pin-threads "$pin_threads" \
            --source-order "$run_source_order" \
            --callback-sync "$callback_sync" --rotor-parallel "$rotor_parallel" --n "$blade_elements" --revolutions 4 \
            --steps-per-rev 36 --p-per-step 2 --check-state false \
            --gc-policy deferred --output "$output"
    done
done
python3 scripts/compare_mpi_rotorhover.py "${outputs[0]}" "${outputs[@]:1}" \
    --output "$task_root/results/scaling-${SLURM_JOB_ID}-comparison.json"
