#!/bin/bash
#SBATCH --job-name=flow-n32-baselines
#SBATCH --partition=compute
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=4
#SBATCH --cpus-per-task=4
#SBATCH --exclusive
#SBATCH --time=01:00:00
set -euo pipefail
# Read the existing frozen v6 code. Do not copy, patch or regenerate its files.
task_root=${1:?Pass the frozen v6 task directory}
selection=${2:-all}
reference=${3:-}
existing_four_rank=${4:-}
[[ $selection == all || $selection == alternates ]] || exit 2
[[ $selection != alternates || -n $reference ]] || { echo 'Alternates requires an existing n32 serial result directory'; exit 2; }
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot"
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
cd "$task_root/code"
mapfile -t hosts < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
(( ${#hosts[@]} == 1 )) || exit 2
result_dir="$task_root/results/n32-one-node-${SLURM_JOB_ID}"
mkdir -p "$result_dir"
layouts=(serial:1:8 mpi:1:8 mpi:2:4 mpi:4:2)
[[ $selection != alternates ]] || layouts=(mpi:1:8 mpi:2:4)
declare -a outputs=()
for layout in "${layouts[@]}"; do
    IFS=: read -r mode ranks threads <<< "$layout"
    (( ranks * threads == 8 )) || exit 2
    choices=(--partition sources --source-order cyclic --nearfield dynamic
        --callback-sync readonly --rotor-parallel true)
    if [[ $mode == serial ]]; then
        choices=(--partition targets --source-order contiguous --nearfield static
            --callback-sync full --rotor-parallel false)
    fi
    output="$result_dir/${mode}-r${ranks}-t${threads}"
    outputs+=("$output")
    printf 'N32_BASELINE mode=%s ranks=%s threads=%s host=%s output=%s\n' \
        "$mode" "$ranks" "$threads" "${hosts[0]}" "$output"
    JULIA_NUM_THREADS=$threads mpirun --host "${hosts[0]}:8" -np "$ranks" \
        --map-by "ppr:$ranks:node:PE=$threads" --bind-to core --report-bindings \
        julia --startup-file=no --project=environments/mpi examples/mpi/rotorhover.jl \
        --mode "$mode" "${choices[@]}" --target-groups 1 --pin-threads true \
        --n 32 --revolutions 4 --steps-per-rev 36 --p-per-step 2 \
        --warmup-steps 144 --check-state false --gc-policy deferred --output "$output"
done
if [[ $selection == all ]]; then
    reference=${outputs[0]}
    outputs=("${outputs[@]:1}")
elif [[ -n $existing_four_rank ]]; then
    outputs+=("$existing_four_rank")
fi
python3 scripts/compare_mpi_rotorhover.py "$reference" "${outputs[@]}" \
    --output "$result_dir/comparison.json"
python3 scripts/summarize_mpi_rotorhover.py "$result_dir/comparison.json" \
    --csv "$result_dir/summary.csv" --markdown "$result_dir/summary.md"
printf 'N32_BASELINE_COMPLETE reference=%s comparison=%s\n' "$reference" "$result_dir/comparison.json"
