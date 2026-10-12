#!/bin/bash
#SBATCH --job-name=flow-n32-source-repeat
#SBATCH --partition=compute
#SBATCH --nodes=3
#SBATCH --ntasks-per-node=4
#SBATCH --cpus-per-task=4
#SBATCH --exclusive
#SBATCH --time=01:00:00
set -euo pipefail
# Read frozen v6 directly; reuse job 37's exported n32 serial reference.
task_root=${1:?Pass the frozen v6 task directory}
reference=${2:?Pass the exported n32 serial result from job 37}
[[ -f "$reference/metadata.toml" ]] || { echo 'The serial reference must already be exported'; exit 2; }
source /etc/profile.d/modules.sh
module use /opt/amazon/modules/modulefiles
module load openmpi5/5.0.9amzn1
export PATH="$task_root/runtime/julia-1.13.1+0.x64.linux.gnu/bin:/opt/slurm/bin:$PATH"
export JULIA_DEPOT_PATH="$task_root/depot" JULIA_NUM_THREADS=2
export MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPAL_PREFIX=/opt/amazon/openmpi5
export OMPI_MCA_pml=ob1 OMPI_MCA_btl=self,sm,tcp
export OMPI_MCA_btl_tcp_if_include=10.0.0.0/16
cd "$task_root/code"
mapfile -t hosts < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
(( ${#hosts[@]} == 3 )) || exit 2
result_dir="$task_root/results/n32-source4r2-repeats-${SLURM_JOB_ID}"
mkdir -p "$result_dir"
declare -a outputs=()
for repeat in 1 2; do
    if (( repeat == 1 )); then
        node_counts=(3 1 2)
        run_hosts=("${hosts[@]}")
    else
        node_counts=(1 2 3)
        run_hosts=("${hosts[@]:1}" "${hosts[0]}")
    fi
    for nodes in "${node_counts[@]}"; do
        ranks=$((4 * nodes))
        host_list=$(printf '%s:8,' "${run_hosts[@]:0:nodes}")
        host_list=${host_list%,}
        output="$result_dir/mpi-sources-n${nodes}-r${ranks}-t2-rep${repeat}"
        outputs+=("$output")
        printf 'N32_REPEAT repeat=%s nodes=%s ranks=%s hosts=%s output=%s\n' \
            "$repeat" "$nodes" "$ranks" "$host_list" "$output"
        mpirun --host "$host_list" -np "$ranks" \
            --map-by ppr:4:node:PE=2 --bind-to core --report-bindings \
            julia --startup-file=no --project=environments/mpi examples/mpi/rotorhover.jl \
            --mode mpi --partition sources --source-order cyclic --target-groups 1 \
            --nearfield dynamic --callback-sync readonly --rotor-parallel true \
            --pin-threads true --n 32 --revolutions 4 --steps-per-rev 36 --p-per-step 2 \
            --warmup-steps 144 --check-state false --gc-policy deferred --output "$output"
    done
done
python3 scripts/compare_mpi_rotorhover.py "$reference" "${outputs[@]}" \
    --output "$result_dir/comparison.json"
python3 scripts/summarize_mpi_rotorhover.py "$result_dir/comparison.json" \
    --csv "$result_dir/summary.csv" --markdown "$result_dir/summary.md"
printf 'N32_REPEAT_COMPLETE reference=%s comparison=%s\n' "$reference" "$result_dir/comparison.json"
