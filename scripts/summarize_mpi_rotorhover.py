#!/usr/bin/env python3
"""Summarize rotor-hover comparison JSON files and artifact directories.

Uses only the Python standard library. Timing groups keep case/software settings,
replica checking and measured rank layouts separate. --results-root remaps remote
artifact paths by directory basename. --csv also writes a neighboring -runs.csv;
--markdown saves the concise tables printed to stdout. Legacy comparisons without
all nine identity checks remain PARTIAL, even when their numerical checks pass.
"""
import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

from compare_mpi_rotorhover import IDENTITY_METADATA, SAME_METADATA, read_metadata

PHASES = ("evaluation_seconds", "exchange_seconds", "sfs_seconds",
          "probe_seconds", "callback_seconds")
EXPECTED_CHECKS = 35


def finite(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"Expected a finite nonnegative measurement, got {value!r}")
    return number


def resolve_artifact(value, results_root):
    directory = Path(value)
    if results_root is not None:
        remapped = results_root / directory.name
        if remapped.is_dir():
            directory = remapped
    if not (directory / "metadata.csv").is_file():
        raise ValueError(f"Missing metadata.csv in {directory}")
    return directory.resolve()


def load_run(directory):
    metadata = read_metadata(directory)
    with (directory / "rank_times.csv").open(newline="") as stream:
        profiles = list(csv.DictReader(stream))
    if not profiles or len(profiles) != int(metadata["ranks"]):
        raise ValueError(f"Rank profile count differs from metadata in {directory}")
    if len({int(p["rank"]) for p in profiles}) != len(profiles):
        raise ValueError(f"Duplicate rank profiles in {directory}")
    threads = {int(p.get("threads", metadata["threads_per_rank"])) for p in profiles}
    if threads != {int(metadata["threads_per_rank"])}:
        raise ValueError(f"Thread profile count differs from metadata in {directory}")
    nodes = (len({p["hostname"] for p in profiles}) if all(p.get("hostname") for p in profiles)
             else int(metadata["nodes"]) if "nodes" in metadata else None)
    if "nodes" in metadata and nodes != int(metadata["nodes"]):
        raise ValueError(f"Active node count differs from metadata in {directory}")
    rss = ([finite(p["maxrss_bytes"]) for p in profiles] if all("maxrss_bytes" in p for p in profiles)
           else [finite(metadata["max_rank_peak_rss_bytes"])] if "max_rank_peak_rss_bytes" in metadata else [])
    settings = {key: metadata.get(key) for key in SAME_METADATA}
    settings["blas_threads"] = metadata.get("blas_threads")
    digest = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
    seconds = finite(metadata["seconds"])
    if seconds <= 0:
        raise ValueError(f"Elapsed time must be positive in {directory}")
    run = {"run": directory.name, "artifact": str(directory), "settings_sha256": digest,
           "check_state": metadata.get("check_state", "unknown"),
           "mode": metadata["mode"], "nodes": nodes, "ranks": len(profiles),
           "threads_per_rank": threads.pop(), "particles": int(metadata["particles"]),
           "seconds": seconds,
           "max_rank_peak_rss_gib": max(rss) / 2**30 if rss else None,
           "validation": "UNCOMPARED", "identity_passed": 0, "identity_checked": 0,
           "checks_passed": 0, "checks_checked": 0,
           "worst_normalized_max": None, "worst_max_field": "",
           "worst_normalized_l2": None, "worst_l2_field": ""}
    available = all(p.get("phase_profile_available", "false").lower() == "true" for p in profiles)
    for phase in PHASES:
        run[phase] = max(finite(p[phase]) for p in profiles) if available else None
    return run, metadata


def apply_comparison(run, metadata, result, reference_metadata):
    identity = result.get("identity_checks")
    if identity is None:
        identity = {key: metadata[key] == reference_metadata[key] for key in IDENTITY_METADATA
                    if key in metadata and key in reference_metadata}
    checks = result.get("checks", {})
    run["identity_checked"] = sum(key in identity for key in IDENTITY_METADATA)
    run["identity_passed"] = sum(identity.get(key) is True for key in IDENTITY_METADATA)
    run["checks_checked"] = len(checks)
    run["checks_passed"] = sum(check.get("passed") is True for check in checks.values())
    failed = (result.get("passed") is not True or bool(result.get("metadata_mismatches")) or
              any(value is not True for value in identity.values()) or
              any(check.get("passed") is not True for check in checks.values()))
    complete = run["identity_checked"] == len(IDENTITY_METADATA) and len(checks) == EXPECTED_CHECKS
    run["validation"] = "FAIL" if failed else "PASS" if complete else "PARTIAL"
    for name, check in checks.items():
        for metric, field in (("normalized_max", "max"), ("normalized_l2", "l2")):
            if metric in check:
                value = finite(check[metric])
                key = "worst_" + metric
                if run[key] is None or value > run[key]:
                    run[key] = value
                    run[f"worst_{field}_field"] = name


def collect_runs(inputs, results_root):
    runs, metadata = {}, {}

    def get_run(value):
        directory = resolve_artifact(value, results_root)
        if directory not in runs:
            runs[directory], metadata[directory] = load_run(directory)
        return directory

    reports = []
    for path in inputs:
        if path.is_dir():
            get_run(path)
        else:
            with path.open() as stream:
                reports.append(json.load(stream))
    for report in reports:
        reference = get_run(report["reference"])
        if runs[reference]["mode"] != "serial" or runs[reference]["ranks"] != 1:
            raise ValueError(f"Comparison reference must be a serial process: {reference}")
        if runs[reference]["validation"] == "UNCOMPARED":
            runs[reference]["validation"] = "REFERENCE"
        for result in report["comparisons"]:
            candidate = get_run(result["candidate"])
            # Never replace failed evidence with a passing duplicate comparison.
            previous = runs[candidate]["validation"]
            apply_comparison(runs[candidate], metadata[candidate], result, metadata[reference])
            if previous == "FAIL":
                runs[candidate]["validation"] = "FAIL"
    return list(runs.values())


def summarize(runs):
    groups = {}
    for run in runs:
        key = tuple(run[field] for field in ("settings_sha256", "check_state", "mode",
                                            "nodes", "ranks", "threads_per_rank"))
        groups.setdefault(key, []).append(run)
    summaries = []
    ordered = sorted(groups.items(), key=lambda item: (*item[0][:2], item[0][2] != "serial",
                                                      item[0][3] or 0, *item[0][4:]))
    for index, (_, members) in enumerate(ordered, start=1):
        first = members[0]
        times = [run["seconds"] for run in members]
        row = {field: first[field] for field in ("settings_sha256", "check_state", "mode",
                                               "nodes", "ranks", "threads_per_rank", "particles")}
        rss = [run["max_rank_peak_rss_gib"] for run in members]
        row.update(group=f"G{index}", runs=len(members), median_seconds=statistics.median(times),
                   min_seconds=min(times), max_seconds=max(times), speedup=None,
                   max_rank_peak_rss_gib=max(rss) if all(value is not None for value in rss) else None)
        for run in members:
            run["group"] = row["group"]
        statuses = {run["validation"] for run in members}
        row["validation"] = next((status for status in ("FAIL", "UNCOMPARED", "PARTIAL")
                                  if status in statuses), "PASS" if "PASS" in statuses else "REFERENCE")
        candidates = [run for run in members if run["validation"] != "REFERENCE"]
        row["identity_checks"] = ",".join(sorted({f'{run["identity_passed"]}/9' for run in candidates})) or "reference"
        row["state_load_checks"] = ",".join(sorted({f'{run["checks_passed"]}/35' for run in candidates})) or "reference"
        for metric, field in (("normalized_max", "max"), ("normalized_l2", "l2")):
            key = "worst_" + metric
            checked = [run for run in members if run[key] is not None]
            worst = max(checked, key=lambda run: run[key]) if checked else None
            row[key] = worst[key] if worst else None
            row[f"worst_{field}_field"] = worst[f"worst_{field}_field"] if worst else ""
            row[f"worst_{field}_run"] = worst["run"] if worst else ""
        for phase in PHASES:
            values = [run[phase] for run in members if run[phase] is not None]
            row[phase] = statistics.median(values) if len(values) == len(members) else None
        summaries.append(row)
    for row in summaries:
        references = [other for other in summaries if other["mode"] == "serial" and other["ranks"] == 1
                      and other["settings_sha256"] == row["settings_sha256"]
                      and other["check_state"] == row["check_state"]]
        # Distinct serial thread layouts are different baselines, not repeat samples.
        if len(references) == 1:
            row["speedup"] = references[0]["median_seconds"] / row["median_seconds"]
        row["reference_group"] = references[0]["group"] if len(references) == 1 else ""
    return summaries


def number(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def table(headers, rows):
    return "\n".join(["| " + " | ".join(headers) + " |",
                      "| " + " | ".join("---" for _ in headers) + " |",
                      *("| " + " | ".join(str(value).replace("|", "\\|") for value in row) + " |" for row in rows)])


def markdown(groups):
    lines = ["Times exclude compilation warmup; speedup uses the matching serial median.", "",
             table(["Group", "Mode", "Nodes", "Ranks", "Threads/rank", "Runs", "Particles",
                    "Median s", "Min–max s", "Speedup", "Peak rank GiB"],
                   [(g["group"], g["mode"], g["nodes"] if g["nodes"] is not None else "—", g["ranks"], g["threads_per_rank"],
                     g["runs"], g["particles"], number(g["median_seconds"]),
                     f'{number(g["min_seconds"])}–{number(g["max_seconds"])}',
                     number(g["speedup"]), number(g["max_rank_peak_rss_gib"])) for g in groups]), "",
             table(["Group", "Settings hash", "Check state", "Validation", "Identity", "State/load", "Worst max", "Worst L2"],
                   [(g["group"], g["settings_sha256"][:12], g["check_state"], g["validation"],
                     g["identity_checks"], g["state_load_checks"],
                     "—" if g["worst_normalized_max"] is None else f'{g["worst_normalized_max"]:.3g} ({g["worst_max_field"]}, {g["worst_max_run"]})',
                     "—" if g["worst_normalized_l2"] is None else f'{g["worst_normalized_l2"]:.3g} ({g["worst_l2_field"]}, {g["worst_l2_run"]})') for g in groups]), "",
             "Median of each run's maximum-rank cumulative phase time (s):", "",
             table(["Group", "Evaluation", "Exchange", "SFS", "Probe", "Callback"],
                   [(g["group"], *(number(g[p]) for p in PHASES)) for g in groups]), "",
             "Phase maxima may come from different ranks and do not sum to wall time. Exchange excludes control collectives. "
             "Peak RSS includes imports, compilation and warmup. PARTIAL means legacy or incomplete verification; "
             "REFERENCE is the comparison baseline, and UNCOMPARED has no comparison evidence.", ""]
    if any(g["speedup"] is None for g in groups):
        lines.extend(["Missing speedups have no unique serial baseline with matching settings and check state.", ""])
    return "\n".join(lines)


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+", help="Comparison JSON files or artifact directories")
    parser.add_argument("--results-root", type=Path, help="Remap artifact directories by basename")
    parser.add_argument("--csv", type=Path, help="Grouped CSV and neighboring -runs.csv")
    parser.add_argument("--markdown", type=Path, help="Save the Markdown tables")
    args = parser.parse_args()
    try:
        runs = collect_runs(args.inputs, args.results_root)
        groups = summarize(runs)
        text = markdown(groups)
        if args.csv:
            write_csv(args.csv, groups)
            write_csv(args.csv.with_name(args.csv.stem + "-runs.csv"), runs)
        if args.markdown:
            args.markdown.parent.mkdir(parents=True, exist_ok=True)
            args.markdown.write_text(text)
        print(text, end="")
        return 1 if any(run["validation"] == "FAIL" for run in runs) else 0
    except (OSError, ValueError, KeyError, TypeError) as exception:
        parser.error(str(exception))


if __name__ == "__main__":
    raise SystemExit(main())
