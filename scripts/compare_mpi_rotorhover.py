#!/usr/bin/env python3
"""Compare headless rotor-hover artifacts; uses only the Python 3 standard library.

Usage: compare_mpi_rotorhover.py SERIAL_DIR MPI_DIR [MPI_DIR ...] [--output report.json]
Both normalized L2 <= 1e-5 and max absolute error/reference scale <= 5e-5
must pass. The scale floor is fixed before measurements: 1e-12 times a physical
characteristic scale (or one native unit for undocumented work rows). Neither
threshold is adjustable through CLI arguments. These test numerical consistency
of the chosen low-fidelity case, not agreement with experiment.
"""
import argparse
import csv
import json
import math
from array import array
from pathlib import Path

L2_TOLERANCE = 1e-5
MAX_TOLERANCE = 5e-5
SCALE_FLOOR = 1e-12
IDENTITY_METADATA = (
    "script_sha256", "solver_sha256", "package_identity_sha256", "database_sha256",
    "manifest_sha256", "julia_version", "python_version", "numpy_version", "scipy_version",
)

# Mode, rank count, thread count and run times intentionally differ. Warmup
# configuration can differ without changing the fresh measured simulation.
SAME_METADATA = (
    "format_version", "n", "revolutions", "steps_per_rev", "p_per_step", "nsteps",
    "particles", "particle_fields", "particle_time", "particle_step",
    "simulation_time", "simulation_step", "radius", "blades", "RPM", "rho", "J",
    "integration", "viscous", "SFS", "fmm_p", "fmm_ncrit",
    "fmm_relative_tolerance", "fmm_absolute_tolerance",
    "gc_policy",
    *IDENTITY_METADATA, "comparison_l2", "comparison_max",
    "comparison_scale_floor",
)


def read_metadata(directory):
    with (directory / "metadata.csv").open(newline="") as stream:
        return {row["key"]: row["value"] for row in csv.DictReader(stream)}


def read_table(directory, filename):
    with (directory / filename).open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        if len(set(header)) != len(header):
            raise ValueError(f"Duplicate columns in {directory / filename}")
        columns = {name: array("d") for name in header}
        for row_index, row in enumerate(reader, start=2):
            if len(row) != len(header):
                raise ValueError(f"Wrong column count at {directory / filename}:{row_index}")
            for name, value in zip(header, row):
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError(f"Nonfinite {name} at {directory / filename}:{row_index}")
                columns[name].append(number)
        return columns


def metric(reference, candidate, names, characteristic_scale):
    reference_squared = 0.0
    difference_squared = 0.0
    maximum_reference = 0.0
    maximum_difference = 0.0
    count = 0
    # fsum avoids precision loss in the aggregate comparison, not the solver.
    for name in names:
        ref, got = reference[name], candidate[name]
        if len(ref) != len(got):
            raise ValueError(f"Length mismatch for {name}: {len(ref)} != {len(got)}")
        reference_squared += math.fsum(x * x for x in ref)
        difference_squared += math.fsum((y - x) ** 2 for x, y in zip(ref, got))
        maximum_reference = max(maximum_reference, max(map(abs, ref), default=0.0))
        maximum_difference = max(maximum_difference, max((abs(y - x) for x, y in zip(ref, got)), default=0.0))
        count += len(ref)
    floor = SCALE_FLOOR * characteristic_scale
    l2_scale = max(math.sqrt(reference_squared), floor * math.sqrt(max(count, 1)))
    max_scale = max(maximum_reference, floor)
    l2 = math.sqrt(difference_squared) / l2_scale
    maximum = maximum_difference / max_scale
    return {"passed": l2 <= L2_TOLERANCE and maximum <= MAX_TOLERANCE,
            "normalized_l2": l2, "normalized_max": maximum,
            "absolute_l2": math.sqrt(difference_squared),
            "max_absolute_error": maximum_difference,
            "l2_scale": l2_scale, "max_scale": max_scale,
            "characteristic_scale": characteristic_scale, "entries": count}


def exact(reference, candidate, names):
    passed = all(reference[name] == candidate[name] for name in names)
    return {"passed": passed, "exact": True, "columns": names}


def load_artifact(directory):
    metadata = read_metadata(directory)
    for key in SAME_METADATA:
        if key not in metadata:
            raise ValueError(f"Missing required metadata {key} in {directory}")
    tables = {name: read_table(directory, name + ".csv")
              for name in ("particles", "history", "blade", "blade_history")}
    count = int(metadata["particles"])
    fields = int(metadata["particle_fields"])
    if fields != 46:
        raise ValueError(f"Expected the pinned 46-row FLOWVPM state, got {fields}")
    if list(tables["particles"]) != ["particle"] + [f"state_{i}" for i in range(1, fields + 1)]:
        raise ValueError("Particle-state columns do not match the declared layout")
    if list(tables["particles"]["particle"]) != list(range(1, count + 1)):
        raise ValueError("Particle identity/order or declared count is inconsistent")
    history = tables["history"]
    steps = int(metadata["nsteps"])
    if list(history["step"]) != list(range(steps + 1)):
        raise ValueError("Artifact does not contain every simulation step including initialization")
    if history["particles"][-1] != count:
        raise ValueError("Final history particle count differs from particle artifact")
    if history["time"][-1] != float(metadata["particle_time"]):
        raise ValueError("Final history time differs from particle time")
    if int(metadata["simulation_step"]) != steps or int(metadata["particle_step"]) != steps:
        raise ValueError("Simulation did not complete the requested steps")
    if not all(x.is_integer() for x in history["particles"]):
        raise ValueError("History particle counts must be integers")
    for field in ("comparison_l2", "comparison_max", "comparison_scale_floor"):
        expected = {"comparison_l2": L2_TOLERANCE, "comparison_max": MAX_TOLERANCE,
                    "comparison_scale_floor": SCALE_FLOOR}[field]
        if float(metadata[field]) != expected:
            raise ValueError(f"Artifact changed the predeclared threshold {field}")
    blade = tables["blade"]
    expected_ids = [(b, e) for b in range(1, int(metadata["blades"]) + 1)
                    for e in range(1, int(metadata["n"]) + 1)]
    if list(zip(blade["blade"], blade["element"])) != expected_ids:
        raise ValueError("Blade element identities/order do not match the case")
    blade_history = tables["blade_history"]
    expected_history_ids = [(step, blade, element) for step in range(steps + 1)
                            for blade, element in expected_ids]
    if list(zip(blade_history["step"], blade_history["blade"], blade_history["element"])) != expected_history_ids:
        raise ValueError("Artifact does not contain all blade values at every step")
    for step in range(steps + 1):
        start = step * len(expected_ids)
        if not all(time == history["time"][step]
                   for time in blade_history["time"][start:start + len(expected_ids)]):
            raise ValueError("Blade-history clocks do not match load-history clocks")
    return metadata, tables


def compare(reference_meta, reference_tables, candidate_dir):
    candidate_meta, candidate_tables = load_artifact(candidate_dir)
    mismatches = {key: [reference_meta.get(key), candidate_meta.get(key)]
                  for key in SAME_METADATA if reference_meta.get(key) != candidate_meta.get(key)}
    report = {"candidate": str(candidate_dir), "metadata_mismatches": mismatches,
              "identity_checks": {key: candidate_meta[key] == reference_meta[key]
                                  for key in IDENTITY_METADATA},
              "mode": candidate_meta["mode"],
              "partition": candidate_meta.get("mpi_partition", "targets"),
              "target_groups": candidate_meta.get("target_groups", "1"),
              "thread_pinning": candidate_meta.get("thread_pinning", "none"),
              "source_order": candidate_meta.get("source_order", "contiguous"),
              "nearfield_scheduler": candidate_meta.get("nearfield_scheduler", "static"),
              "callback_sync": candidate_meta.get("callback_sync", "full"),
              "rotor_parallel": candidate_meta.get("rotor_parallel", "false"),
              "ranks": int(candidate_meta["ranks"]),
              "threads_per_rank": int(candidate_meta["threads_per_rank"]),
              "nodes": int(candidate_meta["nodes"]),
              "max_rank_peak_rss_bytes": int(candidate_meta["max_rank_peak_rss_bytes"]),
              "sum_rank_peak_rss_bytes": int(candidate_meta["sum_rank_peak_rss_bytes"]),
              "seconds": float(candidate_meta["seconds"]),
              "warmup_seconds": float(candidate_meta["warmup_seconds"]), "checks": {}}
    if mismatches:
        report["passed"] = False
        return report
    for name in reference_tables:
        if list(reference_tables[name]) != list(candidate_tables[name]):
            raise ValueError(f"Column mismatch in {name}")
    radius = float(reference_meta["radius"])
    omega = float(reference_meta["RPM"]) * 2 * math.pi / 60
    density = float(reference_meta["rho"])
    ref, got = reference_tables["particles"], candidate_tables["particles"]
    groups = (
        ("position", range(1, 4), radius),
        ("particle_gamma", range(4, 7), omega * radius ** 3),
        ("core_size", [7], radius),
        ("volume", [8], radius ** 3),
        ("particle_circulation", [9], omega * radius ** 2),
        ("velocity", range(10, 13), omega * radius),
        ("vorticity", range(13, 16), omega),
        ("velocity_gradient", range(16, 25), omega),
        ("pse", range(25, 28), omega ** 2 * radius ** 3),
        ("model_coefficient", range(37, 40), 1.0),
        ("sfs", range(40, 43), omega ** 2 * radius ** 3),
        ("previous_velocity", [44], omega * radius),
    )
    for name, rows, scale in groups:
        report["checks"][name] = metric(ref, got, [f"state_{i}" for i in rows], scale)
    # Work rows lack one common physical dimension; check each independently.
    for row in range(28, 37):
        report["checks"][f"work_row_{row}"] = metric(ref, got, [f"state_{row}"], 1.0)
    report["checks"]["particle_flags_and_reserved"] = exact(ref, got,
        ["particle", "state_43", "state_45", "state_46"])
    ref, got = reference_tables["history"], candidate_tables["history"]
    report["checks"]["history_clocks_and_counts"] = exact(ref, got,
        ["step", "time", "dt", "particles", "hub_omitted"])
    for name, scale in (("thrust", density * omega ** 2 * radius ** 4),
                        ("torque", density * omega ** 2 * radius ** 5),
                        ("CT", 1.0), ("CQ", 1.0)):
        report["checks"]["history_" + name] = metric(ref, got, [name], scale)
    ref, got = reference_tables["blade"], candidate_tables["blade"]
    report["checks"]["blade_identity"] = exact(ref, got, ["blade", "element"])
    for name, scale in (("Gamma", omega * radius ** 2),
                        ("Np", density * omega ** 2 * radius ** 3),
                        ("Tp", density * omega ** 2 * radius ** 3)):
        report["checks"]["blade_" + name] = metric(ref, got, [name], scale)
    ref, got = reference_tables["blade_history"], candidate_tables["blade_history"]
    report["checks"]["blade_history_identity_and_clock"] = exact(ref, got,
        ["step", "time", "blade", "element"])
    for name, scale in (("Gamma", omega * radius ** 2),
                        ("Np", density * omega ** 2 * radius ** 3),
                        ("Tp", density * omega ** 2 * radius ** 3)):
        report["checks"]["blade_history_" + name] = metric(ref, got, [name], scale)
    report["passed"] = all(check["passed"] for check in report["checks"].values())
    report["speedup"] = float(reference_meta["seconds"]) / report["seconds"]
    report["rank_efficiency"] = report["speedup"] / report["ranks"]
    report["node_efficiency"] = report["speedup"] * int(reference_meta["nodes"]) / report["nodes"]
    report["thread_efficiency"] = (report["speedup"] * int(reference_meta["threads_per_rank"]) /
                                   (report["ranks"] * report["threads_per_rank"]))
    report["node_hours"] = report["seconds"] * report["nodes"] / 3600
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidates", type=Path, nargs="+")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reference_meta, reference_tables = load_artifact(args.reference)
    if reference_meta["mode"] != "serial" or int(reference_meta["ranks"]) != 1:
        parser.error("Reference must be a one-process serial run")
    report = {"reference": str(args.reference), "normalized_l2_tolerance": L2_TOLERANCE,
              "normalized_max_tolerance": MAX_TOLERANCE, "scale_floor": SCALE_FLOOR,
              "reference_identity": {key: reference_meta[key] for key in IDENTITY_METADATA},
              "reference_seconds": float(reference_meta["seconds"]), "comparisons": []}
    for candidate in args.candidates:
        try:
            result = compare(reference_meta, reference_tables, candidate)
        except (ValueError, KeyError, OSError) as exception:
            result = {"candidate": str(candidate), "passed": False, "error": str(exception)}
        report["comparisons"].append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {candidate}")
        if "speedup" in result:
            print(f"  seconds={result['seconds']:.6f}, speedup={result['speedup']:.4f}, "
                  f"reference_rank_efficiency={result['rank_efficiency']:.4f}")
        if result.get("metadata_mismatches"):
            print("  incompatible metadata: " + ", ".join(result["metadata_mismatches"]))
        for name, check in result.get("checks", {}).items():
            if not check["passed"]:
                print(f"  failed {name}: {check}")
        if "error" in result:
            print("  " + result["error"])
    report["passed"] = all(result["passed"] for result in report["comparisons"])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
