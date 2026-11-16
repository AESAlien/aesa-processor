"""Compare two operation sessions at common scenario phase, without refitting.

Linear interpolation is restricted to a stated maximum sampling gap; reported
differences include measurement and interpolation errors. Repeated cycles in
one session are not counted as independent program restarts.
"""

import argparse
import bisect
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

from .analyze import statistics_for, write_csv


def interpolate(points, phase, maximum_gap):
    times = [point["phase_s"] for point in points]
    index = bisect.bisect_left(times, phase)
    if index < len(points) and abs(times[index] - phase) < 1e-9:
        return points[index], 0.0
    if index == 0 or index == len(points):
        return None, None
    left, right = points[index - 1], points[index]
    gap = right["phase_s"] - left["phase_s"]
    if not 0 < gap <= maximum_gap:
        return None, None
    weight = (phase - left["phase_s"]) / gap
    names = ("observed_e_m", "observed_n_m", "observed_u_m", "observed_doppler_mps")
    return {name: left[name] + weight * (right[name] - left[name]) for name in names}, gap


def selected_bytes(summary):
    data = Path(summary["csv"]).read_bytes().splitlines(keepends=True)
    return data[0] + b"".join(line for line in data[1:] if summary["first_ms"] <= int(line.split(b",")[2]) <= summary["last_ms"])


def read_run(directory):
    directory = Path(directory)
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    with (directory / "assignments.csv").open(newline="", encoding="utf-8") as stream:
        doppler = {int(row["row"]): float(row["doppler_mps"]) for row in csv.DictReader(stream)}
    with (directory / "residuals.csv").open(newline="", encoding="utf-8") as stream:
        rows = []
        for raw in csv.DictReader(stream):
            row = {name: float(value) for name, value in raw.items() if name in ("phase_s", "time_ms", "target", "row", "observed_e_m", "observed_n_m", "observed_u_m")}
            row["observed_doppler_mps"] = doppler[int(row["row"])]
            rows.append(row)
    return summary, rows


def measurement(point):
    east, north, up = (point[name] for name in ("observed_e_m", "observed_n_m", "observed_u_m"))
    return {"range_m": math.sqrt(east * east + north * north + up * up),
            "azimuth_deg": math.degrees(math.atan2(east, north)),
            "elevation_deg": math.degrees(math.atan2(up, math.hypot(east, north))),
            "doppler_mps": point["observed_doppler_mps"]}


def compare(first_summary, first, second_summary, second, maximum_gap=1.2):
    if first_summary["options"] != second_summary["options"]:
        raise ValueError("Two sessions must use exactly the same candidate options")
    options = first_summary["options"]
    period = options["scenario_repeat_period_s"]
    if period is None:
        raise ValueError("Repeated scenario comparison requires a period")
    groups = defaultdict(list)
    for row in second:
        cycle = math.floor((row["time_ms"] / 1000 - options["start_offset_s"]) / period)
        groups[(int(row["target"]), cycle)].append(row)
    for points in groups.values():
        points.sort(key=lambda row: row["phase_s"])
    pairs = []
    for row in first:
        target = int(row["target"])
        reference = measurement(row)
        for (number, cycle), points in groups.items():
            if number != target:
                continue
            point, gap = interpolate(points, row["phase_s"], maximum_gap)
            if point is None:
                continue
            observed = measurement(point)
            difference = {name: observed[name] - value for name, value in reference.items()}
            difference["azimuth_deg"] = (difference["azimuth_deg"] + 180) % 360 - 180
            pairs.append({"target": target, "first_row": int(row["row"]), "second_cycle": cycle,
                          "phase_s": row["phase_s"], "interpolation_gap_s": gap, **difference})
    raw_first, raw_second = selected_bytes(first_summary), selected_bytes(second_summary)
    summary = {"status": "same_model_no_second_session_refit", "max_interpolation_gap_s": maximum_gap,
               "raw_selected_bytes_identical": raw_first == raw_second,
               "first_selected_sha256": hashlib.sha256(raw_first).hexdigest(),
               "second_selected_sha256": hashlib.sha256(raw_second).hexdigest(),
               "first_plot_count": first_summary["plot_count"], "second_plot_count": second_summary["plot_count"],
               "pair_count": len(pairs), "targets": {}}
    for target in sorted({int(row["target"]) for row in first}):
        selected = [row for row in pairs if row["target"] == target]
        summary["targets"][target] = {name: statistics_for(selected, name) for name in ("range_m", "azimuth_deg", "elevation_deg", "doppler_mps")}
    return summary, pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first", type=Path, required=True)
    parser.add_argument("--second", type=Path, required=True)
    parser.add_argument("--max-gap-s", type=float, default=1.2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.max_gap_s) or args.max_gap_s <= 0:
        parser.error("max-gap-s must be positive and finite")
    try:
        first_summary, first = read_run(args.first)
        second_summary, second = read_run(args.second)
        summary, pairs = compare(first_summary, first, second_summary, second, args.max_gap_s)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "comparison.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        write_csv(args.output / "pairs.csv", pairs)
        print(json.dumps(summary, indent=2))
    except (ValueError, TypeError, KeyError, OSError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
