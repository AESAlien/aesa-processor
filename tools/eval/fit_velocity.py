"""Fit constant ENU velocity to an explicitly selected observed phase interval.

This is an observation diagnostic, not a fit of simulator motion options.
Repeated phases are pooled; stable straight motion must be reviewed separately.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from .analyze import statistics_for


def fit_velocity(rows):
    if len(rows) < 3 or len({row["phase_s"] for row in rows}) < 2:
        raise ValueError("Velocity fit requires at least three observations and distinct phases")
    components, errors = [], []
    for axis in ("e", "n", "u"):
        name = f"observed_{axis}_m"
        result = statistics_for(rows, name)
        slope = result["slope_per_s"]
        intercept = result["mean"] - slope * statistics.mean(row["phase_s"] for row in rows)
        components.append(slope)
        errors.append(math.sqrt(statistics.mean((row[name] - intercept - slope * row["phase_s"]) ** 2 for row in rows)))
    return {"status": "observed_linear_fit_not_truth", "count": len(rows),
            "phase_min_s": min(row["phase_s"] for row in rows), "phase_max_s": max(row["phase_s"] for row in rows),
            "velocity_enu_mps": components, "speed_mps": math.hypot(*components),
            "heading_deg": math.degrees(math.atan2(components[0], components[1])), "position_rms_enu_m": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--residuals", required=True, type=Path)
    parser.add_argument("--target", required=True, type=int)
    parser.add_argument("--from-phase", required=True, type=float)
    parser.add_argument("--to-phase", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.from_phase) or not math.isfinite(args.to_phase) or args.from_phase >= args.to_phase:
        parser.error("Require finite from-phase < to-phase")
    try:
        with args.residuals.open(encoding="utf-8", newline="") as stream:
            rows = [{name: float(row[name]) for name in ("phase_s", "observed_e_m", "observed_n_m", "observed_u_m")}
                    for row in csv.DictReader(stream) if int(row["target"]) == args.target and row["active"] == "True"
                    and row["ambiguous"] == "False" and args.from_phase <= float(row["phase_s"]) < args.to_phase]
        result = fit_velocity(rows)
        result.update({"residuals": str(args.residuals.resolve()), "target": args.target,
                       "selection_from_s": args.from_phase, "selection_to_s": args.to_phase})
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
