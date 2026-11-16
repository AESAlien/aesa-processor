"""Rank unit/frame hypotheses using only pre-first-maneuver observations.

The supplied approximate phase origin selects data; ranking is conditional on
that selection and on the independently reviewed continuity-track mapping.
Only equivalent relative ENU conventions are tested, not absolute true north.
"""

import argparse
from dataclasses import asdict, replace
import itertools
import json
import math
from pathlib import Path

from .analyze import phase_time
from .detections import associate, load_events
from .geometry import dot
from .scenario import load_scenario
from .truth import ModelOptions, TruthCalculator


def solve_clock(vectors, errors, cycles, fit_period):
    a = sum(dot(v, v) for v in vectors)
    b = sum(dot(v, v) * cycle for v, cycle in zip(vectors, cycles))
    d = sum(dot(v, v) * cycle * cycle for v, cycle in zip(vectors, cycles))
    u = sum(dot(v, error) for v, error in zip(vectors, errors))
    w = sum(dot(v, error) * cycle for v, error, cycle in zip(vectors, errors, cycles))
    if not a:
        raise ValueError("No motion information for clock fit")
    determinant = a * d - b * b
    if fit_period and determinant > a * d * 1e-12:
        return (u * d - w * b) / determinant, (w * a - u * b) / determinant
    return u / a, 0.0


def fit(scenario, tracks, mapping, origin, period, fixed_options=None):
    seed = ModelOptions(start_offset_s=origin, scenario_repeat_period_s=period, field10_role="start_time")
    points = []
    for track in tracks:
        if str(track.number) not in mapping:
            continue
        number = int(mapping[str(track.number)])
        target = scenario.target(number)
        first = min((m.start for m in target.maneuvers), default=math.inf)
        for plot in track.plots:
            phase = phase_time(plot.time_ms / 1000, seed)
            if target.values[9] + 0.1 <= phase < first - 0.5 and plot.row not in track.ambiguous_rows:
                cycle = math.floor((plot.time_ms / 1000 - origin) / period) if period else 0
                points.append((number, plot, phase, cycle))
    if len(points) < 3:
        raise ValueError("Insufficient initial straight observations")
    candidates = []
    attempted = 0
    combinations = (fixed_options,) if fixed_options is not None else (
        ModelOptions(range_kind=rk, altitude_reference=ar, speed_scale_mps=ss, altitude_scale_m=az,
                     range_scale_m=rs, vertical_kind=vk, heading_sign=hs, heading_zero_deg=hz, vertical_sign=vs)
        for rk, ar, ss, az, rs, vk, hs, hz, vs in itertools.product(
            ("ground", "slant"), ("radar", "msl"), (1, 1852 / 3600, 1 / 3.6), (1, 0.3048), (1000, 1852),
            ("angle", "rate"), (1, -1), (0, 90, 180, 270), (1, -1)))
    for options in combinations:
        attempted += 1
        calculator = TruthCalculator(scenario, replace(options, start_offset_s=0, scenario_repeat_period_s=None,
                                                       field10_role="ignore", field11_role="ignore"))
        initial = {target.number: calculator.predict(target.number, 0) for target in scenario.targets}
        vectors, errors, cycles = [], [], []
        for number, plot, phase, cycle in points:
            state = initial[number]
            elapsed = phase - scenario.target(number).values[9]
            vectors.append(state.velocity_enu_mps)
            errors.append(tuple(p + v * elapsed - observed for p, v, observed in zip(state.position_enu_m, state.velocity_enu_mps, plot.position)))
            cycles.append(cycle)
        delta_origin, delta_period = solve_clock(vectors, errors, cycles, period is not None)
        corrected_period = period + delta_period if period else None
        if corrected_period is not None and corrected_period <= 0:
            continue
        residual = [error[i] - v[i] * (delta_origin + cycle * delta_period)
                    for v, error, cycle in zip(vectors, errors, cycles) for i in range(3)]
        rms = math.sqrt(sum(value * value for value in residual) / len(residual))
        fitted = replace(options, field10_role="start_time", start_offset_s=origin + delta_origin,
                         scenario_repeat_period_s=corrected_period, field11_role="end_time", maneuver_clock="scenario")
        candidates.append({"rms_xyz_m": rms, "options": asdict(fitted)})
    candidates.sort(key=lambda row: row["rms_xyz_m"])
    return {"status": "clock_only_calibration_on_same_data" if fixed_options else "calibration_on_same_data_not_independent_validation",
            "selected_rows": [plot.row for _, plot, _, _ in points],
            "selected_cycles": sorted({cycle for _, _, _, cycle in points}),
            "period_identifiable_from_selection": period is not None and len({cycle for _, _, _, cycle in points}) > 1,
            "counts_by_target": {target.number: sum(number == target.number for number, _, _, _ in points) for target in scenario.targets},
            "selection_origin_s": origin, "selection_period_s": period,
            "attempted_candidate_count": attempted, "rejected_candidate_count": attempted - len(candidates),
            "candidate_count": len(candidates), "top_candidates": candidates[:12]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--ini", type=Path, required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--start-ms", type=int, required=True)
    parser.add_argument("--end-ms", type=int, required=True)
    parser.add_argument("--phase-origin-s", type=float, required=True)
    parser.add_argument("--period-s", type=float)
    parser.add_argument("--clock-options", type=Path, help="Keep these physical options; fit only clock origin and period")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.period_s is not None and args.period_s <= 0:
        parser.error("period-s must be positive")
    try:
        mapping = json.loads(args.mapping.read_text(encoding="utf-8"))
        tracks = associate(load_events(args.csv, args.start_ms, args.end_ms))
        fixed_options = ModelOptions.load(args.clock_options) if args.clock_options else None
        result = fit(load_scenario(args.ini), tracks, mapping, args.phase_origin_s, args.period_s, fixed_options)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "straight_fit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        (args.output / "straight_options.json").write_text(json.dumps(result["top_candidates"][0]["options"], indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"counts_by_target": result["counts_by_target"], "best_rms_xyz_m": result["top_candidates"][0]["rms_xyz_m"],
                          "origin_s": result["top_candidates"][0]["options"]["start_offset_s"],
                          "period_s": result["top_candidates"][0]["options"]["scenario_repeat_period_s"]}, indent=2))
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
