"""Evaluate explicit maneuver hypotheses on an already reviewed association.

This is a finite candidate comparison, not an automatic declaration of truth.
An invalid candidate is reported as invalid, never as zero residual.
"""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

from .analyze import residuals
from .detections import associate, load_events
from .scenario import load_scenario
from .truth import ModelOptions, TruthCalculator


def compare(scenario, tracks, mapping, base):
    variants = {
        "base": {}, "gravity_9_8": {"gravity_mps2": 9.8},
        "gravity_9_80665": {"gravity_mps2": 9.80665},
        "spherical_earth": {"earth_model": "sphere", "vertical_angle_mode": "flight_path"},
        "horizontal_speed": {"speed_kind": "horizontal"},
        "flight_path_angle": {"vertical_angle_mode": "flight_path"},
        "target_relative_clock": {"maneuver_clock": "target"},
        "reverse_turns": {"maneuver_actions": ("climb", "descend", "right", "left")},
        "reverse_vertical": {"maneuver_actions": ("descend", "climb", "left", "right")},
        "vertical_acceleration": {"vertical_model": "acceleration"},
        "vertical_discrete_0_1": {"vertical_model": "discrete_triangular", "vertical_update_step_s": 0.1},
        "vertical_rate_10": {"vertical_model": "rate"},
        "load_factor": {"turn_model": "load_factor"},
        "constant_turn_rate_3": {"turn_model": "rate"},
        "rotating_acceleration_initial": {"turn_model": "rotating_acceleration", "turn_acceleration_reference_deg": None},
        "rotating_acceleration_90": {"turn_model": "rotating_acceleration", "turn_acceleration_reference_deg": 90},
        "speed_coupled_rotation_90": {"turn_model": "speed_coupled_rotation", "turn_acceleration_reference_deg": 90,
                                     "turn_duration_step_s": 0.1},
        "coupled_turn_discrete_vertical": {"turn_model": "speed_coupled_rotation", "turn_acceleration_reference_deg": 90,
                                            "turn_duration_step_s": 0.1, "vertical_model": "discrete_triangular"},
        "return_initial": {"post_maneuver": "initial"},
        "absolute_goals": {"turn_goal": "absolute", "vertical_goal": "absolute"},
    }
    result = []
    for name, changes in variants.items():
        options = replace(base, **changes)
        try:
            rows, _ = residuals(TruthCalculator(scenario, options), tracks, mapping)
            per_target = {}
            for target in scenario.targets:
                selected = [row for row in rows if row["target"] == target.number and row["active"] and not row["ambiguous"]]
                per_target[target.number] = math.sqrt(sum(row["position_error_m"] ** 2 for row in selected) / (3 * len(selected))) if selected else None
            rms = math.sqrt(sum(row["position_error_m"] ** 2 for row in rows) / (3 * len(rows)))
            result.append({"name": name, "changes": changes, "status": "evaluated", "rms_xyz_m": rms, "per_target_rms_xyz_m": per_target})
        except ValueError as error:
            result.append({"name": name, "changes": changes, "status": "invalid", "reason": str(error)})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--ini", type=Path, required=True)
    parser.add_argument("--options", type=Path, required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--start-ms", type=int, required=True)
    parser.add_argument("--end-ms", type=int, required=True)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("stride must be positive")
    try:
        tracks = associate(load_events(args.csv, args.start_ms, args.end_ms))
        for track in tracks:
            track.plots = track.plots[::args.stride]
        result = compare(load_scenario(args.ini), tracks, json.loads(args.mapping.read_text(encoding="utf-8")), ModelOptions.load(args.options))
        args.output.write_text(json.dumps({"stride_per_track": args.stride, "candidates": result}, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
    except (ValueError, OSError, TypeError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
