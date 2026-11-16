"""Associate real plots and compute observed-minus-predicted residuals."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from .detections import associate, load_events
from .geometry import norm
from .scenario import load_scenario
from .truth import ModelOptions, TruthCalculator


MEASUREMENTS = ("range_m", "azimuth_deg", "elevation_deg", "doppler_mps")


def statistics_for(samples, name):
    values = [sample[name] for sample in samples]
    times = [sample["phase_s"] for sample in samples]
    if not values:
        return {"count": 0}
    mean, time_mean = statistics.mean(values), statistics.mean(times)
    denominator = sum((time - time_mean) ** 2 for time in times)
    slope = sum((t - time_mean) * (v - mean) for t, v in zip(times, values)) / denominator if denominator else None
    return {"count": len(values), "mean": mean, "rms": math.sqrt(statistics.mean(v * v for v in values)),
            "max_abs": max(abs(v) for v in values), "slope_per_s": slope}


def phase_time(time_s, options):
    phase = time_s - options.start_offset_s
    if options.scenario_repeat_period_s is not None and phase >= 0:
        phase %= options.scenario_repeat_period_s
    return phase


def track_summary(track):
    return {"track": track.number, "count": len(track.plots),
            "first_ms": track.plots[0].time_ms, "last_ms": track.plots[-1].time_ms,
            "first_position": track.plots[0].position, "last_position": track.plots[-1].position,
            "ambiguous_rows": track.ambiguous_rows}


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def residuals(calculator, tracks, mapping):
    rows, unassigned = [], []
    for track in tracks:
        if str(track.number) not in mapping:
            unassigned.extend(plot.row for plot in track.plots)
            continue
        number = int(mapping[str(track.number)])
        target = calculator.scenario.target(number)
        first_maneuver = min((m.start * calculator.options.maneuver_time_scale_s for m in target.maneuvers), default=math.inf)
        if calculator.options.maneuver_clock == "target":
            if calculator.options.field10_role == "start_time":
                first_maneuver += target.values[9] * calculator.options.target_time_scale_s
        for plot in track.plots:
            prediction = calculator.predict(number, plot.time_ms / 1000)
            phase = phase_time(plot.time_ms / 1000, calculator.options)
            starts = [(m.start * calculator.options.maneuver_time_scale_s, m.kind) for m in target.maneuvers]
            if calculator.options.maneuver_clock == "target" and calculator.options.field10_role == "start_time":
                starts = [(time + target.values[9] * calculator.options.target_time_scale_s, kind) for time, kind in starts]
            # A window remains assigned to its scheduled maneuver until the next
            # scheduled start. This avoids dropping errors just after a wrong model ends.
            window = next((kind for start, kind in reversed(starts) if start <= phase), 0)
            rows.append({"row": plot.row, "track": track.number, "target": number, "time_ms": plot.time_ms,
                         "phase_s": phase, "active": prediction.active,
                         "ambiguous": plot.row in track.ambiguous_rows,
                         "initial_straight": phase < first_maneuver - 0.5,
                         "maneuver_active": prediction.maneuver_kind or 0, "maneuver_window": window,
                         "range_m": plot.slant_range - prediction.slant_range_m,
                         "azimuth_deg": (plot.azimuth - prediction.azimuth_deg + 180) % 360 - 180,
                         "elevation_deg": plot.elevation - prediction.elevation_deg,
                         "doppler_mps": plot.doppler - prediction.doppler_velocity_mps,
                         "position_error_m": norm(tuple(a - b for a, b in zip(plot.position, prediction.position_enu_m))),
                         "up_m": plot.position[2] - prediction.position_enu_m[2],
                         "observed_e_m": plot.position[0], "observed_n_m": plot.position[1], "observed_u_m": plot.position[2],
                         "predicted_e_m": prediction.position_enu_m[0], "predicted_n_m": prediction.position_enu_m[1],
                         "predicted_u_m": prediction.position_enu_m[2]})
    return sorted(rows, key=lambda row: (row["time_ms"], row["row"])), unassigned


def summarize(calculator, rows):
    summary = {"targets": {}, "maneuver_windows": {}, "maneuver_active": {}, "post_maneuver": {}}
    for target in calculator.scenario.targets:
        selected = [row for row in rows if row["target"] == target.number and row["active"] and not row["ambiguous"]]
        straight = [row for row in selected if row["initial_straight"]]
        field_start = target.values[9] * calculator.options.target_time_scale_s if calculator.options.field10_role == "start_time" else 0
        initial = calculator.predict(target.number, calculator.options.start_offset_s + field_start + 1e-8)
        tolerance = norm(initial.velocity_enu_mps) * 0.020
        summary["targets"][target.number] = {
            "count": len(selected), "initial_straight_count": len(straight),
            "distance_tolerance_m": tolerance,
            "initial_straight": {name: statistics_for(straight, name) for name in (*MEASUREMENTS, "up_m")},
            "all": {name: statistics_for(selected, name) for name in (*MEASUREMENTS, "up_m")},
            "passes_distance_max": bool(straight) and max(abs(row["range_m"]) for row in straight) <= tolerance,
        }
    for kind in range(1, 5):
        for field, destination in (("maneuver_window", "maneuver_windows"), ("maneuver_active", "maneuver_active")):
            selected = [row for row in rows if row[field] == kind and row["active"] and not row["ambiguous"]]
            summary[destination][kind] = {name: statistics_for(selected, name) for name in (*MEASUREMENTS, "up_m")}
        after = [row for row in rows if row["maneuver_window"] == kind and row["maneuver_active"] == 0
                 and row["active"] and not row["ambiguous"]]
        summary["post_maneuver"][kind] = {name: statistics_for(after, name) for name in (*MEASUREMENTS, "up_m")}
    return summary


def plot_results(rows, destination):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise ValueError("--plots requires matplotlib; residual CSV/JSON do not") from error
    targets = sorted({row["target"] for row in rows})
    colors = {number: plt.get_cmap("tab10")((number - 1) % 10) for number in targets}
    figure, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True, constrained_layout=True)
    for number in targets:
        selected = [row for row in rows if row["target"] == number]
        for axis, name in zip(axes, MEASUREMENTS):
            axis.scatter([row["phase_s"] for row in selected], [row[name] for row in selected],
                         s=6, alpha=0.6, color=colors[number], label=f"T{number}")
    for axis, name in zip(axes, MEASUREMENTS):
        axis.axhline(0, color="black", linewidth=0.6)
        axis.set_ylabel(name)
        axis.grid(alpha=0.25)
    axes[0].legend(ncol=10, fontsize=8)
    axes[0].set_title("Observed - candidate model; environment conditions recorded in report")
    axes[-1].set_xlabel("Scenario phase (s); repeated cycles overlaid")
    figure.savefig(destination / "residuals.png", dpi=160)
    figure.savefig(destination / "residuals.svg")
    plt.close(figure)
    columns = min(5, len(targets)) or 1
    count_rows = max(1, math.ceil(len(targets) / columns))
    figure, axes = plt.subplots(count_rows, columns, figsize=(3 * columns + 2, 3 * count_rows),
                                squeeze=False, constrained_layout=True)
    for number, axis in zip(targets, axes.flat):
        selected = [row for row in rows if row["target"] == number and row["initial_straight"]]
        axis.scatter([row["phase_s"] for row in selected], [row["range_m"] for row in selected], s=10, color=colors[number])
        axis.set_title(f"T{number}: initial straight")
        axis.set_xlabel("Scenario phase (s)")
        axis.set_ylabel("Range residual (m)")
        axis.axhline(0, color="black", linewidth=.5)
        axis.grid(alpha=.25)
    for axis in list(axes.flat)[len(targets):]:
        axis.set_visible(False)
    figure.savefig(destination / "straight_range.png", dpi=160)
    figure.savefig(destination / "straight_range.svg")
    plt.close(figure)
    if len(targets) == 1:
        selected = sorted(rows, key=lambda row: row["phase_s"])
        phases = [row["phase_s"] for row in selected]
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
        axes[0].scatter(phases, [row["observed_u_m"] for row in selected], s=10,
                        alpha=0.6, label="Observed")
        axes[0].plot(phases, [row["predicted_u_m"] for row in selected],
                     color="black", linewidth=1, label="Candidate model")
        axes[0].set_ylabel("Radar-relative up (m)")
        axes[0].set_title("Single-target height; repeated cycles overlaid")
        axes[0].legend()
        axes[1].scatter(phases, [row["up_m"] for row in selected], s=10, alpha=0.6)
        axes[1].axhline(0, color="black", linewidth=0.6)
        axes[1].set_ylabel("Observed - candidate up (m)")
        axes[1].set_xlabel("Scenario phase (s)")
        for axis in axes:
            axis.grid(alpha=0.25)
        figure.savefig(destination / "height.png", dpi=160)
        figure.savefig(destination / "height.svg")
        plt.close(figure)
        figure, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
        axis.scatter([row["observed_e_m"] / 1000 for row in selected],
                     [row["observed_n_m"] / 1000 for row in selected], s=10, alpha=0.6, label="Observed")
        axis.plot([row["predicted_e_m"] / 1000 for row in selected],
                  [row["predicted_n_m"] / 1000 for row in selected], color="black", linewidth=1, label="Candidate")
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("Radar-relative east (km)")
        axis.set_ylabel("Radar-relative north (km)")
        axis.set_title("Single-target horizontal trajectory; repeated cycles overlaid")
        axis.grid(alpha=0.25)
        axis.legend()
        figure.savefig(destination / "trajectory.png", dpi=160)
        figure.savefig(destination / "trajectory.svg")
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--start-ms", type=int)
    parser.add_argument("--end-ms", type=int)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ini", type=Path)
    parser.add_argument("--options", type=Path)
    parser.add_argument("--mapping", type=Path, help="JSON object: continuity track number -> TARGET number")
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()
    try:
        args.output.mkdir(parents=True, exist_ok=True)
        events = load_events(args.csv, args.start_ms, args.end_ms)
        tracks = associate(events)
        overview = {"csv": str(args.csv.resolve()), "event_count": len(events),
                    "empty_event_count": sum(not event.plots for event in events),
                    "plot_count": sum(len(event.plots) for event in events),
                    "first_ms": events[0].time_ms, "last_ms": events[-1].time_ms,
                    "association": {"max_gap_s": 6, "max_speed_mps": 2500, "position_gate_m": 200, "acceleration_gate_mps2": 120},
                    "tracks": [track_summary(track) for track in tracks]}
        assignments = [{"row": plot.row, "track": track.number, "time_ms": plot.time_ms,
                        "range_m": plot.slant_range, "azimuth_deg": plot.azimuth,
                        "elevation_deg": plot.elevation, "doppler_mps": plot.doppler,
                        "ambiguous": plot.row in track.ambiguous_rows} for track in tracks for plot in track.plots]
        write_csv(args.output / "assignments.csv", assignments)
        if args.mapping:
            if not args.ini or not args.options:
                parser.error("--mapping requires --ini and --options")
            mapping = json.loads(args.mapping.read_text(encoding="utf-8-sig"))
            if not isinstance(mapping, dict) or set(mapping) - {str(track.number) for track in tracks}:
                raise ValueError("Mapping must be an object with existing continuity-track numbers")
            calculator = TruthCalculator(load_scenario(args.ini), ModelOptions.load(args.options))
            rows, unassigned = residuals(calculator, tracks, mapping)
            write_csv(args.output / "residuals.csv", rows)
            overview.update(calculator.describe())
            overview["mapping"] = mapping
            overview["unassigned_rows"] = unassigned
            overview["inactive_prediction_rows"] = [row["row"] for row in rows if not row["active"]]
            overview["residual_summary"] = summarize(calculator, rows)
            if args.plots:
                plot_results(rows, args.output)
        (args.output / "summary.json").write_text(json.dumps(overview, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps({key: overview[key] for key in ("event_count", "empty_event_count", "plot_count", "first_ms", "last_ms")}, indent=2))
    except (ValueError, OSError, TypeError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
