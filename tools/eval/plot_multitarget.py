"""Plot observed/model paths and per-target range residuals from saved analysis."""

import argparse
import csv
import json
import math
from pathlib import Path


def plot(analysis, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    summary = json.loads((analysis / "summary.json").read_text(encoding="utf-8"))
    with (analysis / "residuals.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    targets = sorted({int(row["target"]) for row in rows})
    if not targets:
        raise ValueError("No target observations")
    columns = min(3, len(targets))
    count_rows = math.ceil(len(targets) / columns)
    destination.mkdir(parents=True, exist_ok=True)
    for kind in ("trajectory", "height", "range"):
        figure, axes = plt.subplots(count_rows, columns, figsize=(5 * columns, 4 * count_rows),
                                    squeeze=False, constrained_layout=True)
        for number, axis in zip(targets, axes.flat):
            selected = sorted((row for row in rows if int(row["target"]) == number),
                              key=lambda row: float(row["phase_s"]))
            phases = [float(row["phase_s"]) for row in selected]
            if kind == "trajectory":
                axis.scatter([float(row["observed_e_m"]) / 1000 for row in selected],
                             [float(row["observed_n_m"]) / 1000 for row in selected],
                             s=7, alpha=0.6, label="Observed")
                axis.plot([float(row["predicted_e_m"]) / 1000 for row in selected],
                          [float(row["predicted_n_m"]) / 1000 for row in selected],
                          color="black", linewidth=1, label="Frozen model")
                axis.set_xlabel("East (km)")
                axis.set_ylabel("North (km)")
                axis.set_aspect("equal", adjustable="datalim")
            elif kind == "height":
                axis.scatter(phases, [float(row["observed_u_m"]) for row in selected],
                             s=7, alpha=0.6, label="Observed")
                axis.plot(phases, [float(row["predicted_u_m"]) for row in selected],
                          color="black", linewidth=1, label="Frozen model")
                axis.set_xlabel("Scenario phase (s)")
                axis.set_ylabel("Radar-relative up (m)")
            else:
                limit = summary["residual_summary"]["targets"][str(number)]["distance_tolerance_m"]
                axis.axhspan(-limit, limit, color="gray", alpha=0.2, label="Speed x 20ms")
                axis.scatter(phases, [float(row["range_m"]) for row in selected], s=7, alpha=0.6)
                axis.axhline(0, color="black", linewidth=0.7)
                axis.set_xlabel("Scenario phase (s)")
                axis.set_ylabel("Observed - model range (m)")
            axis.set_title(f"T{number} (n={len(selected)})")
            axis.grid(alpha=0.25)
            axis.legend(fontsize=8)
        for axis in list(axes.flat)[len(targets):]:
            axis.set_visible(False)
        figure.suptitle("Multiple targets; repeated scenario phases overlaid")
        for extension in ("png", "svg"):
            figure.savefig(destination / f"{kind}.{extension}", dpi=160)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        plot(args.analysis, args.output)
    except (ImportError, ValueError, OSError, KeyError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
