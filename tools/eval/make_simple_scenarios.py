"""Create CR-delimited one-target experiments without touching the source ini."""

import argparse
from dataclasses import replace
from pathlib import Path

from .scenario import load_scenario


def create_simple_scenarios(scenario, target_number, destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    target = scenario.target(target_number)
    information = (1, *scenario.information[1:])
    experiments = {"straight": None}
    for kind in range(1, 5):
        selected = next((m for m in target.maneuvers if m.kind == kind), None)
        if selected is None:
            raise ValueError(f"Source TARGET_{target_number} has no kind {kind}")
        experiments[f"kind_{kind}"] = replace(selected, start=10)
    for name, maneuver in experiments.items():
        values = list(target.values)
        values[8] = int(maneuver is not None)
        values[9] = 0
        rows = ["SCENARIO_INFORMATION", " ".join(f"{x:.15g}" for x in information), "", "TARGET_1",
                " ".join(f"{x:.15g}" for x in values), "", "MANEUVER_1"]
        if maneuver:
            rows.append(" ".join(f"{x:.15g}" for x in (maneuver.start, maneuver.kind, maneuver.acceleration, maneuver.goal)))
        path = destination / f"target_{target_number}_{name}.ini"
        # Refuse overwrite, including accidental selection of the original file.
        with path.open("xb") as stream:
            stream.write(("\r".join(rows) + "\r").encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ini", type=Path, required=True)
    parser.add_argument("--target", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        create_simple_scenarios(load_scenario(args.ini), args.target, args.output)
    except (OSError, ValueError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
