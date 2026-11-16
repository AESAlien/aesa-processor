"""Run from repository root: python -m tools.eval --help."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from .scenario import load_scenario
from .truth import ModelOptions, TruthCalculator


def main(argv=None):
    parser = argparse.ArgumentParser(description="Simulator trajectory hypotheses (full validation is incomplete)")
    commands = parser.add_subparsers(dest="command", required=True)
    defaults = commands.add_parser("options", help="Print the complete, unverified default options JSON")
    defaults.add_argument("--output", type=Path)
    inspect = commands.add_parser("inspect", help="Validate and print scenario blocks, preserving raw values")
    predict = commands.add_parser("predict", help="Query position, velocity, radar measurements at processor times")
    for command in (inspect, predict):
        command.add_argument("--ini", type=Path, required=True)
        command.add_argument("--encoding", default="utf-8-sig")
        command.add_argument("--output", type=Path)
    predict.add_argument("--options", type=Path)
    predict.add_argument("--target", type=int, action="append", help="Repeat for several targets; default: all")
    predict.add_argument("--time", type=float, action="append", required=True, help="Processor transmitTime_ms / 1000; repeatable")
    args = parser.parse_args(argv)
    try:
        if args.command == "options":
            result = asdict(ModelOptions())
        else:
            scenario = load_scenario(args.ini, args.encoding)
            if args.command == "inspect":
                result = asdict(scenario)
            else:
                print("WARNING: unverified candidate; full simulator validation criteria have not been met.", file=sys.stderr)
                options = ModelOptions.load(args.options) if args.options else ModelOptions()
                calculator = TruthCalculator(scenario, options)
                targets = args.target if args.target is not None else [target.number for target in scenario.targets]
                result = calculator.describe()
                result["predictions"] = [asdict(calculator.predict(target, time)) for time in args.time for target in targets]
        output = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.write_text(output, encoding="utf-8")
        else:
            print(output, end="")
    except (OSError, ValueError, TypeError, OverflowError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
