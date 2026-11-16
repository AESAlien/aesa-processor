"""Read the simulator's whitespace blocks, including CR-only files."""

from dataclasses import dataclass
import math
from pathlib import Path
import re


@dataclass(frozen=True)
class Maneuver:
    start: float
    kind: int
    acceleration: float
    goal: float


@dataclass(frozen=True)
class Target:
    number: int
    values: tuple[float, ...]
    maneuvers: tuple[Maneuver, ...]


@dataclass(frozen=True)
class Scenario:
    information: tuple[float, ...]
    targets: tuple[Target, ...]
    source: str

    def target(self, number: int) -> Target:
        for target in self.targets:
            if target.number == number:
                return target
        raise ValueError(f"Unknown target: {number}")


def _integer(value: float, context: str) -> int:
    if not value.is_integer():
        raise ValueError(f"{context}: expected an integer, got {value}")
    return int(value)


def parse_scenario(text: str, source: str = "<text>") -> Scenario:
    blocks: dict[str, list[tuple[int, tuple[float, ...]]]] = {}
    current = None
    for line_number, line in enumerate(text.lstrip("\ufeff").splitlines(), 1):
        line = line.split(";", 1)[0].split("#", 1)[0].strip()
        if not line:
            continue
        name = line.strip("[]").strip()
        if re.fullmatch(r"SCENARIO_INFORMATION|(?:TARGET|MANEUVER)_[1-9]\d*", name):
            if name in blocks:
                raise ValueError(f"Line {line_number}: duplicate block {name}")
            current = name
            blocks[name] = []
            continue
        if current is None:
            raise ValueError(f"Line {line_number}: values before a block header")
        try:
            values = tuple(float(value) for value in line.split())
        except ValueError as error:
            raise ValueError(f"Line {line_number}: invalid header or number") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Line {line_number}: non-finite number")
        blocks[current].append((line_number, values))

    information = tuple(value for _, row in blocks.get("SCENARIO_INFORMATION", []) for value in row)
    if len(information) != 7:
        raise ValueError("SCENARIO_INFORMATION must contain exactly 7 values")
    count = _integer(information[0], "Scenario target count")
    if count < 1:
        raise ValueError("Scenario target count must be positive")
    expected = {"SCENARIO_INFORMATION"}
    targets = []
    for number in range(1, count + 1):
        target_name, maneuver_name = f"TARGET_{number}", f"MANEUVER_{number}"
        expected.update((target_name, maneuver_name))
        rows = blocks.get(target_name, [])
        if len(rows) != 1 or len(rows[0][1]) != 11:
            raise ValueError(f"{target_name} must have one row with 11 values")
        if maneuver_name not in blocks:
            raise ValueError(f"Missing block: {maneuver_name}")
        values = rows[0][1]
        maneuver_count = _integer(values[8], f"{target_name} field 9")
        maneuver_rows = blocks[maneuver_name]
        if maneuver_count < 0 or maneuver_count != len(maneuver_rows):
            raise ValueError(f"{maneuver_name}: row count differs from TARGET field 9")
        maneuvers = []
        for line_number, row in maneuver_rows:
            if len(row) != 4:
                raise ValueError(f"Line {line_number}: maneuver must have 4 values")
            kind = _integer(row[1], f"Line {line_number} maneuver kind")
            if row[0] < 0 or kind not in (1, 2, 3, 4):
                raise ValueError(f"Line {line_number}: invalid start time or maneuver kind")
            maneuvers.append(Maneuver(row[0], kind, row[2], row[3]))
        if any(a.start > b.start for a, b in zip(maneuvers, maneuvers[1:])):
            raise ValueError(f"{maneuver_name}: start times must be ordered")
        targets.append(Target(number, values, tuple(maneuvers)))
    if set(blocks) != expected:
        raise ValueError(f"Unexpected blocks: {sorted(set(blocks) - expected)}")
    return Scenario(information, tuple(targets), source)


def load_scenario(path: str | Path, encoding: str = "utf-8-sig") -> Scenario:
    path = Path(path).resolve()
    return parse_scenario(path.read_bytes().decode(encoding), str(path))
