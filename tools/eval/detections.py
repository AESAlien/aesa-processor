"""Read detection events and associate plots by spatial/time continuity."""

import csv
from dataclasses import dataclass, field
import math
from pathlib import Path

from .geometry import norm


@dataclass(frozen=True)
class Plot:
    row: int
    beam_type: str
    request_id: int
    time_ms: int
    index: int
    slant_range: float
    azimuth: float
    elevation: float
    doppler: float

    @property
    def position(self):
        azimuth, elevation = math.radians(self.azimuth), math.radians(self.elevation)
        horizontal = self.slant_range * math.cos(elevation)
        return (horizontal * math.sin(azimuth), horizontal * math.cos(azimuth), self.slant_range * math.sin(elevation))


@dataclass(frozen=True)
class Event:
    time_ms: int
    plots: tuple[Plot, ...]


def load_events(path, start_ms=None, end_ms=None):
    """Validate complete event groups; never silently consume a partial CSV row."""
    events = []
    key, group, expected = None, [], None
    previous_time = None

    def finish():
        if key is None:
            return
        if len(group) != expected or [plot.index for plot in group] != list(range(expected)):
            raise ValueError(f"Incomplete or duplicate plots for event {key}")
        events.append(Event(key[2], tuple(group)))

    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"beamType", "requestId", "transmitTime_ms", "plotCount", "plotIndex", "slantRange_m",
                    "azimuth_deg", "elevation_deg", "dopplerVelocity_mps", "power_db"}
        if set(reader.fieldnames or []) != required:
            raise ValueError("CSV header does not match detection logger schema")
        for row_number, row in enumerate(reader, 2):
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("Incomplete CSV row")
                time_ms, count = int(row["transmitTime_ms"]), int(row["plotCount"])
                if time_ms < 0 or count < 0:
                    raise ValueError("Negative time or count")
                if start_ms is not None and time_ms < start_ms or end_ms is not None and time_ms > end_ms:
                    continue
                if previous_time is not None and time_ms < previous_time:
                    raise ValueError("Time decreases: select a single processor run")
                previous_time = time_ms
                next_key = (row["beamType"], int(row["requestId"]), time_ms)
                if next_key != key:
                    finish()
                    key, group, expected = next_key, [], count
                elif expected != count or count == 0:
                    raise ValueError("Inconsistent or duplicate event")
                if count == 0:
                    if any(row[name] for name in ("plotIndex", "slantRange_m", "azimuth_deg", "elevation_deg",
                                                 "dopplerVelocity_mps", "power_db")):
                        raise ValueError("Empty event has plot data")
                    continue
                values = tuple(float(row[name]) for name in ("slantRange_m", "azimuth_deg", "elevation_deg", "dopplerVelocity_mps", "power_db"))
                if not all(math.isfinite(value) for value in values) or values[0] <= 0:
                    raise ValueError("Invalid plot measurement")
                group.append(Plot(row_number, key[0], key[1], time_ms, int(row["plotIndex"]), *values[:4]))
            except (ValueError, TypeError) as error:
                raise ValueError(f"CSV row {row_number}: {error}") from error
        finish()
    if not events:
        raise ValueError("No events in selected time interval")
    return events


@dataclass
class PlotTrack:
    number: int
    plots: list[Plot] = field(default_factory=list)
    ambiguous_rows: list[int] = field(default_factory=list)

    def velocity(self):
        last = self.plots[-1]
        # Use recent, distinct times; several beams can observe the same target.
        recent = [plot for plot in self.plots[-12:] if 0.05 <= (last.time_ms - plot.time_ms) / 1000 <= 2.5]
        if not recent:
            return None
        first = recent[0]
        dt = (last.time_ms - first.time_ms) / 1000
        return tuple((b - a) / dt for a, b in zip(first.position, last.position))


def associate(events, max_gap_s=6.0, max_speed_mps=2500.0, position_gate_m=200.0, acceleration_gate_mps2=120.0):
    """Constant-velocity gated greedy association, independent of ini hypotheses.

    One plot per continuity track per event. Competing compatible tracks/plots
    are marked as ambiguous; a continuity-track number is NOT a target ID.
    """
    tracks = []
    for event in events:
        if not event.plots:
            continue
        candidates = []
        for track in tracks:
            last = track.plots[-1]
            dt = (event.time_ms - last.time_ms) / 1000
            if not 0 <= dt <= max_gap_s:
                continue
            velocity = track.velocity()
            predicted = tuple(x + (v * dt if velocity else 0) for x, v in zip(last.position, velocity or (0, 0, 0)))
            gate = position_gate_m + (max_speed_mps * dt if velocity is None else 0.5 * acceleration_gate_mps2 * dt * dt)
            for index, plot in enumerate(event.plots):
                distance = norm(tuple(a - b for a, b in zip(plot.position, predicted)))
                physical = norm(tuple(a - b for a, b in zip(plot.position, last.position)))
                if distance <= gate and physical <= position_gate_m + max_speed_mps * dt:
                    candidates.append((distance, track.number, index))
        used_tracks, used_plots = set(), set()
        for distance, number, index in sorted(candidates):
            if number in used_tracks or index in used_plots:
                continue
            track = tracks[number - 1]
            competing = [item for item in candidates if item[1] == number and item[2] != index or item[2] == index and item[1] != number]
            if any(item[0] <= distance * 1.5 + position_gate_m / 4 for item in competing):
                track.ambiguous_rows.append(event.plots[index].row)
            track.plots.append(event.plots[index])
            used_tracks.add(number)
            used_plots.add(index)
        for index, plot in enumerate(event.plots):
            if index not in used_plots:
                tracks.append(PlotTrack(len(tracks) + 1, [plot]))
    return tracks
