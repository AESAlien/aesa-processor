"""Trajectory hypotheses, never a claim of simulator-verified ground truth."""

from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path

from .geometry import (
    antenna_bearing, direct_sphere, dot, ecef_to_enu, enu_to_ecef,
    inverse_rotate, norm, rotate, sphere_position,
)
from .scenario import Scenario


@dataclass(frozen=True)
class ModelOptions:
    # Numeric scales also permit units not listed in the request (e.g. km/h).
    range_scale_m: float = 1000.0
    range_kind: str = "slant"                 # slant | ground
    altitude_scale_m: float = 1.0
    altitude_reference: str = "msl"           # msl | radar
    radar_altitude_scale_m: float = 1.0
    speed_scale_mps: float = 1.0
    speed_kind: str = "total"                 # total | horizontal
    azimuth_zero_deg: float = 0.0
    azimuth_sign: int = 1
    azimuth_frame: str = "enu"                # enu | antenna
    heading_zero_deg: float = 0.0
    heading_sign: int = 1
    heading_frame: str = "enu"                # enu | antenna (horizontal projection)
    vertical_kind: str = "angle"              # angle | rate
    vertical_angle_mode: str = "flight_path"  # flight_path | fixed_plane
    vertical_scale: float = 1.0               # degrees/unit or m/s/unit
    vertical_sign: int = 1
    output_frame: str = "enu"                 # enu | antenna
    output_azimuth_zero_deg: float = 0.0
    output_azimuth_sign: int = 1
    output_elevation_sign: int = 1
    doppler_sign: int = 1                     # +1 receding; -1 approaching
    attitude_enabled: bool = True
    attitude_scale_deg: float = 1.0
    attitude_order: tuple[int, ...] = (0, 1, 2)  # indices within last 3 scenario values
    attitude_signs: tuple[int, ...] = (1, 1, 1)
    attitude_offset_deg: tuple[float, ...] = (0.0, 0.0, 0.0)
    earth_model: str = "flat"                 # flat | sphere
    earth_radius_m: float = 6371000.0
    start_offset_s: float = 0.0               # elapsed = processor time - offset
    scenario_repeat_period_s: float | None = None  # optional global scenario-time wrap
    field10_role: str = "ignore"              # ignore | start_time
    field11_role: str = "ignore"              # ignore | end_time | duration
    target_time_scale_s: float = 1.0
    maneuver_time_scale_s: float = 1.0
    maneuver_clock: str = "target"            # target | scenario
    maneuver_actions: tuple[str, ...] = ("climb", "descend", "left", "right")
    turn_model: str = "lateral_acceleration"   # also load_factor | rate | rotating_acceleration | speed_coupled_rotation
    turn_acceleration_reference_deg: float | None = None  # null: maneuver-entry heading; otherwise fixed ENU
    turn_duration_step_s: float | None = None  # optional ceiling of nominal turn duration to physical ticks
    turn_reference_mode: str = "reset"         # reset | persistent (speed_coupled_rotation)
    turn_duration_speed: str = "entry"         # entry | initial (speed used for nominal duration)
    turn_reference_quantum_deg: float | None = None  # optional nearest-axis quantization of entry reference
    gravity_mps2: float = 9.80665
    acceleration_scale: float = 1.0
    turn_rate_deg_s: float = 3.0               # rate model ignores field 3
    turn_goal: str = "delta"                  # delta | absolute (ENU heading)
    turn_goal_scale_deg: float = 1.0
    vertical_model: str = "triangular"         # triangular | discrete_triangular | acceleration | rate
    vertical_update_step_s: float = 0.1        # physical update interval for discrete_triangular
    vertical_rate_mps: float = 10.0            # rate model ignores field 3
    vertical_goal: str = "delta"              # delta | absolute (altitude reference above)
    vertical_goal_scale_m: float = 0.3048
    post_maneuver: str = "tangent"            # tangent | initial | stop
    overlap_policy: str = "error"             # error | replace
    integration_step_s: float = 0.01

    def __post_init__(self):
        choices = {
            "range_kind": ("slant", "ground"), "altitude_reference": ("msl", "radar"),
            "speed_kind": ("total", "horizontal"), "azimuth_frame": ("enu", "antenna"),
            "heading_frame": ("enu", "antenna"), "vertical_kind": ("angle", "rate"),
            "vertical_angle_mode": ("flight_path", "fixed_plane"),
            "output_frame": ("enu", "antenna"), "earth_model": ("flat", "sphere"),
            "field10_role": ("ignore", "start_time"), "field11_role": ("ignore", "end_time", "duration"),
            "maneuver_clock": ("target", "scenario"),
            "turn_model": ("lateral_acceleration", "load_factor", "rate", "rotating_acceleration", "speed_coupled_rotation"),
            "turn_goal": ("delta", "absolute"), "vertical_model": ("triangular", "discrete_triangular", "acceleration", "rate"),
            "turn_reference_mode": ("reset", "persistent"), "turn_duration_speed": ("entry", "initial"),
            "vertical_goal": ("delta", "absolute"), "post_maneuver": ("tangent", "initial", "stop"),
            "overlap_policy": ("error", "replace"),
        }
        for name, allowed in choices.items():
            if getattr(self, name) not in allowed:
                raise ValueError(f"{name}: choose from {allowed}")
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f"{item.name} must be finite")
        for name in ("range_scale_m", "altitude_scale_m", "radar_altitude_scale_m", "speed_scale_mps",
                     "earth_radius_m", "target_time_scale_s", "maneuver_time_scale_s", "gravity_mps2",
                     "acceleration_scale", "turn_rate_deg_s", "turn_goal_scale_deg", "vertical_rate_mps",
                     "vertical_goal_scale_m", "integration_step_s", "vertical_update_step_s", "vertical_scale", "attitude_scale_deg"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("azimuth_sign", "heading_sign", "vertical_sign", "output_azimuth_sign", "output_elevation_sign", "doppler_sign"):
            if getattr(self, name) not in (-1, 1):
                raise ValueError(f"{name} must be -1 or 1")
        if self.scenario_repeat_period_s is not None and (
                not math.isfinite(self.scenario_repeat_period_s) or self.scenario_repeat_period_s <= 0):
            raise ValueError("scenario_repeat_period_s must be positive or null")
        if self.turn_duration_step_s is not None and self.turn_duration_step_s <= 0:
            raise ValueError("turn_duration_step_s must be positive or null")
        if self.turn_reference_quantum_deg is not None and self.turn_reference_quantum_deg <= 0:
            raise ValueError("turn_reference_quantum_deg must be positive or null")
        if (sorted(self.attitude_order) != [0, 1, 2]
                or any(type(x) is not int for x in self.attitude_order)
                or len(self.attitude_signs) != 3
                or any(x not in (-1, 1) for x in self.attitude_signs)):
            raise ValueError("Invalid attitude permutation or signs")
        if len(self.attitude_offset_deg) != 3 or not all(math.isfinite(x) for x in self.attitude_offset_deg):
            raise ValueError("attitude_offset_deg must have 3 finite angles")
        if len(self.maneuver_actions) != 4 or any(x not in ("climb", "descend", "left", "right", "ignore") for x in self.maneuver_actions):
            raise ValueError("maneuver_actions must map kinds 1..4 to climb/descend/left/right/ignore")

    @classmethod
    def load(cls, path: str | Path):
        values = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if not isinstance(values, dict):
            raise ValueError("Options JSON must be an object")
        unknown = set(values) - {item.name for item in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown model options: {sorted(unknown)}")
        return cls(**values)


@dataclass(frozen=True)
class Prediction:
    target: int
    processor_time_s: float
    elapsed_s: float
    active: bool
    maneuver_kind: int | None
    position_enu_m: tuple[float, ...]
    velocity_enu_mps: tuple[float, ...]
    slant_range_m: float
    azimuth_deg: float
    elevation_deg: float
    doppler_velocity_mps: float
    model_status: str = "unverified_hypothesis"


@dataclass
class _State:
    position: tuple[float, ...]  # flat: E,N,MSL altitude; sphere: lat(rad),lon(rad),MSL altitude
    heading: float
    vertical: float
    speed: float
    rotation_reference: float | None = None


@dataclass
class _Motion:
    kind: int
    start: float
    duration: float
    turn_rate: float = 0.0
    acceleration: float = 0.0
    triangular: bool = False
    final_heading: float | None = None
    final_altitude: float | None = None
    final_vertical: float | None = None
    entry_state: _State | None = None
    acceleration_reference: float | None = None
    next_vertical_update_s: float | None = None
    rotation_angle: float | None = None


class TruthCalculator:
    def __init__(self, scenario: Scenario, options: ModelOptions | None = None):
        self.scenario = scenario
        self.options = options or ModelOptions()
        o = self.options
        info = scenario.information
        if not -90 <= info[1] <= 90 or not -180 <= info[2] <= 180:
            raise ValueError("Scenario latitude/longitude out of range under selected interpretation")
        self.latitude, self.longitude = map(math.radians, info[1:3])
        self.radar_altitude = info[3] * o.radar_altitude_scale_m
        self.attitude = tuple(info[4 + index] * o.attitude_scale_deg * sign + offset for index, sign, offset in zip(o.attitude_order, o.attitude_signs, o.attitude_offset_deg)) if o.attitude_enabled else (0, 0, 0)

    def _horizontal_speed(self, state):
        if self.options.speed_kind == "horizontal":
            return state.speed
        if abs(state.vertical) > state.speed + 1e-9:
            raise ValueError("Vertical speed exceeds total speed; try horizontal speed or another vertical model")
        return math.sqrt(max(0, state.speed ** 2 - state.vertical ** 2))

    def _initial(self, target):
        o, values = self.options, target.values
        distance, speed = values[0] * o.range_scale_m, values[3] * o.speed_scale_mps
        altitude = values[2] * o.altitude_scale_m + (self.radar_altitude if o.altitude_reference == "radar" else 0)
        if distance <= 0 or speed < 0:
            raise ValueError("Initial distance must be positive and speed nonnegative")
        up = altitude - self.radar_altitude
        if o.earth_model == "flat":
            if o.range_kind == "slant" and distance < abs(up):
                raise ValueError("Slant range is smaller than altitude difference")
            horizontal = math.sqrt(max(0, distance ** 2 - up ** 2)) if o.range_kind == "slant" else distance
        else:
            r0, r1 = o.earth_radius_m + self.radar_altitude, o.earth_radius_m + altitude
            if min(r0, r1) <= 0:
                raise ValueError("Altitude is below sphere center")
            if o.range_kind == "slant":
                if not abs(r1 - r0) <= distance <= r0 + r1:
                    raise ValueError("Impossible spherical slant range")
                # Stable half-angle form avoids subtracting nearly equal radii squared.
                angle = 2 * math.asin(math.sqrt(max(0, (distance ** 2 - (r1 - r0) ** 2) / (4 * r0 * r1))))
            else:
                angle = distance / o.earth_radius_m
                if angle >= math.pi:
                    raise ValueError("Ground arc must be shorter than half the sphere")
            horizontal, up = r1 * math.sin(angle), r1 * math.cos(angle) - r0
        bearing = math.radians(o.azimuth_zero_deg + o.azimuth_sign * values[1])
        if o.azimuth_frame == "antenna":
            bearing = antenna_bearing(bearing, horizontal, up, self.attitude)
        if o.earth_model == "flat":
            position = (horizontal * math.sin(bearing), horizontal * math.cos(bearing), altitude)
        else:
            lat, lon = direct_sphere(self.latitude, self.longitude, bearing, angle)
            position = (lat, lon, altitude)
        heading = math.radians(o.heading_zero_deg + o.heading_sign * values[4])
        if o.heading_frame == "antenna":
            direction = rotate((math.sin(heading), math.cos(heading), 0), self.attitude)
            if math.hypot(*direction[:2]) < 1e-12:
                raise ValueError("Heading projects vertically")
            heading = math.atan2(direction[0], direction[1])
        vertical = o.vertical_sign * values[5] * o.vertical_scale
        if o.vertical_kind == "angle":
            angle_vertical = math.radians(vertical)
            if abs(angle_vertical) >= math.pi / 2:
                raise ValueError("Initial flight path angle must be between -90 and 90 degrees")
            vertical = speed * (math.sin(angle_vertical) if o.speed_kind == "total" else math.tan(angle_vertical))
            if o.vertical_angle_mode == "fixed_plane":
                if o.earth_model != "flat":
                    raise ValueError("Fixed-plane angle currently requires flat earth")
                if angle_vertical and any(o.maneuver_actions[m.kind - 1] in ("climb", "descend") for m in target.maneuvers):
                    raise ValueError("Nonzero fixed-plane angle with altitude maneuvers is unsupported")
                vertical = 0.0
        state = _State(position, heading, vertical, speed)
        self._horizontal_speed(state)
        return state

    def _motion(self, maneuver, time, state, initial_horizontal_speed=None):
        o = self.options
        action = o.maneuver_actions[maneuver.kind - 1]
        if action == "ignore":
            return None
        if action in ("left", "right"):
            sign = 1 if action == "right" else -1
            goal = math.radians(maneuver.goal * o.turn_goal_scale_deg)
            delta = abs(goal) if o.turn_goal == "delta" else (sign * (goal - state.heading)) % (2 * math.pi)
            if delta < 1e-12:
                return None
            if o.turn_model == "rate":
                rate = math.radians(o.turn_rate_deg_s)
            else:
                acceleration = maneuver.acceleration * o.acceleration_scale
                if o.turn_model == "load_factor":
                    if acceleration <= 1:
                        raise ValueError("Coordinated level-turn load factor must exceed 1")
                    acceleration = math.sqrt(acceleration ** 2 - 1)
                horizontal = self._horizontal_speed(state)
                if acceleration <= 0 or horizontal <= 0:
                    raise ValueError("Turn requires positive acceleration and horizontal speed")
                rate = acceleration * o.gravity_mps2 / horizontal
            duration = delta / rate
            if o.turn_duration_speed == "initial":
                if initial_horizontal_speed is None or min(initial_horizontal_speed, self._horizontal_speed(state)) <= 0:
                    raise ValueError("Initial-speed turn duration requires positive initial and entry horizontal speed")
                duration *= initial_horizontal_speed / self._horizontal_speed(state)
            if o.turn_duration_step_s is not None:
                duration = math.ceil(duration / o.turn_duration_step_s - 1e-12) * o.turn_duration_step_s
            if o.turn_model in ("rotating_acceleration", "speed_coupled_rotation"):
                if o.earth_model != "flat":
                    raise ValueError("Rotating acceleration is supported only in flat ENU")
                reference = state.heading if o.turn_acceleration_reference_deg is None else math.radians(o.turn_acceleration_reference_deg)
                if o.turn_reference_quantum_deg is not None:
                    quantum = math.radians(o.turn_reference_quantum_deg)
                    reference = math.floor(reference / quantum + 0.5) * quantum
                if o.turn_model == "speed_coupled_rotation":
                    if o.turn_reference_mode == "persistent" and state.rotation_reference is not None:
                        reference = state.rotation_reference
                    return _Motion(maneuver.kind, time, duration,
                                   acceleration=sign * rate * self._horizontal_speed(state), rotation_angle=reference)
                entry = _State(state.position, state.heading, state.vertical, state.speed)
                return _Motion(maneuver.kind, time, duration, sign * rate,
                               entry_state=entry, acceleration_reference=reference)
            return _Motion(maneuver.kind, time, duration, sign * rate, final_heading=state.heading + sign * rate * duration)
        sign = 1 if action == "climb" else -1
        goal = maneuver.goal * o.vertical_goal_scale_m
        delta = sign * abs(goal) if o.vertical_goal == "delta" else goal + (self.radar_altitude if o.altitude_reference == "radar" else 0) - state.position[2]
        if abs(delta) < 1e-12:
            return None
        if delta * sign < 0:
            raise ValueError("Absolute altitude goal conflicts with selected climb/descend kind")
        final_altitude = state.position[2] + delta
        if o.vertical_model == "rate":
            state.vertical = sign * o.vertical_rate_mps
            return _Motion(maneuver.kind, time, abs(delta) / o.vertical_rate_mps, final_altitude=final_altitude)
        acceleration = maneuver.acceleration * o.acceleration_scale * o.gravity_mps2
        if acceleration <= 0:
            raise ValueError("Vertical maneuver requires positive acceleration")
        triangular = o.vertical_model in ("triangular", "discrete_triangular")
        factor = 4 if triangular else 2
        duration = factor * (-sign * state.vertical + math.sqrt(state.vertical ** 2 + 4 * acceleration * abs(delta) / factor)) / (2 * acceleration)
        peak = state.vertical + sign * acceleration * duration / (2 if triangular else 1)
        if o.speed_kind == "total" and max(abs(peak), abs(state.vertical)) > state.speed:
            raise ValueError("Vertical maneuver would exceed total speed")
        if o.vertical_model == "discrete_triangular":
            state.vertical += sign * acceleration * o.vertical_update_step_s
            self._horizontal_speed(state)
            return _Motion(maneuver.kind, time, duration, acceleration=sign * acceleration,
                           next_vertical_update_s=time + o.vertical_update_step_s)
        return _Motion(maneuver.kind, time, duration, acceleration=sign * acceleration, triangular=triangular,
                       final_altitude=final_altitude, final_vertical=state.vertical if triangular else peak)

    def _advance_rotating_acceleration(self, state, motion, elapsed):
        entry = motion.entry_state
        horizontal = self._horizontal_speed(entry)
        reference, rate = motion.acceleration_reference, motion.turn_rate
        angle = reference + rate * elapsed
        east_delta = horizontal * (math.sin(angle) - math.sin(reference))
        north_delta = horizontal * (math.cos(angle) - math.cos(reference))
        east_velocity = horizontal * math.sin(entry.heading)
        north_velocity = horizontal * math.cos(entry.heading)
        east, north, altitude = entry.position
        state.position = (
            east + east_velocity * elapsed + horizontal * ((math.cos(reference) - math.cos(angle)) / rate - math.sin(reference) * elapsed),
            north + north_velocity * elapsed + horizontal * ((math.sin(angle) - math.sin(reference)) / rate - math.cos(reference) * elapsed),
            altitude + entry.vertical * elapsed,
        )
        east_velocity += east_delta
        north_velocity += north_delta
        state.heading = math.atan2(east_velocity, north_velocity)
        state.speed = math.hypot(east_velocity, north_velocity)
        if self.options.speed_kind == "total":
            state.speed = math.hypot(state.speed, entry.vertical)
        state.vertical = entry.vertical

    def _advance_speed_coupled_rotation(self, state, motion, duration):
        # The acceleration direction rotates independently of the velocity heading.
        # RK4 integrates d(reference)/dt = signed acceleration / instantaneous horizontal speed.
        horizontal = self._horizontal_speed(state)
        values = (*state.position[:2], horizontal * math.sin(state.heading),
                  horizontal * math.cos(state.heading), motion.rotation_angle)

        def derivative(point):
            _, _, east_velocity, north_velocity, reference = point
            speed = math.hypot(east_velocity, north_velocity)
            if speed < 1e-9:
                raise ValueError("Speed-coupled turn reached zero horizontal speed")
            return (east_velocity, north_velocity, motion.acceleration * math.cos(reference),
                    -motion.acceleration * math.sin(reference), motion.acceleration / speed)

        first = derivative(values)
        second = derivative(tuple(x + duration * k / 2 for x, k in zip(values, first)))
        third = derivative(tuple(x + duration * k / 2 for x, k in zip(values, second)))
        fourth = derivative(tuple(x + duration * k for x, k in zip(values, third)))
        east, north, east_velocity, north_velocity, motion.rotation_angle = (
            x + duration * (a + 2 * b + 2 * c + d) / 6
            for x, a, b, c, d in zip(values, first, second, third, fourth)
        )
        state.position = (east, north, state.position[2] + state.vertical * duration)
        state.heading = math.atan2(east_velocity, north_velocity)
        state.speed = math.hypot(east_velocity, north_velocity)
        state.rotation_reference = motion.rotation_angle
        if self.options.speed_kind == "total":
            state.speed = math.hypot(state.speed, state.vertical)

    def _advance(self, state, duration, turn_rate, acceleration):
        # Midpoint integration; exact horizontal circular arcs in the flat model.
        o = self.options
        mid_vertical = state.vertical + acceleration * duration / 2
        mid = _State(state.position, state.heading + turn_rate * duration / 2, mid_vertical, state.speed)
        horizontal = self._horizontal_speed(mid)
        heading_delta = turn_rate * duration
        if o.earth_model == "flat":
            x, y, altitude = state.position
            scale = math.sin(heading_delta / 2) / (heading_delta / 2) if heading_delta else 1
            state.position = (x + horizontal * duration * math.sin(mid.heading) * scale,
                              y + horizontal * duration * math.cos(mid.heading) * scale,
                              altitude + mid_vertical * duration)
        else:
            lat, lon, altitude = state.position
            radius = o.earth_radius_m + altitude + mid_vertical * duration / 2
            if radius <= 0:
                raise ValueError("Trajectory passed below sphere center")
            mid_lat = lat + horizontal * math.cos(state.heading) * duration / (2 * radius)
            if abs(mid_lat) >= math.pi / 2 or abs(math.cos(mid_lat)) < 1e-10:
                raise ValueError("Spherical integration across a pole is unsupported")
            state.position = (lat + horizontal * math.cos(mid.heading) * duration / radius,
                              lon + horizontal * math.sin(mid.heading) * duration / (radius * math.cos(mid_lat)),
                              altitude + mid_vertical * duration)
        state.heading += heading_delta
        state.vertical += acceleration * duration
        self._horizontal_speed(state)

    def predict(self, target_number: int, processor_time_s: float) -> Prediction:
        if not math.isfinite(processor_time_s):
            raise ValueError("Time must be finite")
        target, o = self.scenario.target(target_number), self.options
        field_start = target.values[9] * o.target_time_scale_s if o.field10_role == "start_time" else 0
        scenario_time = processor_time_s - o.start_offset_s
        if o.scenario_repeat_period_s is not None and scenario_time >= 0:
            scenario_time %= o.scenario_repeat_period_s
        elapsed = scenario_time - field_start
        end = math.inf
        if o.field11_role != "ignore":
            end = target.values[10] * o.target_time_scale_s
            if o.field11_role == "end_time":
                end -= field_start
            if end < 0:
                raise ValueError("Target ends before it starts")
        active = 0 <= elapsed <= end
        horizon = max(0, min(elapsed, end))
        state = self._initial(target)
        initial_horizontal_speed = self._horizontal_speed(state)
        initial_position = state.position
        initial_heading, initial_vertical = state.heading, state.vertical
        schedule = [
            (m.start * o.maneuver_time_scale_s - (field_start if o.maneuver_clock == "scenario" else 0), m)
            for m in target.maneuvers if o.maneuver_actions[m.kind - 1] != "ignore"
        ]
        if any(t < 0 for t, _ in schedule):
            raise ValueError("Maneuver precedes target start under selected clock")
        index, time, motion = 0, 0.0, None
        while True:
            if motion and time >= motion.start + motion.duration - 1e-10:
                if motion.final_heading is not None:
                    state.heading = motion.final_heading
                if motion.final_altitude is not None:
                    state.position = (*state.position[:2], motion.final_altitude)
                if motion.final_vertical is not None:
                    state.vertical = motion.final_vertical
                if o.post_maneuver == "initial":
                    state.heading, state.vertical = initial_heading, initial_vertical
                elif o.post_maneuver == "stop":
                    state.speed, state.vertical = 0, 0
                motion = None
            while index < len(schedule) and schedule[index][0] <= time + 1e-10:
                if motion and o.overlap_policy == "error":
                    raise ValueError(f"TARGET_{target_number}: overlapping maneuvers at {time:g}s")
                motion = self._motion(schedule[index][1], time, state, initial_horizontal_speed)
                index += 1
            if motion and motion.next_vertical_update_s is not None:
                while motion.next_vertical_update_s <= time + 1e-10:
                    phase = motion.next_vertical_update_s - motion.start
                    acceleration = motion.acceleration if phase < motion.duration / 2 - 1e-10 else -motion.acceleration
                    state.vertical += acceleration * o.vertical_update_step_s
                    self._horizontal_speed(state)
                    motion.next_vertical_update_s += o.vertical_update_step_s
            if time >= horizon - 1e-10:
                break
            boundary = min(horizon, time + o.integration_step_s)
            if index < len(schedule):
                boundary = min(boundary, schedule[index][0])
            turn_rate, acceleration = 0.0, 0.0
            if motion:
                boundary = min(boundary, motion.start + motion.duration)
                turn_rate, acceleration = motion.turn_rate, motion.acceleration
                if motion.next_vertical_update_s is not None:
                    boundary = min(boundary, motion.next_vertical_update_s)
                    acceleration = 0.0
                if motion.triangular:
                    midpoint = motion.start + motion.duration / 2
                    if time < midpoint - 1e-10:
                        boundary = min(boundary, midpoint)
                    else:
                        acceleration = -acceleration
            if boundary <= time:
                raise ValueError("Integration step too small for requested time")
            if motion and motion.rotation_angle is not None:
                self._advance_speed_coupled_rotation(state, motion, boundary - time)
            elif motion and motion.entry_state is not None:
                self._advance_rotating_acceleration(state, motion, boundary - motion.start)
            else:
                self._advance(state, boundary - time, turn_rate, acceleration)
            time = boundary
        horizontal = self._horizontal_speed(state)
        local_velocity = (horizontal * math.sin(state.heading), horizontal * math.cos(state.heading), state.vertical)
        if o.earth_model == "flat":
            position = (*state.position[:2], state.position[2] - self.radar_altitude)
            velocity = local_velocity
            if o.vertical_kind == "angle" and o.vertical_angle_mode == "fixed_plane":
                angle = math.radians(target.values[5] * o.vertical_scale * o.vertical_sign)
                direction = (math.sin(initial_heading), math.cos(initial_heading))
                displacement = sum((state.position[i] - initial_position[i]) * direction[i] for i in range(2))
                along_velocity = sum(velocity[i] * direction[i] for i in range(2))
                position = (position[0] + displacement * (math.cos(angle) - 1) * direction[0],
                            position[1] + displacement * (math.cos(angle) - 1) * direction[1],
                            position[2] + displacement * math.sin(angle))
                velocity = (velocity[0] + along_velocity * (math.cos(angle) - 1) * direction[0],
                            velocity[1] + along_velocity * (math.cos(angle) - 1) * direction[1],
                            velocity[2] + along_velocity * math.sin(angle))
        else:
            lat, lon, altitude = state.position
            target_ecef = sphere_position(lat, lon, o.earth_radius_m + altitude)
            radar_ecef = sphere_position(self.latitude, self.longitude, o.earth_radius_m + self.radar_altitude)
            position = ecef_to_enu(tuple(a - b for a, b in zip(target_ecef, radar_ecef)), self.latitude, self.longitude)
            velocity = ecef_to_enu(enu_to_ecef(local_velocity, lat, lon), self.latitude, self.longitude)
        if not active:
            velocity = (0.0, 0.0, 0.0)
        measured = inverse_rotate(position, self.attitude) if o.output_frame == "antenna" else position
        distance = norm(position)
        if distance < 1e-12:
            raise ValueError("Radar and target coincide: bearing and Doppler undefined")
        azimuth = o.output_azimuth_sign * (math.degrees(math.atan2(measured[0], measured[1])) - o.output_azimuth_zero_deg)
        return Prediction(target_number, processor_time_s, elapsed, active, motion.kind if motion and active else None,
                          position, velocity, distance, (azimuth + 180) % 360 - 180,
                          o.output_elevation_sign * math.degrees(math.atan2(measured[2], math.hypot(*measured[:2]))),
                          o.doppler_sign * dot(position, velocity) / distance)

    def describe(self):
        return {"model_status": "unverified_hypothesis", "source": self.scenario.source, "options": asdict(self.options)}
