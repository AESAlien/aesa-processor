import math
import unittest

from tools.eval.geometry import dot, norm
from tools.eval.scenario import parse_scenario
from tools.eval.truth import ModelOptions, TruthCalculator


def calculator(values="10 0 100 100 90 0 1 10 0 1 100", maneuvers="", info="1 38 127 100 0 0 0", **options):
    scenario = parse_scenario(f"SCENARIO_INFORMATION\n{info}\nTARGET_1\n{values}\nMANEUVER_1\n{maneuvers}")
    return TruthCalculator(scenario, ModelOptions(**options))


class TruthTest(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected, places=6):
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=places)

    def test_straight_line_and_radial_velocity(self):
        result = calculator().predict(1, 2)
        self.assertVectorAlmostEqual(result.position_enu_m, (200, 10000, 0))
        self.assertVectorAlmostEqual(result.velocity_enu_mps, (100, 0, 0))
        self.assertAlmostEqual(result.slant_range_m, math.hypot(200, 10000))
        self.assertAlmostEqual(result.doppler_velocity_mps, 20000 / math.hypot(200, 10000))
        self.assertEqual(result.model_status, "unverified_hypothesis")

    def test_unit_altitude_and_ground_range_options(self):
        result = calculator("1 90 1000 10 0 60 1 10 0 1 100", range_scale_m=1852, range_kind="ground",
                            altitude_scale_m=0.3048, altitude_reference="radar", speed_scale_mps=1852 / 3600,
                            vertical_kind="rate", vertical_scale=0.3048 / 60, speed_kind="horizontal").predict(1, 0)
        self.assertVectorAlmostEqual(result.position_enu_m, (1852, 0, 304.8))
        self.assertVectorAlmostEqual(result.velocity_enu_mps, (0, 10 * 1852 / 3600, 0.3048))

    def test_start_offset_and_field_lifetimes(self):
        calc = calculator(start_offset_s=50, field10_role="start_time", field11_role="duration")
        self.assertFalse(calc.predict(1, 50).active)
        self.assertVectorAlmostEqual(calc.predict(1, 50).velocity_enu_mps, (0, 0, 0))
        self.assertVectorAlmostEqual(calc.predict(1, 53).position_enu_m, (200, 10000, 0))
        self.assertFalse(calc.predict(1, 152).active)
        self.assertVectorAlmostEqual(calc.predict(1, 152).position_enu_m, (10000, 10000, 0))
        calc = calculator(field10_role="start_time", field11_role="end_time")
        self.assertTrue(calc.predict(1, 100).active)
        self.assertFalse(calc.predict(1, 100.001).active)

    def test_turn_arc_event_boundaries_and_tangent_continuation(self):
        calc = calculator("10 0 100 100 0 0 1 10 1 1 100", "1 4 1 90", turn_model="rate", turn_rate_deg_s=90,
                          integration_step_s=0.13)
        radius = 100 / (math.pi / 2)
        middle = calc.predict(1, 1.5)
        self.assertVectorAlmostEqual(middle.position_enu_m, (radius * (1 - math.cos(math.pi / 4)),
                                                          10100 + radius * math.sin(math.pi / 4), 0))
        final = calc.predict(1, 3)
        self.assertVectorAlmostEqual(final.position_enu_m, (radius + 100, 10100 + radius, 0))
        self.assertVectorAlmostEqual(final.velocity_enu_mps, (100, 0, 0))
        self.assertIsNone(final.maneuver_kind)

    def test_both_turn_directions_and_acceleration_interpretations(self):
        for kind, direction in ((3, -1), (4, 1)):
            for model, duration in (("lateral_acceleration", math.pi * 100 / (2 * 9.80665 * 2)),
                                    ("load_factor", math.pi * 100 / (2 * 9.80665 * math.sqrt(3)))):
                with self.subTest(kind=kind, model=model):
                    calc = calculator("10 0 100 100 0 0 1 10 1 1 100", f"0 {kind} 2 90", turn_model=model)
                    self.assertVectorAlmostEqual(calc.predict(1, duration).velocity_enu_mps, (direction * 100, 0, 0))

    def test_rotating_acceleration_entry_reference_matches_circular_turn(self):
        values = "10 0 100 100 105 0 1 10 1 1 100"
        for kind in (3, 4):
            common = dict(gravity_mps2=10, integration_step_s=0.13)
            arc = calculator(values, f"1 {kind} 2 30", **common)
            vector = calculator(values, f"1 {kind} 2 30", turn_model="rotating_acceleration", **common)
            for time in (1, 2, 4, 7):
                self.assertVectorAlmostEqual(vector.predict(1, time).position_enu_m, arc.predict(1, time).position_enu_m)
                self.assertVectorAlmostEqual(vector.predict(1, time).velocity_enu_mps, arc.predict(1, time).velocity_enu_mps)

    def test_fixed_acceleration_reference_changes_speed_and_doppler_consistently(self):
        calc = calculator("10 0 100 100 0 0 1 10 1 1 100", "1 4 2 90", gravity_mps2=10,
                          turn_model="rotating_acceleration", turn_acceleration_reference_deg=90,
                          integration_step_s=0.13)
        end = 1 + math.pi / 0.4
        self.assertNotAlmostEqual(norm(calc.predict(1, 2).velocity_enu_mps), 100, places=2)
        state = calc.predict(1, end)
        self.assertVectorAlmostEqual(state.velocity_enu_mps, (-100, 0, 0))
        self.assertVectorAlmostEqual(state.position_enu_m, (500 * (1 - math.pi / 2),
                                                          10100 + 500 * (math.pi / 2 - 1), 0))
        following = calc.predict(1, end + 1)
        self.assertVectorAlmostEqual(following.position_enu_m, (state.position_enu_m[0] - 100, state.position_enu_m[1], 0))
        for time in (2, end + 1):
            derivative = (calc.predict(1, time + 1e-4).slant_range_m - calc.predict(1, time - 1e-4).slant_range_m) / 2e-4
            self.assertAlmostEqual(calc.predict(1, time).doppler_velocity_mps, derivative, places=5)
        incompatible = calculator("10 0 100 100 0 0 1 10 1 1 100", "1 4 2 90", turn_model="rotating_acceleration", earth_model="sphere")
        with self.assertRaisesRegex(ValueError, "flat ENU"):
            incompatible.predict(1, 2)

    def test_speed_coupled_rotation_matches_aligned_arcs_and_rounds_duration(self):
        for kind, sign in ((3, -1), (4, 1)):
            calc = calculator("10 0 100 100 90 0 1 10 1 1 100", f"1 {kind} 2 30",
                              gravity_mps2=10, turn_model="speed_coupled_rotation",
                              turn_acceleration_reference_deg=90, turn_duration_step_s=0.1,
                              integration_step_s=0.03)
            duration = math.ceil(math.radians(30) / 0.2 / 0.1) * 0.1
            following = calc.predict(1, 1 + duration + 1)
            heading = math.pi / 2 + sign * 0.2 * duration
            self.assertVectorAlmostEqual(following.velocity_enu_mps,
                                         (100 * math.sin(heading), 100 * math.cos(heading), 0), places=6)
            self.assertAlmostEqual(norm(following.velocity_enu_mps), 100, places=6)
        for value in (0, -0.1):
            with self.assertRaises(ValueError):
                ModelOptions(turn_duration_step_s=value)

    def test_speed_coupled_rotation_satisfies_independent_angle_solution(self):
        # In the rotating basis u=v.t, n=v.t', C=|v|-u is constant.
        # n=n0+C*theta; a*t=v0*theta+n0*theta^2/2+C*theta^3/6.
        values = "10 0 100 100 105 0 1 10 1 1 100"
        for kind, sign in ((3, -1), (4, 1)):
            states = []
            for step in (0.07, 0.013):
                calc = calculator(values, f"1 {kind} 2 30", gravity_mps2=10,
                                  turn_model="speed_coupled_rotation", turn_acceleration_reference_deg=90,
                                  integration_step_s=step)
                time = 2.5
                state = calc.predict(1, time)
                c = 100 * (1 - math.sin(math.radians(105)))
                n0 = -100 * math.cos(math.radians(105))
                low, high = (-1, 0) if sign < 0 else (0, 1)
                for _ in range(70):
                    theta = (low + high) / 2
                    polynomial = 100 * theta + n0 * theta ** 2 / 2 + c * theta ** 3 / 6
                    if polynomial < sign * 20 * (time - 1):
                        low = theta
                    else:
                        high = theta
                theta = (low + high) / 2
                speed = 100 + n0 * theta + c * theta ** 2 / 2
                u, n = speed - c, n0 + c * theta
                expected = (u * math.cos(theta) - n * math.sin(theta),
                            -u * math.sin(theta) - n * math.cos(theta), 0)
                self.assertVectorAlmostEqual(state.velocity_enu_mps, expected, places=6)
                derivative = tuple((a - b) / 2e-4 for a, b in zip(
                    calc.predict(1, time + 1e-4).position_enu_m, calc.predict(1, time - 1e-4).position_enu_m))
                self.assertVectorAlmostEqual(derivative, state.velocity_enu_mps, places=5)
                range_rate = (calc.predict(1, time + 1e-4).slant_range_m - calc.predict(1, time - 1e-4).slant_range_m) / 2e-4
                self.assertAlmostEqual(range_rate, state.doppler_velocity_mps, places=5)
                states.append(state)
            self.assertVectorAlmostEqual(states[0].position_enu_m, states[1].position_enu_m, places=6)

    def test_persistent_reference_and_initial_duration_reverse_two_turns(self):
        # Equal, opposite acceleration histories retrace velocity space even when
        # the first turn changes speed. A pause must preserve the reference angle.
        common = dict(gravity_mps2=10, turn_model="speed_coupled_rotation",
                      turn_acceleration_reference_deg=90, turn_reference_mode="persistent",
                      turn_duration_speed="initial", integration_step_s=0.02)
        calc = calculator("10 0 100 100 105 0 1 10 2 1 100", "1 4 2 30\n5 3 2 30", **common)
        final = calc.predict(1, 9)
        self.assertVectorAlmostEqual(final.velocity_enu_mps,
                                     (100 * math.sin(math.radians(105)), 100 * math.cos(math.radians(105)), 0), places=6)
        reset = calculator("10 0 100 100 105 0 1 10 2 1 100", "1 4 2 30\n5 3 2 30",
                           **dict(common, turn_reference_mode="reset"))
        self.assertGreater(norm(tuple(a - b for a, b in zip(reset.predict(1, 9).velocity_enu_mps, final.velocity_enu_mps))), 1)

    def test_vertical_triangular_climb_and_descent_all_four_kinds(self):
        for kind, sign in ((1, 1), (2, -1)):
            calc = calculator("10 0 100 100 0 0 1 10 1 1 100", f"1 {kind} 1 10", vertical_goal_scale_m=1,
                              gravity_mps2=10, speed_kind="horizontal", integration_step_s=0.13)
            middle = calc.predict(1, 2)
            self.assertAlmostEqual(middle.position_enu_m[2], sign * 5)
            self.assertAlmostEqual(middle.velocity_enu_mps[2], sign * 10)
            final = calc.predict(1, 4)
            self.assertAlmostEqual(final.position_enu_m[2], sign * 10)
            self.assertAlmostEqual(final.velocity_enu_mps[2], 0)

    def test_discrete_vertical_leaves_unbalanced_velocity_without_goal_clamp(self):
        for kind, sign in ((1, 1), (2, -1)):
            for integration_step in (0.013, 0.17):
                calc = calculator("10 0 100 100 90 0 1 10 1 1 100", f"1 {kind} 1 1.1",
                                  vertical_model="discrete_triangular", vertical_goal_scale_m=1,
                                  gravity_mps2=10, speed_kind="horizontal", vertical_update_step_s=0.1,
                                  integration_step_s=integration_step)
                # Four accelerating updates, three reversing updates: residual vz = +/-1.
                self.assertAlmostEqual(calc.predict(1, 2).velocity_enu_mps[2], sign)
                self.assertAlmostEqual(calc.predict(1, 2).position_enu_m[2], sign * 1.9)
                self.assertAlmostEqual(calc.predict(1, 3).position_enu_m[2], sign * 2.9)
                time = 1.45
                derivative = (calc.predict(1, time + 1e-4).slant_range_m - calc.predict(1, time - 1e-4).slant_range_m) / 2e-4
                self.assertAlmostEqual(calc.predict(1, time).doppler_velocity_mps, derivative, places=5)
        with self.assertRaises(ValueError):
            ModelOptions(vertical_update_step_s=0)

    def test_discrete_vertical_balanced_updates_finish_level(self):
        calc = calculator("10 0 100 100 90 0 1 10 1 1 100", "1 1 1 2.5",
                          vertical_model="discrete_triangular", vertical_goal_scale_m=1,
                          gravity_mps2=10, vertical_update_step_s=0.1, integration_step_s=0.17)
        self.assertAlmostEqual(calc.predict(1, 4).velocity_enu_mps[2], 0)
        self.assertAlmostEqual(calc.predict(1, 4).position_enu_m[2], 2.5)

    def test_vertical_triangular_nonzero_initial_vertical_speed(self):
        calc = calculator("10 0 100 100 0 3 1 10 1 1 100", "0 1 1 10", vertical_kind="rate", vertical_goal_scale_m=1,
                          gravity_mps2=10, speed_kind="horizontal")
        duration = 2 * (-3 + math.sqrt(109)) / 10
        final = calc.predict(1, duration)
        self.assertAlmostEqual(final.position_enu_m[2], 10)
        self.assertAlmostEqual(final.velocity_enu_mps[2], 3)

    def test_vertical_acceleration_and_rate_post_goal(self):
        accelerated = calculator("10 0 100 100 0 0 1 10 1 1 100", "0 1 1 20", vertical_goal_scale_m=1,
                                 vertical_model="acceleration", gravity_mps2=10, speed_kind="horizontal")
        final = accelerated.predict(1, 3)
        self.assertAlmostEqual(final.position_enu_m[2], 40)
        self.assertAlmostEqual(final.velocity_enu_mps[2], 20)
        rate = calculator("10 0 100 100 0 0 1 10 1 1 100", "0 1 1 20", vertical_goal_scale_m=1,
                          vertical_model="rate", vertical_rate_mps=10, post_maneuver="initial")
        self.assertAlmostEqual(rate.predict(1, 3).position_enu_m[2], 20)
        self.assertAlmostEqual(rate.predict(1, 3).velocity_enu_mps[2], 0)

    def test_attitude_full_rotation_and_antenna_azimuth_constraint(self):
        calc = calculator("10 30 1000 100 0 0 1 10 0 1 100", info="1 38 127 100 12 8 20",
                          azimuth_frame="antenna", heading_frame="antenna", output_frame="antenna")
        result = calc.predict(1, 0)
        self.assertAlmostEqual(result.azimuth_deg, 30)
        self.assertAlmostEqual(result.position_enu_m[2], 900)
        self.assertAlmostEqual(result.slant_range_m, 10000)
        self.assertAlmostEqual(norm(result.velocity_enu_mps), 100)

    def test_yaw_rotation_sign_and_heading(self):
        calc = calculator("10 0 100 100 0 0 1 10 0 1 100", info="1 38 127 100 0 0 90",
                          azimuth_frame="antenna", heading_frame="antenna", output_frame="antenna")
        result = calc.predict(1, 1)
        self.assertVectorAlmostEqual(result.position_enu_m, (10100, 0, 0))
        self.assertAlmostEqual(result.azimuth_deg, 0)

    def test_sphere_initial_slant_and_curvature(self):
        calc = calculator("100 20 100 100 0 0 1 10 0 1 100", earth_model="sphere")
        result = calc.predict(1, 0)
        self.assertAlmostEqual(result.slant_range_m, 100000, places=6)
        self.assertAlmostEqual(result.azimuth_deg, 20)
        self.assertLess(result.elevation_deg, 0)
        self.assertAlmostEqual(norm(result.velocity_enu_mps), 100)

    def test_sphere_ground_arc_and_doppler_derivative(self):
        for earth in ("flat", "sphere"):
            calc = calculator("100 20 1000 100 40 -2 1 10 0 1 100", earth_model=earth, range_kind="ground")
            result = calc.predict(1, 4)
            dt = 0.001
            before, after = calc.predict(1, 4 - dt), calc.predict(1, 4 + dt)
            self.assertAlmostEqual((after.slant_range_m - before.slant_range_m) / (2 * dt), result.doppler_velocity_mps, places=4)
            derivative = tuple((a - b) / (2 * dt) for a, b in zip(after.position_enu_m, before.position_enu_m))
            self.assertVectorAlmostEqual(derivative, result.velocity_enu_mps, places=4)
            self.assertAlmostEqual(dot(result.position_enu_m, result.velocity_enu_mps) / result.slant_range_m,
                                   result.doppler_velocity_mps)

    def test_absolute_goals_and_scenario_maneuver_clock(self):
        calc = calculator("10 0 100 100 90 0 1 10 1 5 100", "6 3 1 0", field10_role="start_time",
                          maneuver_clock="scenario", turn_model="rate", turn_rate_deg_s=90, turn_goal="absolute")
        self.assertVectorAlmostEqual(calc.predict(1, 7).velocity_enu_mps, (0, 100, 0))
        calc = calculator("10 0 100 100 0 0 1 10 1 1 100", "0 1 1 110", vertical_goal="absolute",
                          vertical_goal_scale_m=1, gravity_mps2=10, post_maneuver="initial")
        self.assertAlmostEqual(calc.predict(1, 3).position_enu_m[2], 10)

    def test_overlap_and_unphysical_models_fail_explicitly(self):
        overlapping = calculator("10 0 100 100 0 0 1 10 2 1 100", "0 4 1 90\n1 3 1 90", turn_model="rate")
        with self.assertRaisesRegex(ValueError, "overlapping"):
            overlapping.predict(1, 2)
        bad_vertical = calculator("10 0 100 1 0 0 1 10 1 1 100", "0 1 1 100", vertical_goal_scale_m=1)
        with self.assertRaisesRegex(ValueError, "total speed"):
            bad_vertical.predict(1, 3)
        with self.assertRaisesRegex(ValueError, "Slant range"):
            calculator("1 0 20000 100 0 0 1 10 0 1 100").predict(1, 0)

    def test_sign_options_and_angle_wrap(self):
        result = calculator("10 90 100 100 90 0 1 10 0 1 100", azimuth_sign=-1, heading_sign=-1,
                            output_azimuth_sign=-1, doppler_sign=-1).predict(1, 0)
        self.assertAlmostEqual(result.azimuth_deg, 90)
        self.assertAlmostEqual(result.doppler_velocity_mps, -100)

    def test_invalid_options(self):
        for options in ({"speed_scale_mps": 0}, {"integration_step_s": -1}, {"earth_model": "ellipsoid"},
                        {"doppler_sign": 0}, {"start_offset_s": math.nan}, {"attitude_order": (0, 0, 1)}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ModelOptions(**options)
        with self.assertRaises(ValueError):
            calculator().predict(1, math.inf)

    def test_ignored_maneuver_does_not_interrupt_active_motion(self):
        calc = calculator("10 0 100 100 0 0 1 10 2 1 100", "0 4 1 90\n0.5 3 1 90",
                          turn_model="rate", turn_rate_deg_s=90,
                          maneuver_actions=("climb", "descend", "ignore", "right"))
        self.assertVectorAlmostEqual(calc.predict(1, 2).velocity_enu_mps, (100, 0, 0))

    def test_vertical_total_speed_integration_converges(self):
        positions = []
        for step in (0.1, 0.05, 0.025):
            calc = calculator("10 0 100 100 0 0 1 10 1 1 100", "0 1 1 100", vertical_goal_scale_m=1,
                              gravity_mps2=10, integration_step_s=step)
            result = calc.predict(1, 8)
            self.assertAlmostEqual(norm(result.velocity_enu_mps), 100)
            positions.append(result.position_enu_m[1])
        self.assertLess(abs(positions[2] - positions[1]), abs(positions[1] - positions[0]) / 2)

    def test_spherical_antenna_and_ground_combination(self):
        calc = calculator("100 30 1000 100 0 0 1 10 0 1 100", info="1 38 127 100 12 8 20",
                          earth_model="sphere", range_kind="ground", azimuth_frame="antenna", output_frame="antenna")
        self.assertAlmostEqual(calc.predict(1, 0).azimuth_deg, 30)

    def test_periodic_scenario_with_start_delay_and_global_end(self):
        calc = calculator(field10_role="start_time", field11_role="end_time", scenario_repeat_period_s=100.1)
        first, second = calc.predict(1, 5), calc.predict(1, 105.1)
        self.assertVectorAlmostEqual(first.position_enu_m, second.position_enu_m)
        self.assertFalse(calc.predict(1, 100.05).active)
        self.assertFalse(calc.predict(1, 100.5).active)
        with self.assertRaises(ValueError):
            ModelOptions(scenario_repeat_period_s=0)

    def test_fixed_plane_climb_angle_and_doppler(self):
        calc = calculator("10 0 100 100 0 -10 1 10 1 1 100", "0 4 1 90", vertical_angle_mode="fixed_plane",
                          turn_model="rate", turn_rate_deg_s=90)
        radius = 100 / (math.pi / 2)
        result = calc.predict(1, 2)
        self.assertAlmostEqual(result.position_enu_m[2], -radius * math.sin(math.radians(10)))
        self.assertAlmostEqual(result.velocity_enu_mps[2], 0)
        delta = 0.001
        derivative = (calc.predict(1, 2 + delta).slant_range_m - calc.predict(1, 2 - delta).slant_range_m) / (2 * delta)
        self.assertAlmostEqual(result.doppler_velocity_mps, derivative, places=4)


if __name__ == "__main__":
    unittest.main()
