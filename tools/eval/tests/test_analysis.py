import math
from pathlib import Path
import tempfile
import unittest

from tools.eval.analyze import statistics_for
from tools.eval.detections import Event, Plot, associate, load_events
from tools.eval.detections import PlotTrack
from tools.eval.compare_runs import interpolate
from tools.eval.fit_straight import fit, solve_clock
from tools.eval.intake import snapshot
from tools.eval.make_simple_scenarios import create_simple_scenarios
from tools.eval.scenario import load_scenario, parse_scenario
from tools.eval.truth import ModelOptions


HEADER = "beamType,requestId,transmitTime_ms,plotCount,plotIndex,slantRange_m,azimuth_deg,elevation_deg,dopplerVelocity_mps,power_db\n"


def make_plot(east, time_ms, index=0):
    return Plot(time_ms + index + 2, "SEARCH", 0, time_ms, index, math.hypot(east, 10000),
                math.degrees(math.atan2(east, 10000)), 0, 0)


class AnalysisTest(unittest.TestCase):
    def test_empty_and_multi_plot_events_and_time_filter(self):
        text = HEADER + "SEARCH,1,10,0,,,,,,\nSEARCH,2,20,2,0,1000,0,0,0,10\nSEARCH,2,20,2,1,2000,10,0,1,10\n"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "detections.csv"
            path.write_text(text)
            events = load_events(path)
            self.assertEqual([len(event.plots) for event in events], [0, 2])
            self.assertEqual(len(load_events(path, 20, 20)), 1)

    def test_partial_duplicate_and_nonfinite_events_are_rejected(self):
        cases = [
            "SEARCH,1,10,2,0,1000,0,0,0,10\n",
            "SEARCH,1,10,1,0,1000,0,0,0,10\nSEARCH,1,10,1,0,1000,0,0,0,10\n",
            "SEARCH,1,10,0,0,,,,,\n",
            "SEARCH,1,10,1,0,nan,0,0,0,10\n",
            "SEARCH,1,10,1,0,1000,0\n",
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "detections.csv"
            for text in cases:
                with self.subTest(text=text):
                    path.write_text(HEADER + text)
                    with self.assertRaises(ValueError):
                        load_events(path)

    def test_crossing_paths_keep_identity_using_velocity(self):
        events = [Event(0, (make_plot(-1000, 0), make_plot(1000, 0, 1))),
                  Event(600, (make_plot(400, 600), make_plot(-400, 600, 1))),
                  Event(1200, (make_plot(-200, 1200), make_plot(200, 1200, 1)))]
        tracks = associate(events)
        self.assertEqual(len(tracks), 2)
        self.assertAlmostEqual(tracks[0].plots[-1].position[0], 200)
        self.assertAlmostEqual(tracks[1].plots[-1].position[0], -200)

    def test_competing_tracks_are_flagged(self):
        events = [Event(0, (make_plot(-20, 0), make_plot(20, 0, 1))), Event(100, (make_plot(0, 100),))]
        tracks = associate(events)
        self.assertEqual(sum(len(track.ambiguous_rows) for track in tracks), 1)

    def test_statistics_and_known_clock_recovery(self):
        result = statistics_for([{"phase_s": t, "error": 2 * t - 1} for t in (0, 1, 2)], "error")
        self.assertAlmostEqual(result["slope_per_s"], 2)
        vectors = [(2, 0, 0), (0, 3, 0)]
        errors = [(0.4, 0, 0), (0, 0.9, 0)]
        offset, period = solve_clock(vectors, errors, [1, 2], True)
        self.assertAlmostEqual(offset, 0.1)
        self.assertAlmostEqual(period, 0.1)

    def test_snapshot_is_byte_exact_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            source, destination = Path(folder) / "source.csv", Path(folder) / "copy.csv"
            source.write_bytes(b"header\r\nrow\r\n")
            result = snapshot(source, destination)
            self.assertEqual(source.read_bytes(), destination.read_bytes())
            self.assertEqual(result["bytes"], 13)
            with self.assertRaises(FileExistsError):
                snapshot(source, destination)

    def test_generated_experiments_preserve_source_and_cr_format(self):
        text = "SCENARIO_INFORMATION\r1 38 127 100 0 0 0\rTARGET_1\r10 0 100 100 90 0 1 10 4 1 100\rMANEUVER_1\r10 1 1 100\r30 2 1 100\r50 3 1 30\r70 4 1 30\r"
        scenario = parse_scenario(text)
        with tempfile.TemporaryDirectory() as folder:
            create_simple_scenarios(scenario, 1, folder)
            for path in Path(folder).glob("*.ini"):
                self.assertNotIn(b"\n", path.read_bytes())
                result = load_scenario(path)
                self.assertEqual(len(result.targets), 1)
                self.assertLessEqual(len(result.target(1).maneuvers), 1)
        self.assertEqual(len(scenario.target(1).maneuvers), 4)

    def test_common_phase_interpolation_refuses_extrapolation_and_large_gaps(self):
        points = [{"phase_s": t, "observed_e_m": 10 * t, "observed_n_m": 1000,
                   "observed_u_m": 100, "observed_doppler_mps": 2 * t} for t in (1, 2)]
        result, gap = interpolate(points, 1.5, 1.2)
        self.assertEqual(result["observed_e_m"], 15)
        self.assertEqual(result["observed_doppler_mps"], 3)
        self.assertEqual(gap, 1)
        self.assertIsNone(interpolate(points, 0, 1.2)[0])
        self.assertIsNone(interpolate(points, 1.5, 0.5)[0])

    def test_exact_phase_uses_real_sample(self):
        point = {"phase_s": 5, "observed_e_m": 20}
        result, gap = interpolate([point], 5, 0.1)
        self.assertEqual(result, point)
        self.assertEqual(gap, 0)

    def test_clock_only_fit_preserves_physical_options(self):
        scenario = parse_scenario("SCENARIO_INFORMATION\n1 38 127 100 0 0 0\nTARGET_1\n10 0 100 100 90 0 1 10 1 1 100\nMANEUVER_1\n10 4 1 30\n")
        plots = []
        for cycle in (1, 2):
            for phase in (2, 4, 6):
                time_s = 0.3 + cycle * 100.2 + phase
                plots.append(make_plot(100 * (phase - 1), round(time_s * 1000)))
        options = ModelOptions(gravity_mps2=9.8, altitude_reference="msl")
        result = fit(scenario, [PlotTrack(1, plots)], {"1": 1}, 0.25, 100.1, options)
        best = result["top_candidates"][0]
        self.assertAlmostEqual(best["options"]["start_offset_s"], 0.3)
        self.assertAlmostEqual(best["options"]["scenario_repeat_period_s"], 100.2)
        self.assertEqual(best["options"]["gravity_mps2"], 9.8)
        self.assertLess(best["rms_xyz_m"], 1e-6)
        self.assertEqual(result["candidate_count"], 1)


if __name__ == "__main__":
    unittest.main()
