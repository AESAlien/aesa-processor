import unittest

from monitor.connection import ConnectionChanged
from monitor.protocol import Beam, BeamType, MonitorStatus, Track, TrackList
from monitor.state import MonitorState, ViewMode


def beam(timestamp, beam_type=BeamType.SEARCH, beam_id=0, azimuth=0.0, elevation=0.0):
    return Beam(int(beam_type), timestamp, beam_id, 0, azimuth, elevation, 6.0, 6.0)


def track(track_id):
    return Track(track_id, 2, 10.0, 9.0, 1.0, 2.0, 3.0, 37.0, 127.0, 1.0, 200.0, 45.0, 0.0, 100)


def status(time_ms=0, scale=1.0, paused=False):
    return MonitorStatus(time_ms, scale, paused, True, True)


class OccupancyTest(unittest.TestCase):
    def test_counts_beam_types_within_window(self):
        state = MonitorState()
        for index in range(10):
            beam_type = BeamType.TRACKING if index == 9 else BeamType.SEARCH
            state.apply(beam(index * 10, beam_type, index))

        occupancy = state.occupancy(1000)

        self.assertAlmostEqual(occupancy[BeamType.SEARCH], 0.9)
        self.assertAlmostEqual(occupancy[BeamType.TRACKING], 0.1)
        self.assertAlmostEqual(occupancy[BeamType.CONFIRMATION], 0.0)

    def test_excludes_beams_older_than_window(self):
        state = MonitorState()
        state.apply(beam(0, BeamType.TRACKING))
        state.apply(beam(5000, BeamType.SEARCH))

        self.assertEqual([item.timestamp_ms for item in state.recent_beams(1000)], [5000])
        self.assertAlmostEqual(state.occupancy(1000)[BeamType.SEARCH], 1.0)

    def test_empty_state_has_zero_occupancy(self):
        self.assertEqual(set(MonitorState().occupancy(1000).values()), {0.0})


class SearchSweepTest(unittest.TestCase):
    def test_sweep_time_is_revisit_interval_of_search_cell(self):
        state = MonitorState()
        # 탐색 칸 3개를 10ms 간격으로 두 바퀴 돈다.
        for index in range(6):
            state.apply(beam(index * 10, BeamType.SEARCH, index % 3))

        self.assertEqual(state.search_sweep_ms(), 30)
        self.assertEqual(len(state.search_cells), 3)

    def test_request_beams_do_not_affect_search_cells(self):
        state = MonitorState()
        state.apply(beam(0, BeamType.SEARCH, 0))
        state.apply(beam(10, BeamType.TRACKING, 0))

        self.assertEqual(state.search_cells[0].last_ms, 0)
        self.assertIsNone(state.search_sweep_ms())

    def test_unwraps_azimuth_around_reference(self):
        state = MonitorState()
        state.apply(beam(0, BeamType.SEARCH, 0, azimuth=350.0))

        self.assertAlmostEqual(state.unwrap_azimuth(10.0), 370.0)
        self.assertAlmostEqual(state.unwrap_azimuth(340.0), 340.0)


class ResetTest(unittest.TestCase):
    def test_clears_history_when_processor_time_goes_back(self):
        state = MonitorState()
        state.apply(beam(50000, BeamType.SEARCH, 0))
        state.apply(TrackList([track(1)]))

        state.apply(beam(10, BeamType.SEARCH, 5))

        self.assertEqual([item.timestamp_ms for item in state.beams], [10])
        self.assertEqual(list(state.search_cells), [5])
        self.assertEqual(state.tracks, [])

    def test_clears_history_on_reconnection(self):
        state = MonitorState()
        state.apply(ConnectionChanged(True))
        state.apply(beam(100))
        state.apply(ConnectionChanged(False, "closed"))
        self.assertEqual(len(state.beams), 1)

        state.apply(ConnectionChanged(True))

        self.assertTrue(state.connected)
        self.assertEqual(len(state.beams), 0)


class StatusTest(unittest.TestCase):
    def test_status_becomes_stale_without_updates(self):
        state = MonitorState()
        state.apply(status(), received_at=100.0)

        self.assertTrue(state.status_is_fresh(now=100.5))
        self.assertFalse(state.status_is_fresh(now=101.5))

    def test_now_uses_latest_of_beam_and_status_time(self):
        state = MonitorState()
        state.apply(beam(100))
        state.apply(status(time_ms=250), received_at=0.0)

        self.assertEqual(state.now_ms(), 250)

    def test_tracks_are_sorted_by_id(self):
        state = MonitorState()
        state.apply(TrackList([track(9), track(3)]))

        self.assertEqual([item.track_id for item in state.tracks], [3, 9])


class ViewModeTest(unittest.TestCase):
    def test_auto_mode_follows_speed(self):
        state = MonitorState()
        self.assertEqual(state.resolve_view_mode(ViewMode.AUTO), ViewMode.FLOW)

        state.apply(status(scale=0.1), received_at=0.0)
        self.assertEqual(state.resolve_view_mode(ViewMode.AUTO), ViewMode.DETAIL)

        state.apply(status(scale=1.0, paused=True), received_at=0.0)
        self.assertEqual(state.resolve_view_mode(ViewMode.AUTO), ViewMode.DETAIL)

        state.apply(status(scale=1.0), received_at=0.0)
        self.assertEqual(state.resolve_view_mode(ViewMode.AUTO), ViewMode.FLOW)

    def test_manual_mode_is_kept(self):
        state = MonitorState()
        state.apply(status(scale=0.1), received_at=0.0)

        self.assertEqual(state.resolve_view_mode(ViewMode.FLOW), ViewMode.FLOW)


if __name__ == "__main__":
    unittest.main()
