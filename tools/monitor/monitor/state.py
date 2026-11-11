"""수신한 메시지를 누적하고 화면에 필요한 값을 계산한다.

모든 시간 계산은 처리부 시각(빔 송신 명령과 상태 메시지의 시각)을 기준으로 한다.
그래야 처리부 속도를 바꿔도 점유율과 탐색 한 바퀴 시간이 같은 값으로 나온다.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

from monitor.connection import ConnectionChanged, Event
from monitor.protocol import Beam, BeamType, MonitorStatus, RadarAttitude, Track, TrackList

BEAM_HISTORY_SIZE = 5000
# 이보다 크게 처리부 시각이 거꾸로 가면 처리부가 다시 시작했다고 보고 기록을 지운다.
TIME_RESET_THRESHOLD_MS = 1000
# 이 시간 동안 상태 메시지가 없으면 상태를 알 수 없다고 표시한다.
STATUS_TIMEOUT_SECONDS = 1.0
# 이 배율 이하이거나 정지 상태이면 자동 보기에서 상세 보기를 사용한다.
DETAIL_VIEW_TIME_SCALE = 0.25


@dataclass
class SearchCell:
    azimuth_deg: float
    elevation_deg: float
    width_deg: float
    last_ms: int
    previous_ms: Optional[int] = None


class ViewMode:
    AUTO = "auto"
    DETAIL = "detail"
    FLOW = "flow"


class MonitorState:
    def __init__(self) -> None:
        self.connected = False
        self.connection_detail = "connecting"
        self.status: Optional[MonitorStatus] = None
        self._status_received_at: Optional[float] = None
        self.attitude: Optional[RadarAttitude] = None
        self.tracks: List[Track] = []
        self.beams: Deque[Beam] = deque(maxlen=BEAM_HISTORY_SIZE)
        self.search_cells: Dict[int, SearchCell] = {}
        self._azimuth_reference: Optional[float] = None
        self._last_search_interval_ms: Optional[int] = None
        self._latest_beam_ms: Optional[int] = None

    def apply(self, event: Event, received_at: Optional[float] = None) -> None:
        if isinstance(event, ConnectionChanged):
            self._apply_connection(event)
        elif isinstance(event, Beam):
            self._apply_beam(event)
        elif isinstance(event, TrackList):
            self.tracks = sorted(event.tracks, key=lambda track: track.track_id)
        elif isinstance(event, RadarAttitude):
            self.attitude = event
        elif isinstance(event, MonitorStatus):
            self.status = event
            self._status_received_at = time.monotonic() if received_at is None else received_at

    def _apply_connection(self, event: ConnectionChanged) -> None:
        if event.connected and not self.connected:
            # 다시 연결된 처리부는 새로 시작했을 수 있으므로 이전 기록을 버린다.
            self.clear_history()
            self.status = None
            self._status_received_at = None
        self.connected = event.connected
        self.connection_detail = event.detail

    def clear_history(self) -> None:
        self.tracks = []
        self.beams.clear()
        self.search_cells.clear()
        self._azimuth_reference = None
        self._last_search_interval_ms = None
        self._latest_beam_ms = None

    def _apply_beam(self, beam: Beam) -> None:
        if (self._latest_beam_ms is not None and
                beam.timestamp_ms + TIME_RESET_THRESHOLD_MS < self._latest_beam_ms):
            self.clear_history()

        self.beams.append(beam)
        if self._latest_beam_ms is None or beam.timestamp_ms > self._latest_beam_ms:
            self._latest_beam_ms = beam.timestamp_ms

        if beam.beam_type == BeamType.SEARCH:
            self._apply_search_beam(beam)

    def _apply_search_beam(self, beam: Beam) -> None:
        if self._azimuth_reference is None:
            self._azimuth_reference = beam.azimuth_deg

        cell = self.search_cells.get(beam.beam_id)
        if cell is None:
            self.search_cells[beam.beam_id] = SearchCell(
                self.unwrap_azimuth(beam.azimuth_deg),
                beam.elevation_deg,
                beam.azimuth_width_deg,
                beam.timestamp_ms,
            )
            return

        cell.azimuth_deg = self.unwrap_azimuth(beam.azimuth_deg)
        cell.elevation_deg = beam.elevation_deg
        cell.previous_ms = cell.last_ms
        cell.last_ms = beam.timestamp_ms
        self._last_search_interval_ms = cell.last_ms - cell.previous_ms

    def unwrap_azimuth(self, azimuth_deg: float) -> float:
        """첫 탐색 빔의 방위를 기준으로 ±180도 안에 들도록 방위를 펼친다.

        격자가 0도나 180도를 가로질러도 화면에서 끊기지 않게 한다.
        """
        if self._azimuth_reference is None:
            return azimuth_deg
        reference = self._azimuth_reference
        return reference + ((azimuth_deg - reference + 180.0) % 360.0) - 180.0

    def now_ms(self) -> Optional[int]:
        """화면에 표시할 현재 처리부 시각."""
        candidates = [value for value in (
            self._latest_beam_ms,
            self.status.processor_time_ms if self.status else None,
        ) if value is not None]
        return max(candidates) if candidates else None

    def status_is_fresh(self, now: Optional[float] = None) -> bool:
        if self._status_received_at is None:
            return False
        current = time.monotonic() if now is None else now
        return current - self._status_received_at <= STATUS_TIMEOUT_SECONDS

    def recent_beams(self, window_ms: int) -> List[Beam]:
        """최근 window_ms 동안 송신한 빔을 오래된 순서로 반환한다."""
        now = self.now_ms()
        if now is None:
            return []
        start = now - window_ms
        result = []
        for beam in reversed(self.beams):
            if beam.timestamp_ms < start:
                break
            result.append(beam)
        result.reverse()
        return result

    def occupancy(self, window_ms: int) -> Dict[BeamType, float]:
        """최근 window_ms 동안 빔 종류별 비율을 반환한다. 빔이 없으면 모두 0이다."""
        beams = self.recent_beams(window_ms)
        counts = {beam_type: 0 for beam_type in BeamType}
        for beam in beams:
            if beam.beam_type in counts:
                counts[BeamType(beam.beam_type)] += 1
        total = sum(counts.values())
        if total == 0:
            return {beam_type: 0.0 for beam_type in BeamType}
        return {beam_type: count / total for beam_type, count in counts.items()}

    def search_sweep_ms(self) -> Optional[int]:
        """가장 최근에 다시 방문한 탐색 칸의 방문 간격. 탐색 한 바퀴 시간에 해당한다."""
        return self._last_search_interval_ms

    def resolve_view_mode(self, selected: str) -> str:
        if selected != ViewMode.AUTO:
            return selected
        if self.status and (self.status.paused or self.status.time_scale <= DETAIL_VIEW_TIME_SCALE):
            return ViewMode.DETAIL
        return ViewMode.FLOW
