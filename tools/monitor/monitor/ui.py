"""tkinter 메인 화면."""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import ttk
from typing import List, Optional, Tuple

from monitor.connection import Event
from monitor.protocol import Beam, BeamType, TrackStatus
from monitor.state import MonitorState, ViewMode

REFRESH_MS = 50
MAX_EVENTS_PER_REFRESH = 20000

BACKGROUND = "#ffffff"
EMPTY_CELL = "#f1efe8"
GRID_LINE = "#d3d1c7"
TEXT = "#2c2c2a"
MUTED = "#5f5e5a"
OK = "#3b6d11"
BAD = "#a32d2d"
UNKNOWN = "#888780"
BEAM_COLORS = {
    BeamType.SEARCH: "#378add",
    BeamType.CONFIRMATION: "#1d9e75",
    BeamType.TRACKING: "#d85a30",
}
BEAM_NAMES = {
    BeamType.SEARCH: "탐색",
    BeamType.CONFIRMATION: "확인",
    BeamType.TRACKING: "추적",
}
TRACK_STATUS_NAMES = {
    TrackStatus.INIT: "초기",
    TrackStatus.TRACKING: "추적",
}

# 잔상과 타임라인은 처리부 시각 기준이다. 상세 보기는 짧은 구간을 크게 보여준다.
SEARCH_AFTERGLOW_MS = {ViewMode.FLOW: 400, ViewMode.DETAIL: 150}
REQUEST_AFTERGLOW_MS = {ViewMode.FLOW: 1500, ViewMode.DETAIL: 300}
TIMELINE_WINDOW_MS = {ViewMode.FLOW: 2000, ViewMode.DETAIL: 300}
OCCUPANCY_WINDOW_MS = 5000
DETAIL_BEAM_COUNT = 15
DEFAULT_PLANE_BOUNDS = (-60.0, 60.0, -10.0, 60.0)


def blend(color: str, ratio: float, background: str = EMPTY_CELL) -> str:
    """color를 ratio만큼 background 위에 섞은 색을 반환한다."""
    ratio = min(max(ratio, 0.0), 1.0)
    channels = []
    for index in (1, 3, 5):
        foreground = int(color[index:index + 2], 16)
        back = int(background[index:index + 2], 16)
        channels.append(round(back + (foreground - back) * ratio))
    return "#" + "".join(f"{value:02x}" for value in channels)


def beam_color(beam_type: int) -> str:
    try:
        return BEAM_COLORS[BeamType(beam_type)]
    except ValueError:
        return TEXT


def beam_name(beam_type: int) -> str:
    try:
        return BEAM_NAMES[BeamType(beam_type)]
    except ValueError:
        return str(beam_type)


class MonitorWindow:
    def __init__(self, root: tk.Tk, events: "queue.Queue[Event]", server_text: str) -> None:
        self._root = root
        self._events = events
        self._state = MonitorState()
        self._view_mode = tk.StringVar(value=ViewMode.AUTO)
        self._last_tracks: Optional[list] = None

        root.title("AESA 데이터 처리부 모니터")
        root.configure(background=BACKGROUND)
        root.geometry("1180x760")
        root.minsize(900, 600)

        self._build_top_bar(server_text)
        self._build_beam_area()
        self._build_timeline()
        self._build_track_table()

        root.after(REFRESH_MS, self._refresh)

    # 화면 구성

    def _build_top_bar(self, server_text: str) -> None:
        bar = tk.Frame(self._root, background=BACKGROUND, padx=12, pady=8)
        bar.pack(fill=tk.X)

        self._processor_label = self._status_label(bar, "처리부")
        self._console_label = self._status_label(bar, "운용통제기")
        self._simulator_label = self._status_label(bar, "모의기")
        tk.Label(bar, text=server_text, background=BACKGROUND, foreground=MUTED).pack(side=tk.LEFT, padx=8)

        self._speed_label = tk.Label(bar, text="속도 -", background=BACKGROUND, foreground=TEXT,
                                     font=("TkDefaultFont", 13, "bold"))
        self._speed_label.pack(side=tk.RIGHT, padx=(12, 0))
        self._time_label = tk.Label(bar, text="처리부 시각 -", background=BACKGROUND, foreground=TEXT,
                                    font=("TkFixedFont", 12))
        self._time_label.pack(side=tk.RIGHT, padx=12)
        self._attitude_label = tk.Label(bar, text="", background=BACKGROUND, foreground=MUTED)
        self._attitude_label.pack(side=tk.RIGHT, padx=12)

    def _status_label(self, parent: tk.Widget, name: str) -> tk.Label:
        label = tk.Label(parent, text=f"● {name}", background=BACKGROUND, foreground=UNKNOWN,
                         font=("TkDefaultFont", 12))
        label.pack(side=tk.LEFT, padx=(0, 12))
        return label

    def _build_beam_area(self) -> None:
        area = tk.Frame(self._root, background=BACKGROUND, padx=12)
        area.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(area, background=BACKGROUND)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tk.Label(left, text="빔 방향 (방위·고각, 잔상)", background=BACKGROUND, foreground=MUTED,
                 anchor="w").pack(fill=tk.X)
        self._plane = tk.Canvas(left, background=BACKGROUND, highlightthickness=0, height=320)
        self._plane.pack(fill=tk.BOTH, expand=True)
        legend = tk.Frame(left, background=BACKGROUND)
        legend.pack(fill=tk.X)
        for beam_type in BeamType:
            tk.Label(legend, text=f"■ {BEAM_NAMES[beam_type]}", foreground=BEAM_COLORS[beam_type],
                     background=BACKGROUND).pack(side=tk.LEFT, padx=(0, 12))

        right = tk.Frame(area, background=BACKGROUND, width=320, padx=12)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        mode_row = tk.Frame(right, background=BACKGROUND)
        mode_row.pack(fill=tk.X, pady=(0, 8))
        tk.Label(mode_row, text="보기", background=BACKGROUND, foreground=MUTED).pack(side=tk.LEFT)
        for text, value in (("자동", ViewMode.AUTO), ("상세", ViewMode.DETAIL), ("흐름", ViewMode.FLOW)):
            tk.Radiobutton(mode_row, text=text, value=value, variable=self._view_mode,
                           background=BACKGROUND).pack(side=tk.LEFT)
        self._mode_label = tk.Label(mode_row, text="", background=BACKGROUND, foreground=MUTED)
        self._mode_label.pack(side=tk.RIGHT)

        tk.Label(right, text=f"최근 {OCCUPANCY_WINDOW_MS // 1000}초 빔 점유율", background=BACKGROUND,
                 foreground=MUTED, anchor="w").pack(fill=tk.X)
        self._occupancy = tk.Canvas(right, background=BACKGROUND, highlightthickness=0, height=22)
        self._occupancy.pack(fill=tk.X)
        self._occupancy_label = tk.Label(right, text="", background=BACKGROUND, foreground=TEXT, anchor="w")
        self._occupancy_label.pack(fill=tk.X, pady=(2, 8))

        self._sweep_label = self._metric(right, "탐색 한 바퀴")
        self._track_count_label = self._metric(right, "확정 트랙")

        tk.Label(right, text="최근 빔", background=BACKGROUND, foreground=MUTED, anchor="w").pack(
            fill=tk.X, pady=(8, 0))
        self._beam_list = tk.Text(right, height=DETAIL_BEAM_COUNT, font=("TkFixedFont", 10),
                                  background=BACKGROUND, foreground=TEXT, relief=tk.FLAT,
                                  highlightthickness=0)
        self._beam_list.pack(fill=tk.BOTH, expand=True)
        for beam_type, color in BEAM_COLORS.items():
            self._beam_list.tag_configure(beam_type.name, foreground=color)

    def _metric(self, parent: tk.Widget, title: str) -> tk.Label:
        frame = tk.Frame(parent, background=EMPTY_CELL, padx=8, pady=6)
        frame.pack(fill=tk.X, pady=(0, 6))
        tk.Label(frame, text=title, background=EMPTY_CELL, foreground=MUTED, anchor="w").pack(fill=tk.X)
        value = tk.Label(frame, text="-", background=EMPTY_CELL, foreground=TEXT, anchor="w",
                         font=("TkDefaultFont", 16, "bold"))
        value.pack(fill=tk.X)
        return value

    def _build_timeline(self) -> None:
        frame = tk.Frame(self._root, background=BACKGROUND, padx=12, pady=6)
        frame.pack(fill=tk.X)
        self._timeline_title = tk.Label(frame, text="", background=BACKGROUND, foreground=MUTED, anchor="w")
        self._timeline_title.pack(fill=tk.X)
        self._timeline = tk.Canvas(frame, background=BACKGROUND, highlightthickness=0, height=48)
        self._timeline.pack(fill=tk.X)

    def _build_track_table(self) -> None:
        frame = tk.Frame(self._root, background=BACKGROUND, padx=12, pady=6)
        frame.pack(fill=tk.BOTH)
        tk.Label(frame, text="트랙", background=BACKGROUND, foreground=MUTED, anchor="w").pack(fill=tk.X)
        columns = ("id", "status", "azimuth", "elevation", "range", "doppler", "time")
        headings = ("ID", "상태", "방위(°)", "고각(°)", "거리(km)", "도플러(m/s)", "시각(ms)")
        self._track_table = ttk.Treeview(frame, columns=columns, show="headings", height=7)
        for column, heading in zip(columns, headings):
            self._track_table.heading(column, text=heading)
            self._track_table.column(column, width=110, anchor=tk.E)
        self._track_table.pack(fill=tk.BOTH)

    # 갱신

    def _refresh(self) -> None:
        self._drain_events()
        mode = self._state.resolve_view_mode(self._view_mode.get())
        self._draw_top_bar()
        self._draw_plane(mode)
        self._draw_side_panel(mode)
        self._draw_timeline(mode)
        self._draw_tracks()
        self._root.after(REFRESH_MS, self._refresh)

    def _drain_events(self) -> None:
        for _ in range(MAX_EVENTS_PER_REFRESH):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                return
            self._state.apply(event)

    def _draw_top_bar(self) -> None:
        state = self._state
        self._set_status(self._processor_label, "처리부", state.connected,
                         "" if state.connected else state.connection_detail)

        status = state.status if state.connected and state.status_is_fresh() else None
        self._set_status(self._console_label, "운용통제기",
                         status.operator_console_connected if status else None)
        self._set_status(self._simulator_label, "모의기",
                         status.scenario_simulator_connected if status else None)

        now = state.now_ms()
        self._time_label.configure(text=f"처리부 시각 {format_time(now)}")
        if status is None:
            self._speed_label.configure(text="속도 -", foreground=UNKNOWN)
        elif status.paused:
            self._speed_label.configure(text="정지", foreground=BAD)
        else:
            self._speed_label.configure(text=f"×{format_scale(status.time_scale)}", foreground=TEXT)

        attitude = state.attitude
        if attitude is not None:
            self._attitude_label.configure(
                text=f"자세 R {attitude.roll_deg:.1f}° P {attitude.pitch_deg:.1f}° Y {attitude.yaw_deg:.1f}°")

    def _set_status(self, label: tk.Label, name: str, connected: Optional[bool], detail: str = "") -> None:
        if connected is None:
            label.configure(text=f"● {name} ?", foreground=UNKNOWN)
        elif connected:
            label.configure(text=f"● {name}", foreground=OK)
        else:
            suffix = f" ({detail})" if detail else ""
            label.configure(text=f"● {name} 끊김{suffix}", foreground=BAD)

    def _plane_bounds(self) -> Tuple[float, float, float, float]:
        cells = self._state.search_cells.values()
        if not cells:
            return DEFAULT_PLANE_BOUNDS
        margin = max(cell.width_deg for cell in cells)
        return (
            min(cell.azimuth_deg for cell in cells) - margin,
            max(cell.azimuth_deg for cell in cells) + margin,
            min(cell.elevation_deg for cell in cells) - margin,
            max(cell.elevation_deg for cell in cells) + margin,
        )

    def _draw_plane(self, mode: str) -> None:
        canvas = self._plane
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        left, right, bottom, top = self._plane_bounds()
        padding = 28

        def to_x(azimuth: float) -> float:
            return padding + (azimuth - left) / (right - left) * (width - 2 * padding)

        def to_y(elevation: float) -> float:
            return height - padding - (elevation - bottom) / (top - bottom) * (height - 2 * padding)

        degree_x = (width - 2 * padding) / (right - left)
        degree_y = (height - 2 * padding) / (top - bottom)
        canvas.create_text(padding, height - 8, text=f"{left:.0f}°", fill=MUTED, anchor="w")
        canvas.create_text(width - padding, height - 8, text=f"{right:.0f}°", fill=MUTED, anchor="e")
        canvas.create_text(4, padding, text=f"{top:.0f}°", fill=MUTED, anchor="w")
        canvas.create_text(4, height - padding, text=f"{bottom:.0f}°", fill=MUTED, anchor="w")

        now = self._state.now_ms()
        search_afterglow = SEARCH_AFTERGLOW_MS[mode]
        latest_search_id = None
        for beam in reversed(self._state.beams):
            if beam.beam_type == BeamType.SEARCH:
                latest_search_id = beam.beam_id
                break

        for beam_id, cell in self._state.search_cells.items():
            half_x = cell.width_deg / 2 * degree_x * 0.9
            half_y = cell.width_deg / 2 * degree_y * 0.9
            x = to_x(cell.azimuth_deg)
            y = to_y(cell.elevation_deg)
            age = (now - cell.last_ms) if now is not None else search_afterglow
            ratio = 1.0 - age / search_afterglow
            outline = TEXT if (mode == ViewMode.DETAIL and beam_id == latest_search_id) else ""
            canvas.create_rectangle(x - half_x, y - half_y, x + half_x, y + half_y,
                                    fill=blend(BEAM_COLORS[BeamType.SEARCH], ratio * 0.9),
                                    outline=outline, width=2)

        request_afterglow = REQUEST_AFTERGLOW_MS[mode]
        request_beams = [beam for beam in self._state.recent_beams(request_afterglow)
                         if beam.beam_type != BeamType.SEARCH]
        for index, beam in enumerate(request_beams):
            age = now - beam.timestamp_ms if now is not None else 0
            ratio = 1.0 - age / request_afterglow
            color = beam_color(beam.beam_type)
            x = to_x(self._state.unwrap_azimuth(beam.azimuth_deg))
            y = to_y(beam.elevation_deg)
            radius_x = beam.azimuth_width_deg / 2 * degree_x
            radius_y = beam.elevation_width_deg / 2 * degree_y
            latest = index == len(request_beams) - 1
            canvas.create_oval(x - radius_x, y - radius_y, x + radius_x, y + radius_y,
                               fill=blend(color, ratio, BACKGROUND), outline=color,
                               width=2 if latest else 1)

    def _draw_side_panel(self, mode: str) -> None:
        names = {ViewMode.DETAIL: "상세", ViewMode.FLOW: "흐름"}
        selected = self._view_mode.get()
        self._mode_label.configure(text=f"{names[mode]}{' (자동)' if selected == ViewMode.AUTO else ''}")

        occupancy = self._state.occupancy(OCCUPANCY_WINDOW_MS)
        canvas = self._occupancy
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        x = 0.0
        for beam_type in BeamType:
            segment = occupancy[beam_type] * width
            if segment > 0:
                canvas.create_rectangle(x, 2, x + segment, 20, fill=BEAM_COLORS[beam_type], outline="")
            x += segment
        if x == 0:
            canvas.create_rectangle(0, 2, width, 20, fill=EMPTY_CELL, outline="")
        self._occupancy_label.configure(text="   ".join(
            f"{BEAM_NAMES[beam_type]} {occupancy[beam_type] * 100:.0f}%" for beam_type in BeamType))

        sweep = self._state.search_sweep_ms()
        self._sweep_label.configure(text="-" if sweep is None else f"{sweep / 1000:.2f} s")
        self._track_count_label.configure(text=str(len(self._state.tracks)))

        self._beam_list.configure(state=tk.NORMAL)
        self._beam_list.delete("1.0", tk.END)
        recent = list(self._state.beams)[-DETAIL_BEAM_COUNT:]
        for beam in reversed(recent):
            tag = BeamType(beam.beam_type).name if beam.beam_type in BeamType._value2member_map_ else ""
            line = (f"{format_time(beam.timestamp_ms)} {beam_name(beam.beam_type)} "
                    f"#{beam.beam_id:<5} {beam.azimuth_deg:7.1f} {beam.elevation_deg:6.1f}\n")
            self._beam_list.insert(tk.END, line, tag)
        self._beam_list.configure(state=tk.DISABLED)

    def _draw_timeline(self, mode: str) -> None:
        window = TIMELINE_WINDOW_MS[mode]
        self._timeline_title.configure(text=f"빔 타임라인 (처리부 시각, 최근 {window / 1000:g}초)")
        canvas = self._timeline
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 40)
        now = self._state.now_ms()
        canvas.create_rectangle(0, 0, width, height - 14, fill=EMPTY_CELL, outline="")
        canvas.create_text(0, height - 6, text=f"-{window / 1000:g}s", fill=MUTED, anchor="w")
        canvas.create_text(width, height - 6, text="지금", fill=MUTED, anchor="e")
        if now is None:
            return
        beams: List[Beam] = self._state.recent_beams(window)
        bar_width = max(width / window * 10 * 0.8, 1.0)
        for beam in beams:
            x = width - (now - beam.timestamp_ms) / window * width
            canvas.create_rectangle(x - bar_width, 2, x, height - 16, fill=beam_color(beam.beam_type),
                                    outline="")

    def _draw_tracks(self) -> None:
        tracks = self._state.tracks
        if tracks is self._last_tracks:
            return
        self._last_tracks = tracks
        table = self._track_table
        table.delete(*table.get_children())
        for track in tracks:
            try:
                status = TRACK_STATUS_NAMES[TrackStatus(track.status)]
            except ValueError:
                status = str(track.status)
            table.insert("", tk.END, values=(
                track.track_id,
                status,
                f"{track.azimuth_deg:.2f}",
                f"{track.elevation_deg:.2f}",
                f"{track.slant_range_km:.2f}",
                f"{track.doppler_mps:.1f}",
                track.timestamp_ms,
            ))


def format_time(milliseconds: Optional[int]) -> str:
    if milliseconds is None:
        return "--:--.---"
    minutes, remainder = divmod(milliseconds, 60000)
    return f"{minutes:02d}:{remainder / 1000:06.3f}"


def format_scale(scale: float) -> str:
    return f"{scale:.2f}".rstrip("0").rstrip(".")
