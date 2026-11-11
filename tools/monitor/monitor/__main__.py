"""모니터 실행 진입점.

    python -m monitor --env ../../.env
"""

from __future__ import annotations

import argparse
import queue
import sys
import time
from pathlib import Path
from typing import List, Optional

from monitor.config import ConfigError, MonitorConfig, load_config
from monitor.connection import Event, ProcessorConnection
from monitor.protocol import BeamType
from monitor.state import MonitorState


def parse_arguments(arguments: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="monitor", description="AESA 데이터 처리부 모니터")
    parser.add_argument("--env", type=Path, default=Path(".env"),
                        help="처리부와 같은 .env 파일 경로 (기본값: 현재 디렉터리의 .env)")
    parser.add_argument("--server", help="처리부 서버 주소 ADDRESS:PORT. .env 값을 덮어쓴다")
    parser.add_argument("--local", help="모니터가 바인딩할 주소 ADDRESS:PORT. .env 값을 덮어쓴다")
    parser.add_argument("--headless", action="store_true",
                        help="화면 없이 1초마다 수신 요약을 출력한다")
    parser.add_argument("--duration", type=float, default=None,
                        help="--headless에서 지정한 초만큼 실행한 뒤 종료한다")
    return parser.parse_args(arguments)


def resolve_config(arguments: argparse.Namespace) -> MonitorConfig:
    env_path: Optional[Path] = arguments.env
    # 서버와 로컬 주소를 모두 지정하면 .env 파일이 없어도 실행할 수 있다.
    if not env_path.is_file() and arguments.server and arguments.local:
        env_path = None
    return load_config(env_path, arguments.server, arguments.local)


def run_headless(events: "queue.Queue[Event]", duration: Optional[float]) -> None:
    state = MonitorState()
    started = time.monotonic()
    next_report = started + 1.0
    while duration is None or time.monotonic() - started < duration:
        try:
            state.apply(events.get(timeout=0.1))
        except queue.Empty:
            pass
        if time.monotonic() >= next_report:
            next_report += 1.0
            print(summarize(state), flush=True)


def summarize(state: MonitorState) -> str:
    occupancy = state.occupancy(5000)
    status = state.status
    peers = "-"
    if status is not None:
        peers = (f"console={int(status.operator_console_connected)} "
                 f"simulator={int(status.scenario_simulator_connected)} "
                 f"scale={status.time_scale:g} paused={int(status.paused)}")
    sweep = state.search_sweep_ms()
    return (
        f"connected={int(state.connected)} time_ms={state.now_ms()} {peers} "
        f"beams={len(state.beams)} cells={len(state.search_cells)} "
        f"search={occupancy[BeamType.SEARCH]:.2f} "
        f"confirm={occupancy[BeamType.CONFIRMATION]:.2f} "
        f"track={occupancy[BeamType.TRACKING]:.2f} "
        f"sweep_ms={sweep} tracks={len(state.tracks)}"
    )


def main(arguments: Optional[List[str]] = None) -> int:
    options = parse_arguments(arguments)
    try:
        config = resolve_config(options)
    except ConfigError as error:
        print(f"monitor: {error}", file=sys.stderr)
        return 2

    events: "queue.Queue[Event]" = queue.Queue()
    connection = ProcessorConnection(config, events)
    connection.start()
    server_text = f"{config.server[0]}:{config.server[1]} ← {config.local[0]}:{config.local[1]}"
    try:
        if options.headless:
            print(f"monitor: {server_text}", flush=True)
            run_headless(events, options.duration)
        else:
            import tkinter as tk

            from monitor.ui import MonitorWindow

            root = tk.Tk()
            MonitorWindow(root, events, server_text)
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        connection.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
