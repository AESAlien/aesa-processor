"""데이터 처리부 TCP 서버 접속과 재접속."""

from __future__ import annotations

import queue
import socket
import threading
from dataclasses import dataclass
from typing import Optional, Union

from monitor.config import MonitorConfig
from monitor.protocol import Message, MessageFramer, ProtocolError, decode

RECONNECT_DELAY_SECONDS = 1.0
RECEIVE_SIZE = 65536


@dataclass(frozen=True)
class ConnectionChanged:
    connected: bool
    detail: str = ""


Event = Union[Message, ConnectionChanged]


class ProcessorConnection:
    """수신 스레드에서 처리부에 접속하고 받은 메시지를 큐에 넣는다.

    연결이 끊기거나 프레임이 잘못되면 연결을 닫고 일정 시간 뒤 다시 접속한다.
    모니터는 처리부에 아무것도 보내지 않는다.
    """

    def __init__(self, config: MonitorConfig, events: "queue.Queue[Event]") -> None:
        self._config = config
        self._events = events
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="processor-connection", daemon=True)
        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            if self._socket is not None:
                try:
                    self._socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            detail = self._connect_and_receive()
            self._events.put(ConnectionChanged(False, detail))
            self._stop.wait(RECONNECT_DELAY_SECONDS)

    def _connect_and_receive(self) -> str:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except OSError as error:
            return f"socket: {error}"

        with self._lock:
            self._socket = sock
        try:
            # 이전 연결이 TIME_WAIT 상태여도 같은 로컬 포트로 다시 접속할 수 있게 한다.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(self._config.local)
            sock.connect(self._config.server)
            self._events.put(ConnectionChanged(True))
            return self._receive(sock)
        except OSError as error:
            return str(error)
        finally:
            with self._lock:
                self._socket = None
            sock.close()

    def _receive(self, sock: socket.socket) -> str:
        framer = MessageFramer()
        while not self._stop.is_set():
            data = sock.recv(RECEIVE_SIZE)
            if not data:
                return "closed by processor"
            try:
                for frame in framer.feed(data):
                    self._events.put(decode(frame))
            except ProtocolError as error:
                return f"protocol error: {error}"
        return "stopped"
