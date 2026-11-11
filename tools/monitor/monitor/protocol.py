"""데이터 처리부가 모니터에 보내는 메시지의 바이트 배치와 디코딩.

C++의 src/comm/src/protocol/ST_*.hpp와 같은 배치를 사용한다.
모든 구조체는 1바이트 정렬(#pragma pack(1))이며 리틀 엔디언이다.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Union


class MessageId(IntEnum):
    BEAM_TRANSMIT = 0xAB01
    TRACK_INFORMATION = 0xAC01
    RADAR_ATTITUDE_TO_CONSOLE = 0xAC02
    MONITOR_STATUS = 0xAD01


class BeamType(IntEnum):
    SEARCH = 1
    CONFIRMATION = 2
    TRACKING = 3


class TrackStatus(IntEnum):
    INIT = 1
    TRACKING = 2


# ST_MsgHeader
HEADER = struct.Struct("<HBBIIIBBH")
# ST_BeamTx
BEAM_TX = struct.Struct("<B3xIIIffff28x")
# ST_TrkHeader
TRACK_HEADER = struct.Struct("<H2x")
# ST_Trk
TRACK = struct.Struct("<HBB11fIII28x")
# ST_RadarAttMsg의 헤더 이후 부분
RADAR_ATTITUDE = struct.Struct("<6d")
# ST_MonitorStatusMsg의 헤더 이후 부분
MONITOR_STATUS = struct.Struct("<IfBBBx")

BEAM_TX_MESSAGE_SIZE = HEADER.size + BEAM_TX.size
TRACK_MESSAGE_MAXIMUM_TRACKS = 100
TRACK_MESSAGE_MAXIMUM_SIZE = HEADER.size + TRACK_HEADER.size + TRACK.size * TRACK_MESSAGE_MAXIMUM_TRACKS
RADAR_ATTITUDE_MESSAGE_SIZE = HEADER.size + RADAR_ATTITUDE.size
MONITOR_STATUS_MESSAGE_SIZE = HEADER.size + MONITOR_STATUS.size
MAXIMUM_FRAME_SIZE = TRACK_MESSAGE_MAXIMUM_SIZE


class ProtocolError(ValueError):
    """메시지 형식이 프로토콜과 맞지 않는다."""


@dataclass(frozen=True)
class Header:
    message_id: int
    version: int
    block_size: int
    time_sec: int
    time_nsec: int
    source_id: int
    dest_id: int


@dataclass(frozen=True)
class Beam:
    beam_type: int
    timestamp_ms: int
    beam_id: int
    command_count: int
    azimuth_deg: float
    elevation_deg: float
    azimuth_width_deg: float
    elevation_width_deg: float


@dataclass(frozen=True)
class Track:
    track_id: int
    status: int
    slant_range_km: float
    ground_range_km: float
    azimuth_deg: float
    elevation_deg: float
    doppler_mps: float
    latitude_deg: float
    longitude_deg: float
    altitude_km: float
    speed_mps: float
    heading_deg: float
    flight_path_angle_deg: float
    timestamp_ms: int


@dataclass(frozen=True)
class TrackList:
    tracks: List[Track]


@dataclass(frozen=True)
class RadarAttitude:
    latitude_deg: float
    longitude_deg: float
    altitude_km: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class MonitorStatus:
    processor_time_ms: int
    time_scale: float
    paused: bool
    operator_console_connected: bool
    scenario_simulator_connected: bool


Message = Union[Beam, TrackList, RadarAttitude, MonitorStatus]


def decode_header(data: bytes) -> Header:
    if len(data) < HEADER.size:
        raise ProtocolError("header is too short")
    message_id, version, _, block_size, time_sec, time_nsec, source_id, dest_id, _ = (
        HEADER.unpack_from(data)
    )
    return Header(message_id, version, block_size, time_sec, time_nsec, source_id, dest_id)


def is_known_message_id(message_id: int) -> bool:
    return message_id in MessageId._value2member_map_


def decode(frame: bytes) -> Message:
    """완성된 프레임 하나를 메시지로 디코딩한다."""
    header = decode_header(frame)
    if header.block_size != len(frame):
        raise ProtocolError("block size does not match frame size")

    if header.message_id == MessageId.BEAM_TRANSMIT:
        return _decode_beam(frame)
    if header.message_id == MessageId.TRACK_INFORMATION:
        return _decode_tracks(frame)
    if header.message_id == MessageId.RADAR_ATTITUDE_TO_CONSOLE:
        return _decode_attitude(frame)
    if header.message_id == MessageId.MONITOR_STATUS:
        return _decode_status(frame)
    raise ProtocolError(f"unsupported message id 0x{header.message_id:04X}")


def _require_size(frame: bytes, size: int) -> None:
    if len(frame) != size:
        raise ProtocolError(f"expected {size} bytes, got {len(frame)}")


def _decode_beam(frame: bytes) -> Beam:
    _require_size(frame, BEAM_TX_MESSAGE_SIZE)
    return Beam(*BEAM_TX.unpack_from(frame, HEADER.size))


def _decode_tracks(frame: bytes) -> TrackList:
    if len(frame) < HEADER.size + TRACK_HEADER.size:
        raise ProtocolError("track message is too short")
    (count,) = TRACK_HEADER.unpack_from(frame, HEADER.size)
    if count > TRACK_MESSAGE_MAXIMUM_TRACKS:
        raise ProtocolError("too many tracks")
    _require_size(frame, HEADER.size + TRACK_HEADER.size + TRACK.size * count)

    tracks = []
    offset = HEADER.size + TRACK_HEADER.size
    for _ in range(count):
        values = TRACK.unpack_from(frame, offset)
        track_id, status, _mem_flag = values[0:3]
        floats = values[3:14]
        timestamp_ms = values[14]
        tracks.append(Track(track_id, status, *floats, timestamp_ms))
        offset += TRACK.size
    return TrackList(tracks)


def _decode_attitude(frame: bytes) -> RadarAttitude:
    _require_size(frame, RADAR_ATTITUDE_MESSAGE_SIZE)
    return RadarAttitude(*RADAR_ATTITUDE.unpack_from(frame, HEADER.size))


def _decode_status(frame: bytes) -> MonitorStatus:
    _require_size(frame, MONITOR_STATUS_MESSAGE_SIZE)
    processor_time_ms, time_scale, paused, console, simulator = MONITOR_STATUS.unpack_from(
        frame, HEADER.size
    )
    return MonitorStatus(processor_time_ms, time_scale, bool(paused), bool(console), bool(simulator))


class MessageFramer:
    """TCP 바이트 흐름을 헤더의 block_size 단위 프레임으로 나눈다."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
        """받은 바이트를 추가하고 완성된 프레임을 반환한다.

        헤더가 잘못되면 ProtocolError를 던진다. 경계를 다시 찾을 수 없으므로
        호출자는 연결을 끊어야 한다.
        """
        self._buffer.extend(data)
        frames = []
        while len(self._buffer) >= HEADER.size:
            header = decode_header(bytes(self._buffer[: HEADER.size]))
            if not is_known_message_id(header.message_id):
                raise ProtocolError(f"unknown message id 0x{header.message_id:04X}")
            if header.block_size < HEADER.size or header.block_size > MAXIMUM_FRAME_SIZE:
                raise ProtocolError(f"invalid frame size {header.block_size}")
            if len(self._buffer) < header.block_size:
                break
            frames.append(bytes(self._buffer[: header.block_size]))
            del self._buffer[: header.block_size]
        return frames
