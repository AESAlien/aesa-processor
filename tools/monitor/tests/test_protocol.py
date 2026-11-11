import re
import struct
import unittest
from pathlib import Path

from monitor import protocol
from monitor.protocol import (
    HEADER,
    Beam,
    MessageFramer,
    MessageId,
    MonitorStatus,
    ProtocolError,
    RadarAttitude,
    TrackList,
    decode,
)

PROTOCOL_DIRECTORY = Path(__file__).resolve().parents[3] / "src" / "comm" / "src" / "protocol"


def cpp_sizes():
    """C++ 프로토콜 헤더의 static_assert에서 구조체 크기를 읽는다."""
    sizes = {}
    pattern = re.compile(r"static_assert\(sizeof\((\w+)\)\s*==\s*(\d+)")
    for path in PROTOCOL_DIRECTORY.glob("ST_*.hpp"):
        for name, size in pattern.findall(path.read_text(encoding="utf-8")):
            sizes[name] = int(size)
    return sizes


def header(message_id, block_size):
    return HEADER.pack(message_id, 1, 0, block_size, 0, 0, 0, 0, 0)


def beam_frame(beam_type=1, timestamp=1234, beam_id=21, azimuth=10.5, elevation=-4.5):
    body = protocol.BEAM_TX.pack(beam_type, timestamp, beam_id, 7, azimuth, elevation, 3.0, 3.0)
    return header(MessageId.BEAM_TRANSMIT, protocol.BEAM_TX_MESSAGE_SIZE) + body


def status_frame(time_ms=42310, scale=0.1, paused=1, console=1, simulator=0):
    body = protocol.MONITOR_STATUS.pack(time_ms, scale, paused, console, simulator)
    return header(MessageId.MONITOR_STATUS, protocol.MONITOR_STATUS_MESSAGE_SIZE) + body


def track_frame(track_ids):
    size = HEADER.size + protocol.TRACK_HEADER.size + protocol.TRACK.size * len(track_ids)
    data = header(MessageId.TRACK_INFORMATION, size) + protocol.TRACK_HEADER.pack(len(track_ids))
    for track_id in track_ids:
        data += protocol.TRACK.pack(track_id, 2, 0, 12.5, 10.25, 15.5, 6.25, -120.5,
                                    37.5, 127.25, 2.5, 250.75, 45.5, -2.25, 5678, 0, 0)
    return data


class StructSizeTest(unittest.TestCase):
    # 파이썬 배치가 C++ 구조체 크기와 다르면 모든 메시지가 어긋나므로 크기를 대조한다.
    def test_sizes_match_cpp_static_asserts(self):
        sizes = cpp_sizes()
        self.assertEqual(sizes["ST_MsgHeader"], HEADER.size)
        self.assertEqual(sizes["ST_BeamTx"], protocol.BEAM_TX.size)
        self.assertEqual(sizes["ST_BeamTxMsg"], protocol.BEAM_TX_MESSAGE_SIZE)
        self.assertEqual(sizes["ST_TrkHeader"], protocol.TRACK_HEADER.size)
        self.assertEqual(sizes["ST_Trk"], protocol.TRACK.size)
        self.assertEqual(sizes["ST_TrkMsg"], protocol.TRACK_MESSAGE_MAXIMUM_SIZE)
        self.assertEqual(sizes["ST_RadarAttMsg"], protocol.RADAR_ATTITUDE_MESSAGE_SIZE)
        self.assertEqual(sizes["ST_MonitorStatusMsg"], protocol.MONITOR_STATUS_MESSAGE_SIZE)

    def test_message_ids_match_cpp_header(self):
        text = (PROTOCOL_DIRECTORY / "ST_MsgHeader.hpp").read_text(encoding="utf-8")
        cpp_ids = {name: int(value, 16) for name, value in re.findall(r"(\w+) = (0x[0-9A-Fa-f]+)", text)}
        for message_id in MessageId:
            self.assertEqual(cpp_ids[message_id.name], message_id.value, message_id.name)


class DecodeTest(unittest.TestCase):
    def test_decodes_beam(self):
        beam = decode(beam_frame())
        self.assertIsInstance(beam, Beam)
        self.assertEqual(beam.beam_type, 1)
        self.assertEqual(beam.timestamp_ms, 1234)
        self.assertEqual(beam.beam_id, 21)
        self.assertAlmostEqual(beam.azimuth_deg, 10.5)
        self.assertAlmostEqual(beam.elevation_deg, -4.5)

    def test_decodes_status(self):
        status = decode(status_frame())
        self.assertEqual(
            status,
            MonitorStatus(42310, struct.unpack("<f", struct.pack("<f", 0.1))[0], True, True, False),
        )

    def test_decodes_tracks_including_empty_list(self):
        tracks = decode(track_frame([42, 7]))
        self.assertIsInstance(tracks, TrackList)
        self.assertEqual([track.track_id for track in tracks.tracks], [42, 7])
        self.assertEqual(tracks.tracks[0].timestamp_ms, 5678)
        self.assertAlmostEqual(tracks.tracks[0].slant_range_km, 12.5)
        self.assertEqual(decode(track_frame([])).tracks, [])

    def test_decodes_attitude(self):
        body = protocol.RADAR_ATTITUDE.pack(37.5, 127.25, 1.25, -12.5, 6.25, 45.5)
        attitude = decode(header(MessageId.RADAR_ATTITUDE_TO_CONSOLE,
                                 protocol.RADAR_ATTITUDE_MESSAGE_SIZE) + body)
        self.assertEqual(attitude, RadarAttitude(37.5, 127.25, 1.25, -12.5, 6.25, 45.5))

    def test_rejects_size_mismatch(self):
        frame = beam_frame()
        with self.assertRaises(ProtocolError):
            decode(frame[:-1])
        bad_count = bytearray(track_frame([1]))
        struct.pack_into("<H", bad_count, HEADER.size, 2)
        with self.assertRaises(ProtocolError):
            decode(bytes(bad_count))


class FramerTest(unittest.TestCase):
    def test_reassembles_frames_split_across_reads(self):
        stream = beam_frame(beam_id=1) + status_frame() + beam_frame(beam_id=2)
        framer = MessageFramer()
        frames = []
        for index in range(0, len(stream), 7):
            frames.extend(framer.feed(stream[index:index + 7]))
        self.assertEqual([len(frame) for frame in frames],
                         [protocol.BEAM_TX_MESSAGE_SIZE, protocol.MONITOR_STATUS_MESSAGE_SIZE,
                          protocol.BEAM_TX_MESSAGE_SIZE])
        self.assertEqual(decode(frames[2]).beam_id, 2)

    def test_rejects_unknown_message_id_and_invalid_size(self):
        with self.assertRaises(ProtocolError):
            MessageFramer().feed(header(0xFFFF, HEADER.size))
        with self.assertRaises(ProtocolError):
            MessageFramer().feed(header(MessageId.BEAM_TRANSMIT, HEADER.size - 1))
        with self.assertRaises(ProtocolError):
            MessageFramer().feed(header(MessageId.TRACK_INFORMATION, protocol.MAXIMUM_FRAME_SIZE + 1))


if __name__ == "__main__":
    unittest.main()
