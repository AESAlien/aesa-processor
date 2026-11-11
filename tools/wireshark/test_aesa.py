"""Exercise the Lua dissector in TShark with synthetic TCP captures (stdlib only)."""

import argparse
import csv
import io
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest


SCRIPT_PATH = Path(__file__).with_name("aesa.lua").resolve()
TSHARK_PATH = shutil.which("tshark") or r"C:\Program Files\Wireshark\tshark.exe"


def message(message_id, payload=b"", *, block_size=None, version=0):
    size = 20 + len(payload) if block_size is None else block_size
    return struct.pack("<HBBIIIBBH", message_id, version, 0, size, 12, 34, 1, 2, 0) + payload


def beam():
    return struct.pack("<B3xIII4f28x", 3, 1234, 21, 7, 10.5, -4.5, 3.25, 2.75)


def plot():
    return struct.pack("<I5f", 99, 12.5, 15.5, 6.25, -120.5, 30.5)


def track():
    return struct.pack(
        "<HBB11fIII28x", 42, 2, 0,
        12.5, 10.25, 15.5, 6.25, -120.5, 37.5, 127.25, 2.5, 250.75, 45.5, -2.25,
        5678, 21, 7,
    )


def sample_messages():
    attitude = struct.pack("<6d", 37.5, 127.25, 1.25, -12.5, 6.25, 45.5)
    return [
        message(0xCA01, b"\1\0\0\0"),
        message(0xAB01, beam()),
        message(0xBA01, beam() + struct.pack("<H2x", 1) + plot()),
        message(0xAC01, struct.pack("<H2x", 1) + track()),
        message(0xAC02, attitude),
        message(0xBA02, attitude),
        message(0xAC03, struct.pack("<H2x100H", 1, 42, *([0] * 99))),
    ]


def checksum(data):
    padded = data + b"\0" if len(data) % 2 else data
    total = sum(struct.unpack(f"!{len(padded) // 2}H", padded))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


class Capture:
    def __init__(self):
        self.packets = []

    def packet(self, payload, sequence, acknowledgement, flags, client_port, server_port, reverse=False):
        source = bytes([192, 168, 10, 10])
        destination = bytes([192, 168, 10, 20])
        source_port, destination_port = client_port, server_port
        if reverse:
            source, destination = destination, source
            source_port, destination_port = destination_port, source_port
        tcp = struct.pack("!HHIIBBHHH", source_port, destination_port,
                          sequence, acknowledgement, 0x50, flags, 65535, 0, 0)
        pseudo_header = source + destination + struct.pack("!BBH", 0, 6, len(tcp) + len(payload))
        tcp = tcp[:16] + struct.pack("!H", checksum(pseudo_header + tcp + payload)) + tcp[18:]
        ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 40 + len(payload), len(self.packets),
                         0, 64, 6, 0, source, destination)
        ip = ip[:10] + struct.pack("!H", checksum(ip)) + ip[12:]
        ethernet = bytes.fromhex("00112233445566778899aabb0800")
        self.packets.append(ethernet + ip + tcp + payload)

    def stream(self, chunks, *, server_port=3200, reverse=False):
        client_port = 3100 + len(self.packets)
        self.packet(b"", 100, 0, 0x02, client_port, server_port)
        self.packet(b"", 500, 101, 0x12, client_port, server_port, True)
        self.packet(b"", 101, 501, 0x10, client_port, server_port)
        sequence, acknowledgement = (501, 101) if reverse else (101, 501)
        for chunk in chunks:
            self.packet(chunk, sequence, acknowledgement, 0x18, client_port, server_port, reverse)
            sequence += len(chunk)

    def write(self, path):
        with Path(path).open("wb") as output:
            output.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
            for index, packet in enumerate(self.packets):
                output.write(struct.pack("<IIII", 1700000000, index * 1000, len(packet), len(packet)))
                output.write(packet)


class DissectorTest(unittest.TestCase):
    def read_capture(self, capture, *fields, options=()):
        with tempfile.TemporaryDirectory(prefix="aesa_wireshark_") as directory:
            path = Path(directory) / "capture.pcap"
            capture.write(path)
            environment = dict(os.environ, WIRESHARK_CONFIG_DIR=directory)
            command = [TSHARK_PATH, "-n", "-2", "-r", str(path), "-X", f"lua_script:{SCRIPT_PATH}",
                       *options, "-Y", "aesa", "-T", "fields", "-E", "occurrence=a"]
            output_fields = [*fields, "_ws.expert.message"]
            for name in output_fields:
                command.extend(["-e", name])
            result = subprocess.run(command, env=environment, capture_output=True, text=True,
                                    encoding="utf-8", errors="replace", timeout=30, check=True)
            self.assertNotIn("Lua", result.stderr, result.stderr)
            self.assertNotIn("Lua Error", result.stdout)
            return [dict(zip(output_fields, row)) for row in csv.reader(io.StringIO(result.stdout), delimiter="\t")]

    def test_all_message_types_and_numeric_fields(self):
        capture = Capture()
        for index, data in enumerate(sample_messages()):
            capture.stream([data], reverse=index % 2 == 1)
        rows = self.read_capture(capture, "aesa.message_id", "aesa.operation", "aesa.beam.id",
                                 "aesa.beam.elevation_deg", "aesa.plot.id", "aesa.plot.doppler_mps",
                                 "aesa.track.id", "aesa.track.timestamp_ms", "aesa.track.flight_path_angle_deg",
                                 "aesa.attitude.latitude_deg", "aesa.attitude.altitude_km", "aesa.deletion.id")
        self.assertEqual([row["aesa.message_id"] for row in rows],
                         ["0xca01", "0xab01", "0xba01", "0xac01", "0xac02", "0xba02", "0xac03"])
        self.assertTrue(all(not row["_ws.expert.message"] for row in rows))
        self.assertEqual(rows[0]["aesa.operation"], "1")
        self.assertEqual(rows[1]["aesa.beam.id"], "21")
        self.assertEqual(float(rows[1]["aesa.beam.elevation_deg"]), -4.5)
        self.assertEqual(rows[2]["aesa.plot.id"], "99")
        self.assertEqual(float(rows[2]["aesa.plot.doppler_mps"]), -120.5)
        self.assertEqual(rows[3]["aesa.track.id"], "42")
        self.assertEqual(rows[3]["aesa.track.timestamp_ms"], "5678")
        self.assertEqual(float(rows[3]["aesa.track.flight_path_angle_deg"]), -2.25)
        self.assertEqual(float(rows[4]["aesa.attitude.latitude_deg"]), 37.5)
        self.assertEqual(float(rows[5]["aesa.attitude.altitude_km"]), 1.25)
        self.assertEqual(rows[6]["aesa.deletion.id"], "42")

    def test_split_headers_bodies_and_coalesced_messages(self):
        first, second = sample_messages()[:2]
        for split in (1, 4, 7, 19, 20, 22):
            with self.subTest(split=split):
                capture = Capture()
                capture.stream([first[:split], first[split:] + second + first[:3], first[3:]])
                rows = self.read_capture(capture, "aesa.message_id", "aesa.beam.id")
                ids = ",".join(row["aesa.message_id"] for row in rows)
                self.assertEqual(ids, "0xca01,0xab01,0xca01")
                self.assertTrue(all(not row["_ws.expert.message"] for row in rows))

    def test_zero_and_maximum_counts(self):
        for count in (0, 100):
            with self.subTest(count=count):
                capture = Capture()
                plot_message = message(0xBA01, beam() + struct.pack("<H2x", count) + plot() * count)
                track_message = message(0xAC01, struct.pack("<H2x", count) + track() * count)
                capture.stream([plot_message[:50], plot_message[50:], track_message[:1500], track_message[1500:]])
                rows = self.read_capture(capture, "aesa.plot.count", "aesa.track.count", "aesa.track.id")
                self.assertTrue(all(not row["_ws.expert.message"] for row in rows))
                self.assertEqual(rows[0]["aesa.plot.count"], str(count))
                track_row = next(row for row in rows if row["aesa.track.count"])
                self.assertEqual(track_row["aesa.track.count"], str(count))
                self.assertEqual(track_row["aesa.track.id"], ",".join(["42"] * count))

    def test_invalid_lengths_counts_and_unknown_id(self):
        cases = [
            (message(0xCA01, block_size=0), "block_size must"),
            (message(0xCA01, block_size=19), "block_size must"),
            (message(0xCA01, block_size=65537), "block_size must"),
            (message(0xCA01), "Expected 24 bytes"),
            (message(0xAB01), "Expected 80 bytes"),
            (message(0xBA02), "Expected 68 bytes"),
            (message(0xAC01), "Missing item count"),
            (message(0xBA01, beam() + struct.pack("<H2x", 101)), "exceeds 100"),
            (message(0xAC01, struct.pack("<H2x", 1)), "Expected 112 bytes"),
            (message(0xAC03, struct.pack("<H2x", 101)), "exceeds 100"),
            (message(0xFFFF, b"unknown"), "Unknown AESA message ID"),
        ]
        for data, expected in cases:
            with self.subTest(expected=expected):
                capture = Capture()
                capture.stream([data])
                rows = self.read_capture(capture, "aesa.message_id")
                self.assertEqual(len(rows), 1)
                self.assertIn(expected, rows[0]["_ws.expert.message"])

    def test_custom_port_preference_and_decode_as(self):
        capture = Capture()
        capture.stream([sample_messages()[0]], server_port=43200)
        self.assertEqual(self.read_capture(capture, "aesa.operation"), [])
        for options in (("-o", "aesa.tcp_ports:43200"), ("-d", "tcp.port==43200,aesa")):
            with self.subTest(options=options):
                rows = self.read_capture(capture, "aesa.operation", options=options)
                self.assertEqual(rows[0]["aesa.operation"], "1")
        default_capture = Capture()
        default_capture.stream([sample_messages()[0]])
        self.assertEqual(self.read_capture(default_capture, "aesa.operation", options=("-o", "aesa.tcp_ports:43200")), [])

    def test_compact_deletion_and_version_one(self):
        capture = Capture()
        capture.stream([message(0xAC03, struct.pack("<H2x2H", 2, 42, 43), version=1)])
        rows = self.read_capture(capture, "aesa.version", "aesa.deletion.id")
        self.assertEqual(rows[0]["aesa.version"], "1")
        self.assertEqual(rows[0]["aesa.deletion.id"], "42,43")
        self.assertFalse(rows[0]["_ws.expert.message"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tshark", default=TSHARK_PATH)
    parser.add_argument("--write-sample", type=Path, help="Write a sample pcap instead of running tests")
    arguments = parser.parse_args()
    TSHARK_PATH = arguments.tshark
    if arguments.write_sample:
        sample = Capture()
        for index, data in enumerate(sample_messages()):
            sample.stream([data], reverse=index % 2 == 1)
        sample.write(arguments.write_sample)
        print(arguments.write_sample.resolve())
    else:
        unittest.main(argv=[__file__], verbosity=2)
