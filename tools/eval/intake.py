"""Preserve raw input bytes before selecting a processor-time interval."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def snapshot(source, destination):
    source = Path(source).resolve()
    destination = Path(destination)
    data = source.read_bytes()
    # Exclusive create prevents replacing a previous experiment snapshot.
    with destination.open("xb") as stream:
        stream.write(data)
    return {"source": str(source), "snapshot": str(destination.resolve()), "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--ini", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--start-ms", type=int, required=True)
    parser.add_argument("--end-ms", type=int, required=True)
    parser.add_argument("--conditions", required=True, help="Human-confirmed scenario and environment settings")
    args = parser.parse_args()
    if args.start_ms > args.end_ms:
        parser.error("start-ms must not exceed end-ms")
    args.destination.mkdir(parents=True, exist_ok=False)
    metadata = {"captured_utc": datetime.now(timezone.utc).isoformat(),
                "start_ms": args.start_ms, "end_ms": args.end_ms,
                "conditions": args.conditions, "files": {}}
    for name, source, filename in (("csv", args.csv, "detections.csv"), ("log", args.log, "processor.log"),
                                   ("ini", args.ini, "scenario.ini")):
        metadata["files"][name] = snapshot(source, args.destination / filename)
    (args.destination / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
