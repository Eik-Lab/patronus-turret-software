#!/usr/bin/env python3
"""Convert one telemetry gimbal to replay events and its original configuration.

Usage: telemetry_replay.py RUN_DIR OUTPUT_DIR --gimbal 0
The output directory must not exist. Historical CSVs are rounded and do not
record exact call ordering; this is an approximate reconstruction, not a
bit-exact replay of the hardware run. The generated event stream is repeatable.
"""

import argparse
import csv
import math
from pathlib import Path
import re
import shlex
import sys


def convert(run, output, gimbal, control_only=False):
    text = (run / "run.txt").read_text()
    config = re.search(r"^--- config [^\n]* ---\n(.*?)^--- end config ---", text, re.M | re.S)
    note = re.search(rf"^gimbal {gimbal} (.+)$", text, re.M)
    if config is None or note is None:
        raise ValueError("Missing recorded configuration or gimbal metadata")
    tokens = note[1].split()
    metadata = dict(zip(tokens[::2], tokens[1::2]))
    dropped = re.search(r"dropped_frames (\d+) dropped_ticks (\d+)", text)
    if control_only and (dropped is None or int(dropped[2]) != 0):
        raise ValueError("Control replay needs a completed run with no dropped ticks")
    events = []
    approximate_aims = 0

    def append(time, priority, line):
        if not math.isfinite(time) or time < 0:
            raise ValueError("Invalid event time")
        events.append((time, priority, line))

    with (run / "ticks.csv").open(newline="") as source:
        for row in csv.DictReader(source):
            if int(row["gimbal"]) != gimbal:
                continue
            if control_only:
                if row["state"] == "3":
                    raise ValueError("Control replay cannot reconstruct manual mode")
                elapsed = int(row["control_elapsed_ns"])
                append(elapsed, 0, f"control {elapsed} {row['raw_detection']} "
                                  f"{row['estimate_available']} {row['driving']} {row['reached_home']} "
                                  f"{row.get('pose_usable', '1')}")
                continue
            time = float(row["t_s"])
            if row.get("pose_usable") == "0":
                raise ValueError("Filter replay cannot reconstruct pose-fault resets; use --control")
            if int(row["driving"]):
                append(time, 0, f"pose {time:.17g} {row['pan']} {row['tilt']}")
            if int(row["aim"]):
                aim_time = float(row.get("aim_s") or 0)
                if aim_time <= 0:
                    # Old logs only have the midpoint of the preceding pose read.
                    # Do not invent a later timestamp to force a particular output.
                    aim_time = time
                    approximate_aims += 1
                append(aim_time, 2, f"aim {aim_time:.17g}")

    if not control_only:
        with (run / "frames.csv").open(newline="") as source:
            for row in csv.DictReader(source):
                if int(row["gimbal"]) != gimbal:
                    continue
                time = float(row["step_s"])
                detected = int(row["det"])
                box = "0 0 0 0 0"
                if detected:
                    width, height = float(row["det_w"]), float(row["det_h"])
                    left = float(row["det_cx"]) - width / 2
                    top = float(row["det_cy"]) - height / 2
                    box = f"{left:.17g} {top:.17g} {width:.17g} {height:.17g} {row['det_conf']}"
                append(time, 1, f"frame {time:.17g} {row['timestamp_s']} {row['capture_s']} "
                               f"{row['captured_at_s']} {row['frame_id']} {detected} {box}")
    if not events:
        raise ValueError("No events for selected gimbal")
    # Stable tie ordering: pose, frame, aim. Rounded CSV timestamps can tie.
    events.sort(key=lambda event: event[:2])
    output.mkdir(parents=True, exist_ok=False)
    (output / "config.ini").write_text(config[1])
    (output / "events.txt").write_text("\n".join(event[2] for event in events) + "\n")
    if not control_only:
        print("WARNING: CSV reconstruction is approximate (rounded values and timestamps).", file=sys.stderr)
    if approximate_aims:
        print(f"WARNING: {approximate_aims} aim calls use old pose timestamps; "
              "do not compare their output to live commands.", file=sys.stderr)
    if dropped is None or any(int(n) for n in dropped.groups()):
        print("WARNING: telemetry completeness is unknown or records were dropped.", file=sys.stderr)
    print(shlex.join(["build/replay/tracking_replay", str(output / "config.ini"),
                      metadata["camera"], str(gimbal), metadata["home_tilt"],
                      str(output / "events.txt"), "100"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--gimbal", type=int, default=0)
    parser.add_argument("--control", action="store_true",
                        help="replay recorded state-machine inputs (requires new, complete telemetry)")
    args = parser.parse_args()
    try:
        convert(args.run, args.output, args.gimbal, args.control)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Replay conversion: {error}\n")
