#!/usr/bin/env python3
"""Summarize a live run without assuming camera IDs or matching rewritten PTS."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import subprocess

from live_run import save_json

NONE = (1 << 64) - 1


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return {"samples": 0}
    def percentile(fraction):
        return values[max(0, math.ceil(fraction * len(values)) - 1)]
    return {"samples": len(values), "p50": percentile(.5), "p95": percentile(.95),
            "p99": percentile(.99), "max": values[-1]}


def analyse_trace(path, stop_ns=None):
    stages = defaultdict(list)
    dropped, complete, malformed = 0, False, 0
    with path.open() as stream:
        for line in stream:
            if line.startswith("#"):
                try:
                    fields = dict(item.split("=") for item in line[1:].split())
                    dropped = int(fields["dropped"])
                    complete = fields["complete"] == "1"
                except (ValueError, KeyError):
                    malformed += 1
                    complete = False
                continue
            if line.startswith("stage,"):
                continue
            try:
                stage, when, pts, offset, size = next(csv.reader([line]))
                stages[stage].append((int(when), int(pts), int(offset), int(size)))
            except (ValueError, csv.Error):
                malformed += 1
    result = {"writer_complete": complete, "dropped_trace_records": dropped,
              "malformed_rows": malformed, "stages": {}, "matched_durations_ms": {}}
    for stage, events in stages.items():
        events.sort()
        timestamps = [event[0] for event in events]
        span = (timestamps[-1] - timestamps[0]) / 1e9 if len(events) > 1 else 0
        counts = Counter(event[1] for event in events if event[1] != NONE)
        result["stages"][stage] = {
            "buffers": len(events), "span_s": span,
            "fps_over_observed_span": (len(events) - 1) / span if span > 0 else None,
            "first_monotonic_ns": timestamps[0], "last_monotonic_ns": timestamps[-1],
            "tail_to_stop_s": max(0, (stop_ns - timestamps[-1]) / 1e9) if stop_ns else None,
            "gaps_ms": distribution((b-a)/1e6 for a, b in zip(timestamps, timestamps[1:])),
            "missing_pts": sum(event[1] == NONE for event in events),
            "duplicate_pts": sum(n-1 for n in counts.values()),
        }
    for name, start, end in (("conversion", "convert_in", "convert_out"),
                              ("inference", "infer_in", "infer_out"),
                              ("osd", "osd_in", "osd_out"),
                              ("postconversion", "post_in", "post_out"),
                              ("encoding", "encode_in", "encode_out"),
                              ("source_to_encoder", "source", "encode_out")):
        left, right = stages[start], stages[end]
        lc = Counter(e[1] for e in left)
        rc = Counter(e[1] for e in right)
        # Never pair by ordinal or guess a PTS offset; ambiguous keys stay unmatched.
        starts = {e[1]: e[0] for e in left if e[1] != NONE and lc[e[1]] == 1}
        durations = [(e[0] - starts[e[1]]) / 1e6 for e in right
                     if e[1] in starts and rc[e[1]] == 1 and e[0] >= starts[e[1]]]
        summary = distribution(durations)
        summary.update(input_buffers=len(left), output_buffers=len(right),
                       unmatched_inputs=len(left)-len(durations),
                       unmatched_outputs=len(right)-len(durations))
        result["matched_durations_ms"][name] = summary
    return result


def analyse_video(path):
    command = ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_streams",
               "-show_frames", "-show_entries",
               "stream=codec_name,profile,width,height:frame=best_effort_timestamp_time,key_frame,pict_type",
               "-of", "json", str(path)]
    try:
        run = subprocess.run(command, capture_output=True, text=True, timeout=180)
        if run.returncode != 0:
            return {"error": run.stderr, "returncode": run.returncode}
        data = json.loads(run.stdout)
        frames = data.get("frames", [])
        timestamps = [float(f["best_effort_timestamp_time"]) for f in frames
                      if "best_effort_timestamp_time" in f]
        with path.with_suffix(".frames.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(["frame", "receiver_pts_s", "key_frame", "picture_type"])
            for index, frame in enumerate(frames):
                writer.writerow([index, frame.get("best_effort_timestamp_time", ""),
                                 frame.get("key_frame", ""), frame.get("pict_type", "")])
        return {"frames": len(frames), "streams": data.get("streams", []),
                "decode_errors": run.stderr,
                "pts_gaps_ms": distribution((b-a)*1000 for a, b in zip(timestamps, timestamps[1:]) if b >= a),
                "backwards_pts": sum(b < a for a, b in zip(timestamps, timestamps[1:]))}
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def analyse_resources(path, manifest):
    samples = []
    if not path.exists():
        return {"samples": 0}
    for line in path.read_text().splitlines():
        try:
            samples.append(json.loads(line))
        except ValueError:
            continue
    cpu = []
    pid = manifest.get("application_pid")
    ticks = manifest.get("clock_ticks_per_second")
    previous = None
    for sample in samples:
        stat = sample.get(f"/proc/{pid}/stat")
        if not stat or not ticks:
            continue
        fields = stat.rsplit(")", 1)[1].split()
        total = int(fields[11]) + int(fields[12])  # utime/stime, fields 14/15
        now = sample["monotonic_ns"] / 1e9
        if previous and now > previous[0]:
            cpu.append(100 * (total-previous[1]) / ticks / (now-previous[0]))
        previous = now, total
    return {"samples": len(samples), "process_cpu_percent_one_core_is_100": distribution(cpu),
            "minimum_disk_free_bytes": min((s["disk_free_bytes"] for s in samples), default=None)}


def analyse(directory, video=True):
    manifest = json.loads((directory / "manifest.json").read_text())
    result = {"schema": 1, "state": manifest.get("state"),
              "stop_reason": manifest.get("stop_reason"), "shutdown": manifest.get("shutdown", {}),
              "pipeline": {}, "recordings": {}, "warnings": [],
              "timing_contract": "Host pad traversal; same unique PTS only. Not GPU kernel, camera acquisition, or glass-to-glass latency. Receiver PTS has a separate origin."}
    for camera in manifest["cameras"]:
        trace = directory / f"pipeline-{camera}.csv"
        if trace.exists():
            info = analyse_trace(trace, manifest.get("stop_requested_monotonic_ns"))
            result["pipeline"][camera] = info
            if not info["writer_complete"] or info["dropped_trace_records"] or info["malformed_rows"]:
                result["warnings"].append(f"{camera}: trace incomplete or records lost")
            for stage in ("source", "infer_out", "encode_out"):
                stats = info["stages"].get(stage)
                if not stats:
                    result["warnings"].append(f"{camera}: no {stage} buffers")
                elif stats["tail_to_stop_s"] is not None and stats["tail_to_stop_s"] > 2:
                    result["warnings"].append(f"{camera}: {stage} stopped {stats['tail_to_stop_s']:.2f}s before stop request")
        elif manifest.get("trace"):
            result["warnings"].append(f"{camera}: missing pipeline trace; rebuild application")
        segments = sorted(directory.glob(f"{camera}-*.mkv"))
        if not segments:
            result["warnings"].append(f"{camera}: no recorded video")
        if video:
            for segment in segments:
                info = analyse_video(segment)
                result["recordings"][segment.name] = info
                if info.get("error") or info.get("decode_errors") or not info.get("frames"):
                    result["warnings"].append(f"{segment.name}: missing frames or decode failure; inspect recording diagnostics")
    for name, status in result["shutdown"].items():
        if (name == "tegrastats" and status.get("returncode") == -2
                and status.get("signals") == ["SIGINT"] and not status.get("forced")):
            continue  # Expected recorder-requested termination of the optional collector.
        if status.get("forced") or status.get("returncode") != 0:
            result["warnings"].append(f"{name}: forced or nonzero exit; not clean shutdown")
    result["resources"] = analyse_resources(directory / "resources.jsonl", manifest)
    result["warnings"].append("Probe counts are not camera block IDs. No claim of zero acquisition loss or detection accuracy.")
    save_json(directory / "summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--skip-video", action="store_true")
    args = parser.parse_args()
    result = analyse(args.directory.resolve(), not args.skip_video)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
