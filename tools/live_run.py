#!/usr/bin/env python3
"""Prepare or record a motor-disabled live run. No cameras open without --execute."""
import argparse
import configparser
import datetime
import glob
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def read_config(path):
    config = configparser.ConfigParser(interpolation=None, strict=True)
    with path.open() as stream:
        config.read_file(stream)
    return config


def command_output(command, cwd=ROOT):
    try:
        run = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=15)
        return {"command": command, "returncode": run.returncode,
                "stdout": run.stdout, "stderr": run.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": command, "error": str(error)}


def prepare(config_path, binary, output, mode, port, trace=True):
    """Snapshot inputs and create safe config; never launch application or inspect cameras."""
    output.mkdir(parents=True, exist_ok=False)
    config = read_config(config_path)
    (output / "source-system.ini").write_bytes(config_path.read_bytes())
    manifest = {"schema": 1, "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "monotonic_ns": time.monotonic_ns(), "platform": platform.platform(),
                "binary": {"path": str(binary), "sha256": digest(binary)},
                "source_config": str(config_path), "mode": mode, "trace": trace,
                "assets": [], "cameras": [], "state": "prepared",
                "git": command_output(["git", "rev-parse", "HEAD"]),
                "git_status": command_output(["git", "status", "--porcelain"])}
    patch = command_output(["git", "diff", "HEAD", "--binary"])
    (output / "source.patch").write_text(patch.get("stdout", ""))
    manifest["source_sha256"] = {
        str(path.relative_to(ROOT)): digest(path)
        for folder in ("src", "tools", "config")
        for path in sorted((ROOT / folder).rglob("*"))
        if path.is_file() and path.suffix in (".cpp", ".h", ".py", ".ini", ".txt")
    }
    for section in ("general", "sensor", "commands", "telemetry"):
        if not config.has_section(section):
            config.add_section(section)
    config["general"].update(mode=mode, tracking="false", log_level="info")
    config["sensor"]["enabled"] = "false"
    config["commands"]["enabled"] = "false"
    config["telemetry"].update(enabled="true", dir=str(output / "telemetry"))
    for section in config.sections():
        if section == "can" or section.startswith("can_"):
            config[section]["calibrate_on_boot"] = "false"
    for index, camera in enumerate(("rgb", "mono")):
        active = mode == "both" or mode == camera
        config[camera]["enabled"] = "true" if active else "false"
        if not active:
            continue
        infer_path = Path(config[camera]["infer_config"])
        if not infer_path.is_absolute():
            infer_path = ROOT / infer_path
        infer_path = infer_path.resolve(strict=True)
        if infer_path.suffix.lower() in (".yml", ".yaml"):
            raise ValueError("Recorder requires an INI inference configuration")
        infer = read_config(infer_path)
        properties = infer["property"]
        if not properties.get("model-engine-file"):
            raise ValueError("A prebuilt engine is required; recording never builds engines")
        for key in ("model-engine-file", "onnx-file", "custom-lib-path", "labelfile-path"):
            if key not in properties:
                continue
            asset = Path(properties[key])
            if not asset.is_absolute():
                asset = infer_path.parent / asset
            asset = asset.resolve()
            # ONNX is provenance only; it is removed below to prevent fallback builds.
            if not asset.is_file() and key != "onnx-file":
                raise FileNotFoundError(asset)
            manifest["assets"].append({"camera": camera, "key": key, "path": str(asset),
                                       "sha256": digest(asset) if asset.is_file() else None})
            properties[key] = str(asset)
        properties.pop("onnx-file", None)
        (output / f"source-{camera}-infer.ini").write_bytes(infer_path.read_bytes())
        snapshot = output / f"{camera}-infer.ini"
        with snapshot.open("w") as stream:
            infer.write(stream)
        config[camera].update(infer_config=str(snapshot), udp_host="127.0.0.1",
                              udp_port=str(port + index))
        manifest["cameras"].append(camera)
    effective = output / "system.ini"
    with effective.open("w") as stream:
        config.write(stream)
    manifest["effective_config_sha256"] = digest(effective)
    manifest["application_command"] = [str(binary), "--config", str(effective), "--record-only"]
    save_json(output / "manifest.json", manifest)
    return manifest


def receiver_command(output, camera, port):
    return ["gst-launch-1.0", "-e", "-v", "udpsrc", "address=127.0.0.1", f"port={port}",
            "caps=application/x-rtp,media=video,encoding-name=H264,clock-rate=90000", "!",
            "rtpjitterbuffer", "latency=100", "!", "rtph264depay", "!", "h264parse", "!",
            "splitmuxsink", f"location={output / (camera + '-%05d.mkv')}",
            "muxer-factory=matroskamux", "max-size-time=60000000000", "async-finalize=true"]


def stop_process(process, grace):
    """Only signal a child process group that this recorder created."""
    start = time.monotonic()
    signals = []
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.poll()
            break
        signals.append(sig.name)
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            pass
    return {"returncode": process.poll(), "signals": signals,
            "forced": any(s != "SIGINT" for s in signals),
            "shutdown_seconds": time.monotonic() - start}


def sample_resources(pids, output):
    result = {"monotonic_ns": time.monotonic_ns(), "wall_ns": time.time_ns(),
              "disk_free_bytes": shutil.disk_usage(output).free}
    paths = [Path("/proc/stat"), Path("/proc/meminfo")]
    paths.extend(Path(f"/proc/{pid}/{name}") for pid in pids for name in ("stat", "status", "io"))
    for pattern in ("/sys/class/thermal/thermal_zone*/temp", "/sys/class/thermal/thermal_zone*/type",
                    "/sys/class/devfreq/*/cur_freq",
                    "/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq"):
        paths.extend(Path(p) for p in glob.glob(pattern))
    for path in paths:
        try:
            result[str(path)] = path.read_text().strip()
        except (OSError, TypeError, UnicodeError) as error:
            # Some Thor sysfs files return no bytes rather than a normal read error.
            # Optional measurements must not stop recording; retain what was unavailable.
            result.setdefault("read_errors", {})[str(path)] = f"{type(error).__name__}: {error}"
    return result


def execute(output, manifest, seconds, grace, min_free_gb):
    for tool in ("gst-launch-1.0", "ffprobe"):
        if not shutil.which(tool):
            raise RuntimeError(f"Required tool missing: {tool}")
    config = read_config(output / "system.ini")
    if digest(output / "system.ini") != manifest["effective_config_sha256"]:
        raise RuntimeError("Prepared config was modified")
    bitrate = sum(config.getint(c, "encoder_bitrate", fallback=3000) for c in manifest["cameras"])
    required = min_free_gb * 1024**3 + seconds * bitrate * 1000 / 8 * 1.5
    if shutil.disk_usage(output).free < required:
        raise RuntimeError("Insufficient free disk for estimated recording plus reserve")
    manifest["tools"] = {t: command_output([t, "--version" if t.startswith("gst") else "-version"])
                         for t in ("gst-launch-1.0", "ffprobe")}
    manifest["clock_ticks_per_second"] = os.sysconf("SC_CLK_TCK")
    manifest["requested_seconds"] = seconds
    manifest["state"] = "starting"
    save_json(output / "manifest.json", manifest)
    children, handles = {}, []
    interrupted = []
    old_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, lambda number, _: interrupted.append(number))
    env = os.environ.copy()
    if manifest["trace"]:
        env["PATRONUS_TRACE_DIR"] = str(output)
    else:
        env.pop("PATRONUS_TRACE_DIR", None)
    reason = "startup_failure"
    try:
        tegrastats = shutil.which("tegrastats")
        manifest["tegrastats"] = "unavailable"
        if tegrastats:
            command = [tegrastats, "--interval", "1000"]
            log = (output / "tegrastats.log").open("w")
            handles.append(log)
            children["tegrastats"] = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            manifest["tegrastats"] = command
        for camera in manifest["cameras"]:
            command = receiver_command(output, camera, config.getint(camera, "udp_port"))
            manifest[f"receiver_{camera}"] = command
            log = (output / f"receiver-{camera}.log").open("w")
            handles.append(log)
            children[camera] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)
        time.sleep(1)
        if any(children[c].poll() is not None for c in manifest["cameras"]):
            raise RuntimeError("Receiver failed before application startup; inspect logs")
        if interrupted:
            reason = "user_signal"
            return
        log = (output / "application.log").open("w")
        handles.append(log)
        app = subprocess.Popen(manifest["application_command"], cwd=output, env=env,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children["application"] = app
        manifest.update(state="recording", application_pid=app.pid,
                        process_pids={name: child.pid for name, child in children.items()},
                        application_start_monotonic_ns=time.monotonic_ns())
        save_json(output / "manifest.json", manifest)
        deadline = time.monotonic() + seconds
        reason = "runtime_failure"
        with (output / "resources.jsonl").open("w") as stream:
            while True:
                sample = sample_resources([child.pid for child in children.values()], output)
                stream.write(json.dumps(sample) + "\n")
                stream.flush()
                if interrupted:
                    reason = "user_signal"
                elif app.poll() is not None:
                    reason = "application_exited"
                elif any(children[c].poll() is not None for c in manifest["cameras"]):
                    reason = "receiver_exited"
                elif sample["disk_free_bytes"] < min_free_gb * 1024**3:
                    reason = "disk_reserve"
                elif time.monotonic() >= deadline:
                    reason = "duration"
                else:
                    time.sleep(1)
                    continue
                break
    finally:
        # Stop producer first; give local RTP packets time to drain, then finalize segments.
        manifest["stop_reason"] = reason
        manifest["stop_requested_monotonic_ns"] = time.monotonic_ns()
        manifest["shutdown"] = {}
        if "application" in children:
            manifest["shutdown"]["application"] = stop_process(children.pop("application"), grace)
        time.sleep(0.3)
        for name, process in children.items():
            manifest["shutdown"][name] = stop_process(process, grace)
        for handle in handles:
            handle.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        manifest["state"] = "stopped"
        manifest["end_monotonic_ns"] = time.monotonic_ns()
        save_json(output / "manifest.json", manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/inference")
    parser.add_argument("--config", type=Path, default=ROOT / "config/system.ini")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mode", choices=("both", "rgb", "mono"), default="both")
    parser.add_argument("--seconds", type=float, default=300)
    parser.add_argument("--port", type=int, default=15000, help="First of two loopback RTP ports")
    parser.add_argument("--shutdown-grace", type=float, default=15)
    parser.add_argument("--min-free-gb", type=float, default=2)
    parser.add_argument("--no-trace", action="store_true", help="Instrumentation-overhead control")
    parser.add_argument("--execute", action="store_true", help="Open cameras and record; motors disabled")
    args = parser.parse_args()
    if (any(not math.isfinite(v) or v <= 0 for v in
            (args.seconds, args.shutdown_grace, args.min_free_gb)) or not 1024 <= args.port <= 65534):
        parser.error("Require positive durations/reserve and port 1024..65534")
    output = (args.output or ROOT / "live-runs" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")).resolve()
    try:
        manifest = prepare(args.config.resolve(strict=True), args.binary.resolve(strict=True),
                           output, args.mode, args.port, not args.no_trace)
        print(f"Run directory: {output}", flush=True)
        if not args.execute:
            print("Prepared only; no cameras or processes launched. Use --execute with a NEW run directory.")
            return
        execute(output, manifest, args.seconds, args.shutdown_grace, args.min_free_gb)
        subprocess.run([os.sys.executable, str(ROOT / "tools/live_run_analyse.py"), str(output)], check=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Recording failed: {error}\nEvidence retained in {output}\n")


if __name__ == "__main__":
    main()
