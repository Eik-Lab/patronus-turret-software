# Recording live pipeline evidence

`live_run.py` prepares an isolated run by default. Only `--execute` opens
cameras. It never enables motors, boot calibration, the sensor serial port or
the UDP command server: the generated configuration disables them and the new
application `--record-only` switch enforces those overrides again. Do not use
an old binary. This is a camera/inference/streaming baseline, **not** an actuated
control-loop or motor-bus benchmark.

## Before a run on Thor

Build the current source with the normal production build procedure. Have
`gst-launch-1.0`, RTP/H264 parser and Matroska plugins, and `ffprobe` installed.
Both selected inference configurations must reference existing working engines
and parser libraries. The recorder copies INI configs, resolves model paths and
removes the ONNX fallback so a recording cannot rebuild the engine. It does not
change the production configuration, models, power mode, clocks or services.

Check that no other application owns the cameras, and that loopback ports
15000/15001 are free. Do not stop unrelated services automatically. Use a local
disk with at least 2 GiB reserve plus the recording estimate. At 3+5 Mbps,
video alone is approximately 300 MB per five minutes; trace/log overhead is
additional. Runs are private data and ignored under `live-runs/`.

Record scene, lighting, camera placement, exposure/gain setup, target distance
and motion, ambient temperature, power mode, other workloads and any visible
failure in a `notes.txt` inside each run. The command server is disabled, so
configure camera exposure beforehand. Settings in a source config are not proof
of the camera's actual exposure. Keep a representative static scene and motion
scene; encoded footage includes overlays and is not raw sensor ground truth.

## Prepare, then record

From the checkout, using the freshly built executable. Omitting `--mode` uses
`[general] mode` from the supplied configuration (currently mono-only); an
explicit `--mode both` deliberately re-enables both cameras in the isolated run.

```sh
# Preparation only: no application, receiver or camera starts.
python3 tools/live_run.py --binary build/inference

# User-initiated live recording, five minutes, a fresh directory each time.
python3 tools/live_run.py --binary build/inference --seconds 300 --execute
```

The tool prints the run directory. Preparation is a dry run; `--execute` creates
a new directory rather than reusing the prepared one. `--output PATH` must not
already exist. Ctrl+C requests shutdown; allow up to the configured 15-second
grace plus escalation for each child. Forced shutdown is recorded as a failure,
not hidden. A pipeline can fail while the process remains alive: inspect both
camera summaries and application logs, not just its exit code.

Run the same scene with `--mode rgb`, `--mode mono`, then `--mode both`; repeat
the sequence in reversed order to expose thermal/order effects. Use identical
engines, resolutions, frame-rate settings, exposure and encoder configuration.
Collect at least three repetitions per condition. To measure **additional pad
tracing overhead**, repeat matched runs with `--no-trace`; this keeps telemetry,
receivers, resource sampling and logging enabled. It is not a measurement of
total recording overhead versus ordinary production. Recording and local
receivers themselves consume CPU, memory bandwidth and disk bandwidth.

## Evidence and interpretation

Each run includes:

- `manifest.json`: binary/asset/source hashes, Git state, requested duration,
  commands, process IDs and exact shutdown signals/results; `source.patch` and
  effective/source config copies. Source hashes include new recorder files;
  they do not prove which source built an arbitrary supplied binary.
- `pipeline-{rgb,mono}.csv`: source, preconversion, mux output, inference, OSD,
  postconversion and encoder boundary events, using host monotonic time and
  original buffer PTS/offset/size. No pixel maps or GPU synchronization are added.
- `{rgb,mono}-NNNNN.mkv`: existing encoded RTP received on loopback, without
  re-encoding, segmented at keyframes near 60 seconds. The receiver uses a
  100 ms jitter buffer. This replaces delivery to the configured remote viewer;
  it does not measure the real network/client or display latency.
- `application.log`, `patronus.log`, receiver logs and existing `telemetry/`
  tracking data. Tracking telemetry follows configured consumers and may cover
  only one camera; independent pad traces cover both active pipelines.
- `resources.jsonl`: once-per-second system/process CPU, memory, I/O, thermal
  and available frequency readings. Receiver process data is preserved too;
  summary CPU currently covers the application only. Missing sysfs readings
  remain unavailable, not zero. If installed, `tegrastats` also records raw
  GPU/CPU/EMC/thermal/power samples to `tegrastats.log` at one-second intervals;
  unavailable fields are not estimated. Its output is retained for later
  analysis, not parsed into `summary.json`.
- `summary.json`: per-stage throughput/gaps/tail-to-stop, p50/p95/p99/max of
  matched durations, unmatched counts, trace completeness/loss and decoded
  video diagnostics. Offline decoding writes per-segment `.frames.csv` files.

Trace buffers are bounded to 8192 pending events per camera plus one writer
batch. The writer flushes every 250 ms; overflow increments a dropped counter.
An unset `PATRONUS_TRACE_DIR` installs no probes or trace writer. A killed process
may lose buffered events: a final `complete=1` requires trace teardown, but is
not evidence of successful video delivery. Missing probes, dropped records,
decoder errors, empty recordings, forced exits or long pre-stop stalls invalidate
a clean-run claim. A long gap can also reflect a slow configured frame rate;
judge it against the actual workload rather than the warning alone.

Timing joins require an identical, unique, valid PTS on both boundaries. Rewritten,
missing or duplicate PTS remain explicitly unmatched; no ordinal/offset guessing.
Pad traversal measures residence/waiting as well as work, **not** isolated CUDA
kernel cost, acquisition latency or glass-to-glass latency. Receiver timestamps
have a separate origin. Offsets are not proven camera block IDs. Stage counts
need not equal frame counts at every encoder boundary. No zero-camera-loss or
detection-accuracy claim follows from these traces. Review footage separately.

Re-run analysis without cameras:

```sh
python3 tools/live_run_analyse.py live-runs/RUN_DIRECTORY
# Quick trace/resource analysis without decoding video:
python3 tools/live_run_analyse.py live-runs/RUN_DIRECTORY --skip-video
```

Share the complete run directory (or retain videos locally and first share
manifest, summary, notes, traces, resources and logs). Preserve failed runs too;
do not compare only the successful samples. Analysis runs after recording stops
so decoding does not contaminate the live measurement interval.

## Camera-free verification

The replay CMake configuration registers `live_recording`; when GStreamer
development libraries are available it also builds a fakesrc/identity trace
fixture. Python uses only the standard library:

```sh
cmake --build build/replay -j2
ctest --test-dir build/replay --output-on-failure
python3 tools/test_live_run.py
```

The standalone Python command skips the C++ integration case unless passed
`--trace-driver build/replay/run_trace_fixture`. No test opens a camera or motor.
