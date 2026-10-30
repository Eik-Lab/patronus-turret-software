# Tracking replay and timing baseline

This is a baseline for changing the tracking implementation, not a simulated
camera or motor. It calls the production `step_filter`, `aim_at`, and sensor control law
with explicit timestamps. It never opens a camera or sends motor commands.
Tracking algorithms and scheduling are unchanged by this instrumentation.

The filter owns no heap allocation: `FilterState` contains its tracker and fixed
pose history inline. Resolve `FilterParameters` once, hold them constant, and use
the same parameters for that state's frame and tick calls. Pose interpolation
consumes whole samples, so the ring remains an array of structs rather than
separate coordinate arrays. Exposing inline tracker storage makes the filter
header depend on kalman-cpp/Eigen; consumers must link `kalman::kalman`.

## Build without hardware SDKs

Requires a C++17 compiler, CMake >= 3.28, pkg-config, GLib development headers,
Python 3, and network access for the pinned Eigen and kalman-cpp dependencies.
Neither CUDA, DeepStream, Pylon nor CANdle is needed or fetched in this mode.

```sh
cmake -S . -B build/replay -DPATRONUS_REPLAY_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/replay -j
ctest --test-dir build/replay --output-on-failure
```

## Replay a recorded run

```sh
python3 tools/telemetry_replay.py telemetry/RUN build/replay-input --gimbal 0
```

The converter creates a **new** directory containing the recorded configuration
and a sorted event stream. It prints a shell command using the recorded camera
and level tilt. Run that command, redirecting stdout to `baseline.csv` and stderr
to `baseline-timing.txt`. Its final argument is the number of measured replays.
Run the same input after a change and compare the output CSVs. Compare numbers
with explicit tolerances across compilers/architectures; byte equality is useful
for repeated runs of the same executable, not a portability guarantee.

Recorded CSV values and timestamps are rounded. Equal event times are ordered
pose, frame, aim. Old logs lack aim-call times: the converter warns and uses
the pose timestamp, which can precede that tick's filter update. Dropped records
and incomplete runs also produce warnings. **Do not treat reconstructed output
as a bit-exact reproduction of live commands.** Once converted, the event stream
is a fixed input for comparing implementations.

Output commands are normalized [-1, 1], before motor scaling, clamps, soft limits,
and bus I/O. Filter replay requires `filter_enabled=true`. Control-state replay
below also supports unfiltered mode. Neither mode simulates physical movement,
motor limits, follower bearing calculations, mailbox overwrites, inference, or
scheduling.

## Replay control-loop decisions

```sh
python3 tools/telemetry_replay.py telemetry/RUN build/control-input --gimbal 0 --control
```

This mode uses the new tick columns `control_elapsed_ns`, `raw_detection`,
`estimate_available`, `driving`, and `reached_home`. It requires a completed run
with no dropped ticks and refuses older logs lacking those inputs. It does not
guess state-machine inputs from rounded filter output. Input time is measured
from loop startup, so the initial detection timestamp is exactly zero.

Synthetic events use integer nanoseconds to preserve deadline boundaries:

```text
control ELAPSED_NS RAW_DETECTION ESTIMATE_AVAILABLE DRIVING HOME_REACHED [POSE_USABLE]
```

All flags must be 0 or 1. `POSE_USABLE` defaults to 1 for older control streams;
new telemetry records it explicitly. A false value requests a hold for the leader
and followers, suppresses home completion, and suspends timeout-driven home transitions.
Each event calls the same `step_control` used by
the live loop, then applies supplied home feedback only if it requested a return.
Control events supply their own detection/estimate flags; they do not implicitly
consume earlier `frame` events. Use separate streams for control and filter
replay unless their time origins have explicitly been aligned.

When control events are present, CSV output appends `phase`, `next_phase`,
`leader_action`, `follower_action`, `aiming`, and `stop_after_home`:

- Phases: 0 tracking, 1 returning home, 2 at home. `phase` matches live telemetry
  before end-of-tick transitions; `next_phase` includes those transitions and feedback.
- Live telemetry also records phase 3 (manual). The operator's mode is not a replay
  input, so `telemetry_replay.py --control` refuses runs that contain it.
- Actions: 0 no command, 1 aim, 2 hold, 3 return home. These are requests, not
  proof a motor moved. `stop_after_home=1` requests the extra leader stop on arrival.
- Raw detections refresh freshness even when the filter rejects them. Filtered
  aiming requires an estimate with age strictly less than 45 configured ticks.
- At the home-delay boundary, the loop stops first and changes phase; return
  motion starts on the next tick. Followers continue parking when the leader is home.
- Overlay-only mode emits no motor actions. In unfiltered mode only this tick's
  raw detection can request aiming; retained estimates are ignored.

Timing output adds `control_state` p50/p99/max, including decision and feedback
processing but no CAN calls. The live loop still owns I/O, applies motor limits,
and keeps its existing scheduler.

## Hardware outcomes and fault policy

Motor reads preserve each axis's SDK error, validity and host exchange midpoint.
A failed read has a NaN value, never a valid zero angle. Both axes must be valid
and younger than 45 configured control ticks at the freshness check (450 ms for
a 10 ms tick). This reuses the existing freshness window; it is not a validated
physical safety bound. It does not bound the subsequent bus call or certify the
age of the encoder's internal sample.

An unusable leader pose clears the filter/history and requests holds; recovery
requires valid feedback and a new filter estimate. Home/follower moves independently
check their own reads. Failed soft-limit feedback requests zero on both axes.
Partial/failed nonzero writes trigger one best-effort zero write per axis. Results
of the original writes and the stop attempts stay separate. No SDK result proves
physical motion or a successful physical stop. Failed writes are retried through
normal subsequent ticks, not an unbounded retry loop.

Leader tick telemetry includes `pose_usable`, per-axis `*_read_s/status/error`,
`*_requested`, `*_attempted`, `*_write_status/error`, `*_limit_value/s/status/error`,
`*_stop_status/error`, and the additional `return_*` position reads used by home
movement. Stop fields also record the extra zero writes on home arrival. Status
values are 0 not attempted, 1 OK, 2 error, 3 unavailable. SDK errors are separate
integers (-1 means no SDK call); SDK OK is 1 in the pinned CANdle version.
The legacy `cmd_pan/cmd_tilt` fields describe original SDK-successful writes, not
the later fallback stop or measured motion; use the outcome columns for faults.
Detailed follower outcomes are returned by the motor API but are not recorded
as independent control streams.

`drive_to` (including return-home and follower moves) holds one bus lock across
its position reads, calculation, separate soft-limit reads, writes and fallback
stops. Other bus callers cannot interleave within that operation. They may wait
longer; this does not bound SDK blocking, guarantee fairness or stop physical motion.
Feedback reuse remains disabled: a normal calibrated two-axis move still performs
four reads and two writes. No command-age/movement allowance or reuse switch has
been added. Those require validated bounds before changing soft-limit feedback.

The hardware-free production-wrapper regression uses the pinned CANdle SDK
headers and logger source, but replaces SDK I/O and never links transport code:

```sh
bash tools/test_motor_transaction.sh /path/to/CANdle-SDK
```

Use SDK revision `60c4e2c973de5632d81783271d54941a0202b91b` and a Linux C++17
toolchain with libusb headers. No hardware or SDK installation is needed beyond
these build inputs. This test is separate from replay-only CTest, which deliberately
does not fetch CANdle. It checks fresh limit readings, boundaries, inward recovery,
fault/stale holds, fallback stops, arrival and concurrent transaction isolation.
The process-lifetime fixture deliberately never runs motor initialization/shutdown.

Control replay reproduces the recorded hold/recovery decisions using `pose_usable`
and the validated `reached_home` result; it does not replay CAN transactions.
Filter conversion refuses runs with pose faults because it cannot reconstruct
filter resets from the existing filter event format. Use `--control` for those runs.

## Synthetic filter inputs

One whitespace-delimited event per line, in nondecreasing event-time order;
comments start with `#`. Filter event times are seconds on the same monotonic epoch.
Equal timestamps retain file order.

```text
pose TIME PAN_RADIANS TILT_RADIANS
frame CALL_TIME ARRIVAL_TIME CAPTURE_CADENCE MID_EXPOSURE FRAME_ID DETECTED LEFT TOP WIDTH HEIGHT CONFIDENCE
aim CALL_TIME
```

`DETECTED` is 0 or 1; use zero box fields for 0. Capture cadence and mid-exposure
may be 0 to exercise the production fallback. Frame arrival/capture timestamps
may repeat or regress even though **event** times must be ordered. Coordinates
are detection-image pixels, not muxed display pixels. CLI:

```text
tracking_replay CONFIG CAMERA GIMBAL LEVEL_TILT EVENTS REPEATS
```

Gimbal indices are zero-based. Level tilt is the calibrated home tilt in radians.
No wall-clock sleeps occur. The filter is reconstructed for each pass; a complete
warm-up pass is excluded. Input parsing, initialization, output formatting, and
timing-sample storage are outside the timed regions. Arrays are allocated before
measurement. Reported p50/p99 (nearest rank) and maximum include dispatch, output
assignment, clock overhead and any OS preemption. Very short operations approach
timer overhead. These are CPU-side wall timings, **not end-to-end latency**.
Record compiler/build flags, CPU, power mode and workload alongside each result.
Run on the Jetson for deployment-relevant numbers; do not benchmark sanitizers.

`filter_step` includes every frame. `filter_successful_update` reports only frames
that entered with an initialized track and returned an accepted measurement;
`filter_seed` separately reports initial acquisition and reacquisition. Duplicate
timestamps, gated-out detections, coasting and missing estimates do not enter
either subset. Empty subsets emit no timing line. Classification and preallocated
sample storage occur outside timing; each subset reuses the original full-step
duration, not an isolated Kalman kernel duration. Warm-up is excluded throughout.
CSV outputs and CLI arguments are unchanged. Extra storage/classification/sorting
changes benchmark overhead, so do not compare whole-process time against the old
harness as evidence of a filter speedup. Use this harness for both sides of future
refactor comparisons, with identical inputs and matching subset counts.

## Live timing

With telemetry enabled, `ticks.csv` now appends:

- `loop_start_s`, `loop_us`: loop body including I/O, waits, frame telemetry,
  overlay publication, followers and state transitions, but excluding submission
  of the tick record itself. Start-to-start spacing also includes that submission.
- `pose_read_us`: leader pose acquisition including bus-lock wait.
- `observation_wait_us`: detection mailbox wait, including timeout.
- `filter_us`: `step_filter`, zero when no step ran.
- `aim_s`: the timestamp passed to `aim_at`, zero when not called.
- `command_start_s`, `command_done_s`: leader command sequence start/return for
  filtered/raw aim, hold and return-home, including fallback/arrival stops but
  excluding arrival logging and followers. Zero when no command ran. These are
  **not** physical actuation times or confirmation of successful motion.
- `control_us`, `aim_compute_us`, `follower_us`: state decision, aim/control
  calculation (including empty aim results), and all follower work for this tick.
- `previous_submit_us`: preceding tick's telemetry queue submission cost; first
  tick is zero. `iteration_us`: start-to-start spacing, including submission and
  reset overhead. Final shutdown submission has no subsequent tick to report it.
- `bus_lock_wait_us`, `sdk_read_us`, `sdk_write_us`, `sdk_read_calls`,
  `sdk_write_calls`: per-leader tick totals including its followers, soft-limit
  reads and fallback stops. Reads/writes count logical SDK calls, including failed
  calls, not physical CAN frames or internal retries. Missing devices generate no
  SDK calls. Lock timing includes uncontended acquisition overhead.
- `source_frame_id`, `source_probe_s`, `source_publish_s`, `source_capture_s`:
  latest consumed observation, retained across no-frame ticks, not necessarily
  the detection accepted by the filter. Capture time is estimated mid-exposure;
  zero means unavailable. `frames.csv` also appends `published_s`, stamped just
  before mailbox publication. `observation_received_s` is zero on no-frame ticks;
  `skipped_observations` counts publications overwritten since this reader's last
  consumption, not camera or GPU frame drops.
- `control_elapsed_ns`, `raw_detection`, `estimate_available`, `reached_home`:
  state-machine inputs and return-home result for control replay. Elapsed time
  uses integer nanoseconds from loop startup, not the `steady_now_s()` epoch.

`python3 tools/telemetry_analyse.py telemetry/RUN` (requires numpy) reports p50,
p99 and maxima for durations and source-frame age through leader command return,
including coasting ticks. It separates SDK-successful aim writes from all attempts.
Publication-to-consumption includes mailbox lock/scheduling delay. Capture-to-probe
is **aggregate pipeline latency**, not isolated GPU inference time; there are no
CUDA synchronization points. Nested timings overlap and must not be added.
The steady-state path is covered; boot/calibration and physical actuation are not.
Existing logs still work. Zero/inactive durations are excluded from percentiles;
per-tick counts include zero. Missing/nonfinite/backwards timestamp intervals are
excluded. No deadline-miss claim is made: this is not a fixed-period scheduler.
Most new timing clocks are gated by telemetry; publication adds one clock read
even when telemetry is disabled. Larger records, clocks and writer load perturb
the workload. Compare enabled/disabled runs on hardware to quantify overhead;
replay alone cannot establish live latency. Run the regression suite with a
NumPy-enabled Python to also exercise the analyzer's timing report.

## Offline encoder validation

The checked-in `config/system.ini` selects `encoder_backend=thor` for both cameras
based on measured dual-pipeline capacity and CPU use. Omitted keys still default
to `x264` for compatibility; select it explicitly for software encoding. The
`thor` backend keeps post-OSD conversion in NVMM and uses Thor H.264 hardware,
followed by parsing before the existing RTP payloader. Both accept kbps. Thor
uses High profile, GOP/IDR 60, no B frames, CBR, P4/UltraLowLatency and 600 ms VBV;
the x264 preset, tuning and default GOP/rate control remain unchanged. Unknown
backends, invalid bitrate or missing hardware plugins/properties fail startup;
there is no silent fallback. This option does not change control feedback reuse.

Build and run the actual encoder-bin regression independently of camera SDKs:

```sh
c++ -std=c++17 -O3 -DNDEBUG -Isrc tools/test_video_encoder.cpp \
  src/pipeline/video_encoder.cpp src/core/config.cpp \
  $(pkg-config --cflags --libs gstreamer-1.0) -pthread -o /tmp/test-video-encoder
python3 tools/test_video_encoder.py /tmp/test-video-encoder --thor -v
```

Requires GStreamer video/test/RTP/parser/x264 plugins and ffprobe; `--thor` also
requires Thor's NVIDIA encoder and converter. Omit it to test software only.
The test generates synthetic frames, uploads to NVMM for hardware, passes through
the production encoder bin and an in-process RTP pay/depay roundtrip, then decodes
with ffprobe. It never opens a camera, motor, UDP socket or production service.
It checks both camera selections, omitted/explicit software defaults, rejection
without fallback, frame count/dimensions, no B frames, and Thor High/GOP 60.

Earlier synthetic benchmarks showed lower CPU use and higher saturated capacity,
but slightly worse paced latency and a mixed RGB PSNR/SSIM tradeoff at matched
achieved bitrate. Actual-production-bin dual tests subsequently showed higher
capacity and lower CPU with Thor/Thor, but do not establish production quality
equivalence. The repository selection is not deployment or acceptance: representative
footage, achieved bitrate, receiver compatibility and end-to-end latency still need
validation. An in-process RTP roundtrip is not a network/display acceptance test.
