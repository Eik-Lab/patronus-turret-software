# Stdout profiling

The reusable Linux C++17 recorder is a self-contained CMake subproject in
[`profiling/`](../profiling/README.md). Copy that directory and link
`patronus::profiling` in another project. This application's GStreamer probes,
camera PTS joins, stage summary and motor/inference categories remain in
`src/pipeline/profiling*.cpp`, supplied through the recorder's report extensions.

Profiling is opt-in and defaults to OFF:

```sh
cmake -S . -B build-profile -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPATRONUS_PROFILING=ON
cmake --build build-profile -j
./build-profile/inference --record-only
```

Stop normally with Ctrl-C and wait for shutdown. A hotpath-style summary is
printed directly to **stdout**, after workers join. The profiler creates **no
files or directories** and ignores `PATRONUS_PROFILE_DIR` and `PATRONUS_TRACE_DIR`.
There is no CSV/JSON/timeline export or offline analysis step. Application logs,
explicitly configured tracking telemetry, and the separate live recording tool
are independent; this flag does not disable their output. Use `--no-trace` with
`tools/live_run.py`; that tool captures stdout in its application log.

## Report

The report leads with a compact, flat **stage summary**, followed by actual
**function trees**. Both retain Calls/Avg/P50/P95/P99/P999/Total/Calls per second.

* Stage summary: per-camera inference host latency (`nvinfer` input to OSD input
  after our callback), post-capture buffer preparation (source output to
  pre-converter output), delivery cadence, and recorded motor host operations.
  Camera totals require unique matching PTS, never sums of child averages or
  percentiles. Motor totals aggregate nonnested recorded operations, not whole
  control-loop periods or mechanical movement. Unknown values stay unavailable.
* Inference function tree: actual names such as `osd_sink_pad_buffer_probe`,
  `read_exposure_s`, `advance_capture_clock`, `captured_at`, `push`,
  `draw_predictions`, and `project_prediction`. Tracking's `step_filter` and
  `project_aim` remain separate roots when executed on another thread.
* Motor function tree: actual observed functions such as `set_velocity`,
  `lock_bus`, `set_velocity_locked`, `apply_limit`, `write_axis`, and SDK
  `setTargetVelocity`. With motors disabled, a plain not-exercised note replaces
  the tree; no static call paths are presented as recorded calls.

Tree roots identify the stream/gimbal and thread. Indentation uses recorded
same-thread interval nesting, including cross-stream children; ambiguous threads
and event loss suppress inferred trees. Uninstrumented intermediate calls are
not reconstructed. Child sample counts can differ; percentiles do not add.

Opaque internals are explained outside the trees: Pylon sensor acquisition and
buffer filling, NVIDIA preprocessing, TensorRT kernels and model-output parsing
are unavailable. There is no measured capture function tree. Buffer preparation
starts with an **already captured** frame: host pad residence is not GPU
completion or a separately measured CPU-to-GPU memcpy. Source/converter allocator
types and caps features are inspected on first buffers without mapping pixels;
these describe representation, not proof of copying or zero-copy.

Detailed wall/CPU hotspots and pad residence distributions remain available below
the trees. These reuse the same samples, not additional stage work. Synthetic
fixtures test motor nesting without exercising real CAN or motor hardware.

* Ranked inclusive and self wall/CPU hotspots, grouped by `stream/name@thread`:
  hotpath-style bordered, aligned columns for calls, **Avg, P50, P95, P99, P999**,
  total, share of table total, and calls/second. Durations carry ns/us/ms/s units;
  resource units remain in their metric names. P999 means the 99.9th percentile. Percentiles
  are exact nearest-rank statistics over retained samples, not interpolated.
  Fewer than 1000 samples cannot resolve that tail reliably.
* Instrumented thread totals use wall interval union and root-span CPU, avoiding
  nested double counting. Inclusive hotspot totals intentionally count children
  again; their percentages are not shares of total process execution time.
* Capture delivery cadence and pipeline residence for conversion, mux, inference,
  OSD, postconversion, encoding and payloading, plus source-to-encoder residence.
  Capture-to-observation age estimates are separate from execution hotspots.
* CPU, GPU and memory percentiles and first/last/delta/peak sampled values.
* UDP attempted bytes, push/list operations, failures and per-second rates.

Scopes include prediction, tracking ticks, observation waits/probes, pose reads,
filter updates, control and motor commands. Only exercised instrumentation
produces rows: absent work is unavailable, not a measured zero. `--record-only`
deliberately disables motors and control-related hardware.

Self time subtracts immediate recorded children on the same thread, even across
streams. Uninstrumented work, overhead, waiting and scheduling remain in wall
self time. CPU uses `CLOCK_THREAD_CPUTIME_ID`, not GPU timing. Missing CPU
measurements are omitted (the CPU table's count exposes partial coverage), never
zero-filled. CPU self can exceed wall self for tiny scopes because the clock
reads have different boundaries. Crossing or identical intervals make self time
and CPU root totals unavailable for that thread.

Residence pairs only unique valid same-camera PTS at both endpoints, with
nonnegative time differences. Duplicate, missing, rewritten or backwards PTS
are never guessed; missing residence rows are unavailable. DeepStream may
rewrite PTS at the mux. Marker counts expose coverage but are not frame-loss
proof. RTP duplicates are expected. Capture cadence is host delivery cadence,
not a direct sensor exposure measurement. Inference is host pad residence, not
GPU kernel duration. UDP spans measure synchronous downstream pushes into
`udpsink`, including buffer lists; a list operation is not a packet count.
There is **no client receipt, network delivery acknowledgement, source-to-UDP,
one-way or glass-to-glass latency measurement**.

Resources are sampled every 250ms (also at startup/shutdown). One busy process
core is 100%; multithreaded process CPU can exceed 100%. System CPU/memory
are system-wide. Memory metrics are bytes. NVML dynamically samples **device 0**
utilization and device memory, not per-function allocations or per-process GPU
utilization. Unsupported metrics explicitly say unavailable; Jetson GPU-memory
accounting may be unsupported. No allocation attribution, leak detection,
GPU kernel tracing, sampled call stacks or live TUI is provided.

## Memory and overhead

Events stay in memory, up to **1,000,000 events per session**. This can consume
hundreds of MB including strings, vectors, and report-time grouping. Keep runs
bounded. After capacity is reached, subsequent events are discarded and the
report explicitly shows the loss count, labels distributions partial, and omits
self-time estimates. Exact statistics cover retained events, not missing ones.
There is no disk spill. A crash, SIGKILL, or forced exit loses the summary.

Enabled profiling adds clock reads, locking, allocations, pad probes and resource
sampling. Its own memory/CPU is included in process samples. Compare against an
OFF build under the same conditions instead of subtracting a guessed overhead.

## Disabled means no-op

`-DPATRONUS_PROFILING=OFF` removes instrumentation at preprocessing time, even
at `-O0`. Scope/pipeline arguments are not evaluated; wrapped calls execute
normally. No profiler code, CPU-clock instrumentation, resource sampler, NVML
loading, profiling probes, legacy pad writer or profiler thread is linked into
the application. Neither old environment variable enables an OFF build.

Replay-only CTest builds check disabled argument elimination and, when profiling
is ON, verify stdout percentiles/nesting and real loopback UDP buffers/lists:

```sh
ctest --test-dir <replay-build> --output-on-failure
```
