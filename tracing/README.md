# Copyable GStreamer pad tracer

Copy this entire directory into another repository. It is an independent
CMake 3.21+/C++17 project requiring GStreamer 1.0 development files, pkg-config
and threads. It does not fetch dependencies or read parent-project files. There
are no camera, CUDA, DeepStream, CAN, application logger or profiling-module
dependencies. Unlike the stdout profiler, this library explicitly records CSV
files when its installation API is called.

## Embed

```cmake
add_subdirectory(tracing)
target_link_libraries(my_application PRIVATE patronus::tracing)
```

```cpp
#include <patronus/tracing.h>

// After constructing a GStreamer pipeline, before setting it to PLAYING:
bool ok = patronus::tracing::install(pipeline, "/existing-directory/events.csv",
  {{"my_source", "src", "source"}, {"my_transform", "sink", "transform_in"},
   {"my_transform", "src", "transform_out"}});
```

The output file must not exist; existing files are never overwritten. The
directory must already exist. Supply element names, **static** pad names and
stage labels; labels cannot contain commas, quotes, CR or LF. Configuration and
labels are copied, so temporary strings are safe. Failure to create the file or find a
pad returns false, with a diagnostic on stderr. Destroy the pipeline on failure:
earlier probes may already be installed. Allocation/thread failures can throw.

Probes share ownership of the asynchronous writer. To drain it and finalize the
file, stop the pipeline and release all references retaining its traced pads.
No global shutdown hook or application signal handler is installed. Orderly
teardown writes `complete=1`; crashes/SIGKILL may leave incomplete data.

Each buffer event records:

```text
stage,monotonic_ns,pts_ns,offset,bytes
```

Events are stamped using the host steady clock. PTS and offset are copied
unchanged, including GStreamer's unavailable sentinels; bytes is buffer size.
Only buffer probes are installed, not buffer-list/UDP-send tracing. No pixels are
mapped and no GPU synchronization is added. These are host pad observations,
not camera acquisition, isolated GPU kernels or end-to-end delivery latency.

Pending events are bounded to 8192 plus one writer batch. Overflow increments a
dropped counter. The writer flushes every 250 ms and on teardown, with periodic
`# dropped=N complete=0/1` lines. `complete=1` means the writer drained during
teardown, not that no records were dropped or that the pipeline succeeded.
Write errors are reported to stderr; the trace must not be treated as complete.

The library reads no environment variables. To disable tracing, do not call
`install`; no probes, threads or files are then created. The original
application's `PATRONUS_TRACE_DIR`, camera-based filenames and pad mapping live
in its own adapter, not in this directory. Its inference executable remains
stdout-profile-only and does not link this legacy CSV tracer.

## Standalone build and test

```sh
cmake -S tracing -B build-tracing
cmake --build build-tracing
ctest --test-dir build-tracing --output-on-failure
```

The test uses only `fakesrc`, `identity` and `fakesink`; no camera, GPU, motor or
network access. Tests default to ON for standalone builds, OFF when embedded;
set `PATRONUS_TRACING_BUILD_TESTS` to override. Test CSVs are temporary and removed.
