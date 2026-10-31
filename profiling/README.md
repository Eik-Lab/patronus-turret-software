# Copyable stdout profiler

Copy this entire directory into another repository. It builds independently or
via `add_subdirectory`; it does not read anything from its parent project and
does not fetch dependencies. Requirements: CMake 3.21+, C++17 and, when enabled,
Linux. CPU/memory sampling uses `/proc`, POSIX clocks and threads. NVIDIA NVML
headers are optional; the driver library is loaded dynamically when present.
There are no CUDA, DeepStream, Pylon, GStreamer, CAN or Python dependencies.

## Embed in a project

```cmake
add_subdirectory(profiling)
target_link_libraries(my_application PRIVATE patronus::profiling)
```

Configure with `-DPATRONUS_PROFILING=ON` to enable. The default is OFF: the target
is header-only, no recorder/sampler/NVML loader is built, scope arguments are
discarded without evaluation, and call wrappers execute only the original call.
Use the target rather than defining the feature macro manually.

```cpp
#include <patronus/profiling.h>

void decode() {
  PATRONUS_PROFILE(decode, "worker", UINT64_MAX);
  // Work to measure.
}

int main() {
  patronus::profiling::Session profiling;
  decode();
} // Join all producers before this Session is destroyed.
```

The generic report works for arbitrary names without callbacks: nested function
scopes, inclusive/self wall and thread CPU hotspots, resource percentiles and
trends. Tables include Avg/P50/P95/P99/P999. Scopes measure host time, not GPU
kernel completion. Only recorded same-thread interval nesting is shown; unknown
CPU values remain unavailable and ambiguous intervals suppress inferred trees.
Trees use UTF-8 `├─`, `└─` and `│` connectors, supported by Nerd Fonts and most
monospace terminal fonts; no font-specific private-use icons are required.

Use one Session per process, constructed before producers start and destroyed
after they stop/join. The report prints to stdout on normal destruction. A
signal handler should request orderly application shutdown, not print a report
or destroy the Session directly. SIGKILL/crashes cannot preserve the summary.
No profiler files are created. At most 1,000,000 events are retained in RAM;
overflow is reported explicitly and self times/trees are suppressed on loss.

## Standalone build and test

```sh
cmake -S profiling -B build-profiler -DPATRONUS_PROFILING=ON
cmake --build build-profiler
ctest --test-dir build-profiler --output-on-failure
```

Repeat with `-DPATRONUS_PROFILING=OFF` to check disabled behavior. Standalone
tests default to ON only when this directory is the top-level CMake project;
embedding projects can opt in with `PATRONUS_PROFILING_BUILD_TESTS`.

## Application-specific reports

`Session` optionally accepts `ReportOptions`. `tree_root` returns a tree title
and display context for a selected root, or an empty title to skip it. A null
selector includes every recorded span in the generic function tree.
`summary` and `details` append application-owned sections before the function
trees and after the hotspots, respectively. They receive the output stream,
retained events, valid selected root events, and elapsed seconds. References
must not escape the callback; reporting runs after recording and sampling stop.
`print_table` provides the same formatting/percentiles to extensions (ranked
samples are milliseconds, unranked samples retain their original units).

`record` accepts `span` (execution), `estimate` (not execution), and `gauge`
(resource distribution) events. Other kinds can carry application-owned data to
report extensions without polluting execution or resource statistics. This
repository's camera joins, stage summary, UDP interpretation and function
categories are implemented outside this directory using that extension API.
