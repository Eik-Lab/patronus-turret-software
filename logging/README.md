# Copyable logger

Copy this entire directory into another repository. It builds independently or
via `add_subdirectory`, requires CMake 3.21+, C++17 and threads, and does not fetch
dependencies or read parent-project files. There are no camera, GPU, GStreamer,
profiling or tracing dependencies. The library is header-only.

## Embed

```cmake
add_subdirectory(logging)
target_link_libraries(my_application PRIVATE patronus::logging)
```

```cpp
#include <patronus/logging.h>

int main() {
  using namespace patronus::logging;
  set_log_level(Level::Info);
  if (!set_log_file("my-application.log"))
    return 1; // Or continue with console-only output.
  LOG(Info, "Started worker " << 3);
  set_log_file(std::nullopt); // Disable file output; console output continues.
}
```

The default threshold is Warning and the default file is `patronus.log` in the
working directory, preserving the original application's behavior. Set the file
before the first log message to select another path or disable file output
without creating the default file. Files open in append mode; parent directories
must already exist. `set_log_file` returns false if opening fails; the console
continues working and the next unfiltered message reports the failure once on
stderr. Another call can select a different file or retry after an open failure.

Messages use local timestamps, colored severity labels on stdout and plain text
in the file. Each line is flushed. Levels are Debug, Info, Warning, Error and
Critical. `parse_level` accepts their lowercase names plus `warn`, returning an
empty optional for unknown names. `LOG(level, stream_expression)` retains the
existing macro spelling; filtered expressions are not evaluated.

Configuration and output are shared across translation units in a process.
Writes and file changes are serialized with a mutex; the threshold is atomic.
Configure at startup for a deterministic destination for early messages, and
join logging threads before process teardown. Do not log from signal handlers.

## Standalone build and test

```sh
cmake -S logging -B build-logging
cmake --build build-logging
ctest --test-dir build-logging --output-on-failure
```

Tests default to ON only when this directory is the top-level project. Embedding
projects can opt in with `PATRONUS_LOGGING_BUILD_TESTS`. Tests use temporary files
and do not access hardware or the network.
