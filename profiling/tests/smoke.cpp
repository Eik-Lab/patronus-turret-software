#include "patronus/profiling.h"

#include <iostream>
#include <sstream>

int main() {
  std::ostringstream captured;
  auto *previous = std::cout.rdbuf(captured.rdbuf());
  bool calls_ok = true;
  {
    patronus::profiling::Session session;
#ifdef PATRONUS_PROFILING
    using patronus::profiling::record;
    // Unknown-to-the-library function names must work without any application
    // callback, including nesting across stream labels and separate estimates.
    record("span", "parse_document", "example", 1000000, 4000000, 0, UINT64_MAX, 3000000);
    record("span", "decode_field", "other", 2000000, 1000000, 0, UINT64_MAX, 500000);
    record("estimate", "remote_age", "example", 1000000, 9000000);
#else
    int evaluated = 0;
    PATRONUS_PROFILE(ignored, ++evaluated, undefined_id());
    const auto result =
      PATRONUS_PROFILE_CALL(ignored_call, undefined_stream(), undefined_id(), ++evaluated);
    calls_ok = result == 1 && evaluated == 1;
#endif
  }
  std::cout.rdbuf(previous);
  const auto output = captured.str();
#ifdef PATRONUS_PROFILING
  if (output.find("Function tree\n") == std::string::npos ||
      output.find("parse_document [example@") == std::string::npos ||
      output.find("└─ decode_field") == std::string::npos ||
      output.find("P999") == std::string::npos ||
      output.find("dropped_events=0") == std::string::npos ||
      output.find("example/remote_age") == std::string::npos ||
      output.find("End Patronus profiling") == std::string::npos)
    return 1;
#else
  if (!output.empty())
    return 1;
#endif
  return calls_ok ? 0 : 1;
}
