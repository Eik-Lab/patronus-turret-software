// This must compile and run at -O0 without GStreamer or the profiling library.
#include "pipeline/profiling.h"

#include "patronus/profiling.h"

#ifdef PATRONUS_PROFILING
  #error "The disabled fixture must not inherit the profiling feature"
#endif

int main() {
  int evaluated = 0;
  // Undeclared identifiers deliberately prove argument tokens are discarded.
  PATRONUS_PROFILE(ignored, ++evaluated, this_function_must_not_be_resolved());
  if (!PATRONUS_PROFILE_PIPELINE(++evaluated, this_camera_must_not_be_resolved()))
    return 1;
  if (evaluated != 0)
    return 2;
  const auto value = PATRONUS_PROFILE_CALL(ignored_call, this_stream_is_not_resolved(),
                                           this_id_is_not_resolved(), ++evaluated);
  return value == 1 && evaluated == 1 ? 0 : 3;
}
