#include "patronus/tracing.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>

namespace {
std::string read(const std::string &path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int run(const std::string &directory) {
  const auto path = directory + "/events.csv";
  auto *pipeline = gst_parse_launch(
    "fakesrc name=producer num-buffers=17 sizetype=fixed sizemax=64 do-timestamp=true ! "
    "identity name=transform ! fakesink name=consumer",
    nullptr);
  if (!pipeline)
    return 1;
  bool installed = false;
  {
    // Neither the labels nor the caller's configuration survive until playback.
    std::vector<patronus::tracing::Location> locations{{"producer", "src", std::string(60, 'x')},
                                                       {"consumer", "sink", "delivered"}};
    installed = patronus::tracing::install(pipeline, path, locations);
    locations[0].stage = "changed by caller";
  }
  bool ok = false;
  if (installed) {
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    auto *bus = gst_element_get_bus(pipeline);
    auto *message = gst_bus_timed_pop_filtered(
      bus, 5 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    ok = message && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
    if (message)
      gst_message_unref(message);
    gst_object_unref(bus);
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline); // Drains the asynchronous writer before reading.
  if (!ok)
    return 2;
  const auto original = read(path);
  if (original.find("stage,monotonic_ns,pts_ns,offset,bytes\n") != 0 ||
      original.find("# dropped=0 complete=1\n") == std::string::npos)
    return 3;
  std::istringstream lines(original);
  std::string line;
  int source_count = 0, delivered_count = 0;
  while (std::getline(lines, line)) {
    if (line.rfind(std::string(60, 'x') + ",", 0) == 0)
      ++source_count;
    else if (line.rfind("delivered,", 0) == 0)
      ++delivered_count;
    else if (!line.empty() && line[0] != '#' && line.rfind("stage,", 0) != 0)
      return 4;
    if ((line.rfind(std::string(60, 'x') + ",", 0) == 0 || line.rfind("delivered,", 0) == 0) &&
        line.substr(line.rfind(',')) != ",64")
      return 5;
  }
  if (source_count != 17 || delivered_count != 17)
    return 6;

  auto *empty = gst_pipeline_new("empty");
  const bool overwritten = patronus::tracing::install(empty, path, {});
  const bool missing_pad =
    patronus::tracing::install(empty, directory + "/missing.csv", {{"absent", "src", "missing"}});
  const bool invalid_label =
    patronus::tracing::install(empty, directory + "/invalid.csv", {{"absent", "src", "bad,label"}});
  gst_object_unref(empty);
  return overwritten || missing_pad || invalid_label || read(path) != original ||
             std::filesystem::exists(directory + "/invalid.csv")
           ? 7
           : 0;
}
} // namespace

int main() {
  gst_init(nullptr, nullptr);
  auto *directory = g_dir_make_tmp("patronus-tracing-XXXXXX", nullptr);
  if (!directory)
    return 8;
  const int result = run(directory);
  std::filesystem::remove_all(directory);
  g_free(directory);
  return result;
}
