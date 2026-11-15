// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_converter.hpp"
#include <yaml-cpp/yaml.h>

#include <charconv>
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/resource.h>

namespace {

template <typename T> T positiveNumber(const std::string &text) {
  T result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      result == 0)
    throw std::invalid_argument("invalid positive integer: " + text);
  return result;
}

void apply(vq_map_server::map2d::ConversionOptions &options, const std::string &name,
           const std::string &value) {
  using namespace vq_map_server::map2d;
  if (name == "block_size") {
    options.block_size = positiveNumber<std::uint32_t>(value);
    if (options.block_size > 64)
      throw std::invalid_argument("block_size must be in [1, 64]");
  } else if (name == "frame_id") {
    options.frame_id = value;
  } else if (name == "cell_encoding") {
    if (value == "ternary") options.encoding = CellEncoding::Ternary;
    else if (value == "binary") options.encoding = CellEncoding::Binary;
    else throw std::invalid_argument("cell_encoding must be ternary or binary");
  } else if (name == "unknown_policy") {
    if (value == "reject") options.unknown_policy = UnknownPolicy::Reject;
    else if (value == "free") options.unknown_policy = UnknownPolicy::Free;
    else if (value == "occupied") options.unknown_policy = UnknownPolicy::Occupied;
    else throw std::invalid_argument("unknown_policy must be reject, free, or occupied");
  } else if (name == "max_map_bytes") {
    options.max_map_bytes = positiveNumber<std::size_t>(value);
  } else {
    throw std::invalid_argument("unknown configuration parameter: " + name);
  }
}

void usage() {
  std::cout << "Usage: vq_map_convert_2d --input YAML --output H5 [--config FILE]\n"
               "  [--block-size N] [--frame-id FRAME] [--cell-encoding ternary|binary]\n"
               "  [--unknown-policy reject|free|occupied] [--max-map-bytes N]\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const auto start = std::chrono::steady_clock::now();
    std::map<std::string, std::string> flags;
    for (int i = 1; i < argc; ++i) {
      const std::string name = argv[i];
      if (name == "--help") { usage(); return 0; }
      if (name != "--input" && name != "--output" && name != "--config" &&
          name != "--block-size" && name != "--frame-id" &&
          name != "--cell-encoding" && name != "--unknown-policy" &&
          name != "--max-map-bytes")
        throw std::invalid_argument("unknown argument: " + name);
      if (i + 1 == argc || std::string(argv[i + 1]).rfind("--", 0) == 0)
        throw std::invalid_argument("missing value for " + name);
      if (!flags.emplace(name, argv[++i]).second)
        throw std::invalid_argument("duplicate argument: " + name);
    }
    if (!flags.count("--input") || !flags.count("--output") ||
        flags.at("--input").empty() || flags.at("--output").empty())
      throw std::invalid_argument("--input and --output are required");

    vq_map_server::map2d::ConversionOptions options;
    if (flags.count("--config")) {
      const auto root = YAML::LoadFile(flags.at("--config"));
      if (!root.IsMap() || root.size() != 1)
        throw std::invalid_argument("config must contain only the /** mapping");
      const auto scope = root["/**"];
      if (!scope.IsMap() || scope.size() != 1)
        throw std::invalid_argument("config /** must contain only ros__parameters");
      const auto parameters = scope["ros__parameters"];
      if (!parameters.IsMap())
        throw std::invalid_argument("config ros__parameters must be a mapping");
      std::set<std::string> seen;
      for (const auto &entry : parameters) {
        if (!entry.first.IsScalar() || !entry.second.IsScalar())
          throw std::invalid_argument("configuration parameters must be scalar");
        const auto name = entry.first.as<std::string>();
        if (!seen.insert(name).second)
          throw std::invalid_argument("duplicate configuration parameter: " + name);
        apply(options, name, entry.second.as<std::string>());
      }
    }
    for (const auto &flag : flags) {
      std::string name = flag.first.substr(2);
      if (name == "input" || name == "output" || name == "config") continue;
      for (auto &c : name) if (c == '-') c = '_';
      apply(options, name, flag.second);
    }
    const auto map = vq_map_server::map2d::convertOccupancyGrid(flags.at("--input"), options);
    vq_map_server::map2d::writeHdf5(flags.at("--output"), map);
    struct rusage usage_info {};
    if (getrusage(RUSAGE_SELF, &usage_info) != 0)
      throw std::runtime_error("getrusage failed");
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "output: " << flags.at("--output") << '\n'
              << "memory_bytes: " << map.memoryBytes() << '\n'
              << "peak_rss_kib: " << usage_info.ru_maxrss << '\n'
              << "elapsed_ms: " << elapsed << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "vq_map_convert_2d: " << error.what() << '\n';
    return 1;
  }
}
