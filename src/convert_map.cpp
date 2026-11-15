// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map_converter.hpp"
#include <yaml-cpp/yaml.h>

#include <charconv>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <sys/resource.h>

namespace {

template <typename T> T unsignedNumber(const std::string &text) {
  T result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    throw std::invalid_argument("invalid unsigned integer: " + text);
  return result;
}

float voxelSize(const std::string &text) {
  float value{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value,
                                      std::chars_format::general);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      !std::isfinite(value) || value <= 0)
    throw std::invalid_argument("invalid voxel size: " + text);
  return value;
}

void apply(vq_map_server::ConversionOptions &options, const std::string &name,
           const std::string &value) {
  if (name == "voxel_size") options.voxel_size = voxelSize(value);
  else if (name == "block_size") options.block_size = unsignedNumber<std::uint32_t>(value);
  else if (name == "frame_id") options.frame_id = value;
  else if (name == "max_map_bytes") options.max_map_bytes = unsignedNumber<std::size_t>(value);
  else throw std::invalid_argument("unknown configuration parameter: " + name);
}

void usage() {
  std::cout << "Usage: vq_map_convert --input FILE --output FILE [--config FILE]\n"
               "  [--voxel-size M] [--block-size N] [--frame-id FRAME] [--max-map-bytes N]\n";
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
          name != "--voxel-size" && name != "--block-size" &&
          name != "--frame-id" && name != "--max-map-bytes")
        throw std::invalid_argument("unknown argument: " + name);
      if (i + 1 == argc || std::string(argv[i + 1]).rfind("--", 0) == 0)
        throw std::invalid_argument("missing value for " + name);
      if (!flags.emplace(name, argv[++i]).second)
        throw std::invalid_argument("duplicate argument: " + name);
    }
    if (!flags.count("--input") || !flags.count("--output") ||
        flags.at("--input").empty() || flags.at("--output").empty())
      throw std::invalid_argument("--input and --output are required");
    vq_map_server::ConversionOptions options;
    if (flags.count("--config")) {
      const auto root = YAML::LoadFile(flags.at("--config"));
      const auto scope = root["/**"];
      const auto parameters = scope ? scope["ros__parameters"] : YAML::Node{};
      if (!root.IsMap() || !scope.IsMap() || !parameters.IsMap())
        throw std::invalid_argument("config must contain /**: ros__parameters mapping");
      for (const auto &entry : parameters) {
        if (!entry.first.IsScalar() || !entry.second.IsScalar())
          throw std::invalid_argument("configuration parameters must be scalar");
        apply(options, entry.first.as<std::string>(), entry.second.as<std::string>());
      }
    }
    for (const auto &flag : flags) {
      std::string name = flag.first.substr(2);
      if (name == "input" || name == "output" || name == "config") continue;
      for (auto &c : name) if (c == '-') c = '_';
      apply(options, name, flag.second);
    }
    const auto map = vq_map_server::convertPointCloud(flags.at("--input"), options);
    vq_map_server::writeHdf5(flags.at("--output"), map);
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
    std::cerr << "vq_map_convert: " << error.what() << '\n';
    return 1;
  }
}
