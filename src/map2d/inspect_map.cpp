// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_io.hpp"

#include <charconv>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <sys/resource.h>

namespace {

std::size_t unsignedNumber(const std::string &text) {
  std::size_t result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    throw std::invalid_argument("invalid unsigned integer: " + text);
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const auto start = std::chrono::steady_clock::now();
    std::string path;
    std::size_t max_bytes = vq_map_server::map2d::kDefaultMaxMapBytes;
    std::size_t queries = 0;
    bool have_limit = false, have_queries = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "Usage: vq_map_inspect_2d H5 [--max-map-bytes N] [--queries N]\n";
        return 0;
      }
      if (arg == "--max-map-bytes" || arg == "--queries") {
        if (i + 1 == argc) throw std::invalid_argument("missing value for " + arg);
        bool &seen = arg == "--queries" ? have_queries : have_limit;
        if (seen) throw std::invalid_argument("duplicate argument: " + arg);
        seen = true;
        const auto value = unsignedNumber(argv[++i]);
        if (arg == "--queries") queries = value;
        else max_bytes = value;
      } else {
        if (arg.empty() || arg[0] == '-' || !path.empty())
          throw std::invalid_argument("unexpected argument: " + arg);
        path = arg;
      }
    }
    if (path.empty()) throw std::invalid_argument("HDF5 path is required");
    if (!max_bytes) throw std::invalid_argument("max-map-bytes must be positive");
    const auto map = vq_map_server::map2d::readHdf5(path, max_bytes);
    const auto load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const auto &m = map.metadata();
    std::cout << "frame_id: " << m.frame_id << '\n'
              << "resolution: " << m.resolution << '\n'
              << "origin: " << m.origin[0] << ' ' << m.origin[1] << '\n'
              << "yaw: " << m.yaw << '\n'
              << "dimensions: " << m.dimensions[0] << ' ' << m.dimensions[1] << '\n'
              << "block_size: " << m.block_size << '\n'
              << "cell_encoding: "
              << (m.encoding == vq_map_server::map2d::CellEncoding::Ternary
                      ? "ternary" : "binary") << '\n'
              << "index_bit_width: " << static_cast<unsigned>(map.indexBitWidth()) << '\n'
              << "blocks: " << map.indices().size() / (map.indexBitWidth() / 8) << '\n'
              << "dictionary_patterns: " << map.dictionarySize() << '\n'
              << "cell_count: " << map.cellCount() << '\n'
              << "memory_bytes: " << map.memoryBytes() << '\n';
    if (have_queries) {
      std::mt19937_64 random(0x56514d4150ULL);
      std::size_t occupied = 0, unknown = 0;
      const auto query_start = std::chrono::steady_clock::now();
      for (std::size_t i = 0; i < queries; ++i) {
        std::array<std::int64_t, 2> cell;
        // Rejection sampling is uniform and specifies the sequence across libraries.
        for (std::size_t a = 0; a < 2; ++a) {
          const auto bound = static_cast<std::uint64_t>(m.dimensions[a]);
          const auto threshold = (std::uint64_t{0} - bound) % bound;
          std::uint64_t sample;
          do { sample = random(); } while (sample < threshold);
          cell[a] = static_cast<std::int64_t>(sample % bound);
        }
        const auto state = map.at(cell);
        occupied += state == vq_map_server::map2d::CellState::Occupied;
        unknown += state == vq_map_server::map2d::CellState::Unknown;
      }
      const auto query_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - query_start).count();
      std::cout << "queries: " << queries << '\n'
                << "query_occupied: " << occupied << '\n'
                << "query_unknown: " << unknown << '\n'
                << "query_elapsed_ms: " << query_ms << '\n';
    }
    struct rusage usage_info {};
    if (getrusage(RUSAGE_SELF, &usage_info) != 0)
      throw std::runtime_error("getrusage failed");
    std::cout << "peak_rss_kib: " << usage_info.ru_maxrss << '\n'
              << "elapsed_ms: " << load_ms << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "vq_map_inspect_2d: " << error.what() << '\n';
    return 1;
  }
}
