// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_converter.hpp"
#include "../common/pattern_dictionary.hpp"

#include <nav2_map_server/map_io.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vq_map_server::map2d {
namespace {

std::string extension(const std::string &path) {
  auto value = std::filesystem::path(path).extension().string();
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

VqMap convertOccupancyGrid(const std::string &path, const ConversionOptions &options) {
  if (extension(path) != ".yaml" && extension(path) != ".yml")
    throw std::invalid_argument("input must be YAML or YML");
  if (options.block_size == 0 || options.block_size > 64 ||
      (options.encoding != CellEncoding::Binary && options.encoding != CellEncoding::Ternary))
    throw std::invalid_argument("invalid block size or encoding");
  if (options.unknown_policy != UnknownPolicy::Reject &&
      options.unknown_policy != UnknownPolicy::Free &&
      options.unknown_policy != UnknownPolicy::Occupied)
    throw std::invalid_argument("invalid unknown policy");
  if (options.encoding == CellEncoding::Ternary && options.unknown_policy != UnknownPolicy::Reject)
    throw std::invalid_argument("unknown replacement only applies to binary encoding");
  const auto parameters = nav2_map_server::loadMapYaml(path);
  if (parameters.mode != nav2_map_server::MapMode::Trinary)
    throw std::invalid_argument("only Nav2 trinary mode is supported");
  const auto image_extension = extension(parameters.image_file_name);
  if (image_extension != ".pgm" && image_extension != ".png")
    throw std::invalid_argument("map image must be PGM or PNG");
  if (!std::isfinite(parameters.resolution) || parameters.resolution <= 0 ||
      parameters.resolution > std::numeric_limits<float>::max() ||
      static_cast<float>(parameters.resolution) <= 0 ||
      !std::all_of(parameters.origin.begin(), parameters.origin.end(),
                   [](double v) { return std::isfinite(v); }) ||
      !std::isfinite(parameters.free_thresh) || !std::isfinite(parameters.occupied_thresh) ||
      parameters.free_thresh < 0 || parameters.occupied_thresh > 1 ||
      parameters.free_thresh >= parameters.occupied_thresh)
    throw std::invalid_argument("invalid YAML geometry or thresholds");
  MapMetadata metadata;
  metadata.frame_id = options.frame_id;
  metadata.resolution = static_cast<float>(parameters.resolution);
  metadata.origin = {parameters.origin[0], parameters.origin[1]};
  metadata.yaw = parameters.origin[2];
  metadata.block_size = options.block_size;
  metadata.encoding = options.encoding;
  // Nav2 owns image inversion, alpha, negate, and threshold interpretation.
  // Its decoded grid and our unpacked uint32 block IDs are transient scratch;
  // max_map_bytes constrains the final dictionary plus compact index payload.
  nav_msgs::msg::OccupancyGrid grid;
  nav2_map_server::loadMapFromFile(parameters, grid);
  metadata.dimensions = {grid.info.width, grid.info.height};
  const std::uint64_t cells = std::uint64_t{grid.info.width} * grid.info.height;
  if (grid.info.width == 0 || grid.info.height == 0 ||
      grid.info.width > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) ||
      grid.info.height > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) ||
      cells != grid.data.size())
    throw std::invalid_argument("invalid image dimensions or decoded cell count");
  const std::size_t side = options.block_size;
  const std::size_t bits = static_cast<std::uint8_t>(options.encoding);
  const std::size_t pattern_bytes = (side * side * bits + 7) / 8;
  const auto nx = (std::uint64_t{grid.info.width} + side - 1) / side;
  const auto ny = (std::uint64_t{grid.info.height} + side - 1) / side;
  if (nx > std::numeric_limits<std::size_t>::max() / ny ||
      nx * ny > options.max_map_bytes)
    throw std::runtime_error("compact index payload exceeds max_map_bytes");
  common::PatternDictionary builder(pattern_bytes, options.max_map_bytes);
  std::vector<std::uint32_t> block_ids;
  block_ids.reserve(static_cast<std::size_t>(nx * ny));
  for (std::size_t by = 0; by < ny; ++by)
    for (std::size_t bx = 0; bx < nx; ++bx) {
      std::vector<std::uint8_t> pattern(pattern_bytes, 0);
      for (std::size_t y = 0; y < side && by * side + y < grid.info.height; ++y)
        for (std::size_t x = 0; x < side && bx * side + x < grid.info.width; ++x) {
          const auto value = grid.data[(by * side + y) * grid.info.width + bx * side + x];
          std::uint8_t code = 0;
          if (value == 100) {
            code = 1;
          } else if (value == -1) {
            if (options.encoding == CellEncoding::Ternary) {
              code = 2;
            } else {
              switch (options.unknown_policy) {
                case UnknownPolicy::Reject:
                  throw std::invalid_argument("binary map contains unknown cells");
                case UnknownPolicy::Free: code = 0; break;
                case UnknownPolicy::Occupied: code = 1; break;
              }
            }
          } else if (value != 0) {
            throw std::invalid_argument("Nav2 trinary loader returned an invalid occupancy");
          }
          const auto bit = (x + side * y) * bits;
          pattern[bit / 8] |= static_cast<std::uint8_t>(code << (bit % 8));
        }
      block_ids.push_back(builder.add(pattern));
    }
  const std::uint8_t width = builder.size() <= 256 ? 8 : builder.size() <= 65536 ? 16 : 32;
  const std::size_t bytes = width / 8;
  auto dictionary = std::move(builder).take();
  if (block_ids.size() > (options.max_map_bytes - dictionary.size()) / bytes)
    throw std::runtime_error("dictionary and compact index payload exceed max_map_bytes");
  std::vector<std::uint8_t> indices(block_ids.size() * bytes);
  for (std::size_t block = 0; block < block_ids.size(); ++block)
    for (std::size_t byte = 0; byte < bytes; ++byte)
      indices[block * bytes + byte] = static_cast<std::uint8_t>(block_ids[block] >> (8 * byte));
  return VqMap(std::move(metadata), std::move(dictionary), std::move(indices), width);
}

}  // namespace vq_map_server::map2d
