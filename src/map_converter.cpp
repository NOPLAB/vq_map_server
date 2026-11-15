// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
// Modified from vq_occupancy_compressor, commit 6157adcde9bd94b796f67dfaf1ffbd67492e3827:
// PatternDictionaryBuilder.cpp first-occurrence map/vector deduplication and
// VoxelGrid.cpp x-fastest, least-significant-bit-first byte patterns.
// Modifications: generic PCL input, checked geometry, sparse block assembly,
// bounded uint32 indices, and a flat dictionary.
#include "vq_map_server/map_converter.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vq_map_server {
namespace {

std::size_t checkedMultiply(std::size_t a, std::size_t b) {
  if (b && a > std::numeric_limits<std::size_t>::max() / b)
    throw std::runtime_error("point-cloud geometry exceeds addressable size");
  return a * b;
}

double coordinate(const std::uint8_t *data, const pcl::PCLPointField &field,
                  bool swap) {
  const std::size_t size = field.datatype == pcl::PCLPointField::FLOAT32 ? 4 : 8;
  std::array<std::uint8_t, 8> bytes{};
  std::copy_n(data + field.offset, size, bytes.begin());
  if (swap) std::reverse(bytes.begin(), bytes.begin() + size);
  if (size == 4) {
    float value;
    std::memcpy(&value, bytes.data(), 4);
    return value;
  }
  double value;
  std::memcpy(&value, bytes.data(), 8);
  return value;
}

}  // namespace

VqMap convertPointCloud(const std::string &path, const ConversionOptions &options) {
  if (!std::isfinite(options.voxel_size) || options.voxel_size <= 0 ||
      options.block_size < 1 || options.block_size > 64 ||
      options.frame_id.empty() || options.frame_id.size() > 4095 ||
      options.frame_id.find('\0') != std::string::npos ||
      options.frame_id.find_first_of(" \t\r\n") != std::string::npos ||
      options.max_map_bytes == 0)
    throw std::invalid_argument("invalid conversion options");

  pcl::PCLPointCloud2 cloud;
  auto extension = std::filesystem::path(path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  int result;
  if (extension == ".pcd") result = pcl::io::loadPCDFile(path, cloud);
  else if (extension == ".ply") result = pcl::io::loadPLYFile(path, cloud);
  else throw std::invalid_argument("input must be a PCD or PLY file");
  if (result < 0) throw std::runtime_error("cannot read point cloud: " + path);
  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0)
    throw std::runtime_error("empty point cloud");

  std::array<pcl::PCLPointField, 3> fields;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    std::size_t matches = 0;
    for (const auto &field : cloud.fields) {
      if (field.name != names[axis]) continue;
      ++matches;
      fields[axis] = field;
    }
    const auto &field = fields[axis];
    if (matches != 1 || field.count != 1 ||
        (field.datatype != pcl::PCLPointField::FLOAT32 &&
         field.datatype != pcl::PCLPointField::FLOAT64))
      throw std::runtime_error("XYZ fields must be scalar FLOAT32 or FLOAT64");
    const std::size_t bytes = field.datatype == pcl::PCLPointField::FLOAT32 ? 4 : 8;
    if (field.offset > cloud.point_step || bytes > cloud.point_step - field.offset)
      throw std::runtime_error("XYZ field exceeds point record");
  }
  const auto row_bytes = checkedMultiply(cloud.width, cloud.point_step);
  if (cloud.row_step < row_bytes ||
      checkedMultiply(cloud.row_step, cloud.height) > cloud.data.size())
    throw std::runtime_error("malformed point-cloud data layout");
  const std::uint16_t endian_probe = 1;
  const bool host_big = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
  const bool swap = static_cast<bool>(cloud.is_bigendian) != host_big;

  // Visit the PCL storage twice instead of making a narrowed PointXYZ copy.
  auto visit = [&](auto &&consume) {
    for (std::size_t y = 0; y < cloud.height; ++y) {
      for (std::size_t x = 0; x < cloud.width; ++x) {
        const auto *data = cloud.data.data() + y * cloud.row_step + x * cloud.point_step;
        std::array<double, 3> point;
        for (std::size_t a = 0; a < 3; ++a) point[a] = coordinate(data, fields[a], swap);
        if (std::all_of(point.begin(), point.end(),
                        [](double value) { return std::isfinite(value); }))
          consume(point);
      }
    }
  };
  std::array<double, 3> minimum, maximum;
  minimum.fill(std::numeric_limits<double>::infinity());
  maximum.fill(-std::numeric_limits<double>::infinity());
  bool finite = false;
  visit([&](const auto &point) {
    finite = true;
    for (std::size_t a = 0; a < 3; ++a) {
      minimum[a] = std::min(minimum[a], point[a]);
      maximum[a] = std::max(maximum[a], point[a]);
    }
  });
  if (!finite) throw std::runtime_error("point cloud has no finite XYZ points");

  MapMetadata metadata;
  metadata.frame_id = options.frame_id;
  metadata.voxel_size = options.voxel_size;
  metadata.block_size = options.block_size;
  std::array<std::size_t, 3> blocks{};
  std::size_t block_count = 1;
  std::size_t voxel_count = 1;
  for (std::size_t a = 0; a < 3; ++a) {
    if (minimum[a] < -std::numeric_limits<float>::max() ||
        minimum[a] > std::numeric_limits<float>::max())
      throw std::runtime_error("origin cannot be represented as float");
    metadata.origin[a] = static_cast<float>(minimum[a]);
    // The persisted float origin must not exclude a FLOAT64 minimum by rounding up.
    if (static_cast<double>(metadata.origin[a]) > minimum[a])
      metadata.origin[a] = std::nextafter(metadata.origin[a],
                                         -std::numeric_limits<float>::infinity());
    if (!std::isfinite(metadata.origin[a]))
      throw std::runtime_error("origin cannot be represented as finite float");
    const double last = std::floor((maximum[a] - metadata.origin[a]) /
                                   static_cast<double>(options.voxel_size));
    if (!std::isfinite(last) || last < 0 ||
        last >= std::numeric_limits<std::int32_t>::max())
      throw std::runtime_error("voxel dimensions exceed positive int32 range");
    metadata.dimensions[a] = static_cast<std::uint32_t>(last) + 1;
    voxel_count = checkedMultiply(voxel_count, metadata.dimensions[a]);
    blocks[a] = (static_cast<std::size_t>(metadata.dimensions[a]) +
                 options.block_size - 1) / options.block_size;
    block_count = checkedMultiply(block_count, blocks[a]);
  }
  const std::size_t size = options.block_size;
  const std::size_t pattern_bytes = (size * size * size + 7) / 8;
  const std::size_t index_bytes = checkedMultiply(block_count, sizeof(std::uint32_t));
  if (index_bytes > options.max_map_bytes ||
      pattern_bytes > options.max_map_bytes - index_bytes)
    throw std::runtime_error("map exceeds max_map_bytes");

  std::map<std::size_t, std::vector<std::uint8_t>> occupied;
  visit([&](const auto &point) {
    std::array<std::size_t, 3> cell;
    for (std::size_t a = 0; a < 3; ++a)
      cell[a] = static_cast<std::size_t>(std::floor(
          (point[a] - metadata.origin[a]) / static_cast<double>(options.voxel_size)));
    const auto block = cell[0] / size + blocks[0] *
        (cell[1] / size + blocks[1] * (cell[2] / size));
    auto it = occupied.try_emplace(block, pattern_bytes, 0).first;
    const auto bit = cell[0] % size + size * (cell[1] % size + size * (cell[2] % size));
    it->second[bit / 8] |= static_cast<std::uint8_t>(1u << (bit % 8));
  });

  std::vector<std::uint8_t> dictionary;
  std::vector<std::uint32_t> indices;
  indices.reserve(block_count);
  std::map<std::vector<std::uint8_t>, std::uint32_t> pattern_to_index;
  const std::vector<std::uint8_t> empty(pattern_bytes, 0);
  auto occupied_it = occupied.begin();
  // Adapted upstream first-occurrence assignment: traversal order, not map key
  // ordering, determines dictionary IDs.
  for (std::size_t block = 0; block < block_count; ++block) {
    const auto &pattern = occupied_it != occupied.end() && occupied_it->first == block
        ? occupied_it->second : empty;
    auto it = pattern_to_index.find(pattern);
    if (it == pattern_to_index.end()) {
      const auto next_index = dictionary.size() / pattern_bytes;
      if (next_index >= std::numeric_limits<std::uint32_t>::max() ||
          pattern_bytes > options.max_map_bytes - index_bytes - dictionary.size())
        throw std::runtime_error("dictionary exceeds max_map_bytes or uint32 range");
      it = pattern_to_index.emplace(pattern, static_cast<std::uint32_t>(next_index)).first;
      dictionary.insert(dictionary.end(), pattern.begin(), pattern.end());
    }
    indices.push_back(it->second);
    if (occupied_it != occupied.end() && occupied_it->first == block) ++occupied_it;
  }
  return VqMap(std::move(metadata), std::move(dictionary), std::move(indices));
}

}  // namespace vq_map_server
