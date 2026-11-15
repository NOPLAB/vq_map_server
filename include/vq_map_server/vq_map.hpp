// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vq_map_server {

struct MapMetadata {
  std::string frame_id = "map";
  float voxel_size = 0.1F;
  std::array<float, 3> origin{};
  std::array<std::uint32_t, 3> dimensions{};
  std::uint32_t block_size = 13;
};

enum class CellState { Free, Occupied, OutOfBounds };

// Validated on construction; all queries are immutable and thread-safe.
// Both block and voxel order are x + width_x * (y + width_y * z).
// Each dictionary pattern is byte-aligned and its first voxel is bit 0.
class VqMap {
public:
  VqMap(MapMetadata metadata, std::vector<std::uint8_t> dictionary,
        std::vector<std::uint32_t> indices);

  const MapMetadata &metadata() const noexcept { return metadata_; }
  const std::array<std::uint32_t, 3> &blockDimensions() const noexcept {
    return block_dimensions_;
  }
  const std::vector<std::uint8_t> &dictionary() const noexcept { return dictionary_; }
  const std::vector<std::uint32_t> &indices() const noexcept { return indices_; }
  std::size_t patternBytes() const noexcept { return pattern_bytes_; }
  std::size_t dictionarySize() const noexcept { return dictionary_.size() / pattern_bytes_; }
  std::uint64_t voxelCount() const noexcept { return voxel_count_; }
  std::size_t memoryBytes() const noexcept;

  CellState at(const std::array<std::int64_t, 3> &cell) const noexcept;
  CellState atWorld(const std::array<double, 3> &point) const noexcept;
  // Half-open world-space bounds; returns at most max_points cell centers.
  std::vector<std::array<double, 3>> occupiedCenters(
      const std::array<double, 3> &minimum,
      const std::array<double, 3> &maximum, std::size_t max_points) const;

private:
  MapMetadata metadata_;
  std::array<std::uint32_t, 3> block_dimensions_{};
  std::size_t pattern_bytes_ = 0;
  std::uint64_t voxel_count_ = 0;
  std::vector<std::uint8_t> dictionary_;
  std::vector<std::uint32_t> indices_;
};

}  // namespace vq_map_server
