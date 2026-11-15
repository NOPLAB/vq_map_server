// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vq_map_server::map2d {

enum class CellState { Free, Occupied, Unknown, OutOfBounds };
enum class CellEncoding : std::uint8_t { Binary = 1, Ternary = 2 };

struct MapMetadata {
  std::string frame_id = "map";
  float resolution = 0.05F;
  std::array<double, 2> origin{};
  double yaw = 0;
  std::array<std::uint32_t, 2> dimensions{};
  std::uint32_t block_size = 24;
  CellEncoding encoding = CellEncoding::Ternary;
};

// Immutable compressed grid. Blocks and cells are x-fastest, bits LSB-first.
// Binary codes: 0 free, 1 occupied. Ternary adds 2 unknown; code 3 is invalid.
// Indices are little-endian byte records of 8, 16, or 32 bits.
class VqMap {
public:
  VqMap(MapMetadata metadata, std::vector<std::uint8_t> dictionary,
        std::vector<std::uint8_t> indices, std::uint8_t index_bit_width);

  const MapMetadata &metadata() const noexcept { return metadata_; }
  const std::array<std::uint32_t, 2> &blockDimensions() const noexcept {
    return block_dimensions_;
  }
  const std::vector<std::uint8_t> &dictionary() const noexcept { return dictionary_; }
  const std::vector<std::uint8_t> &indices() const noexcept { return indices_; }
  std::uint8_t indexBitWidth() const noexcept { return index_bit_width_; }
  std::size_t patternBytes() const noexcept { return pattern_bytes_; }
  std::size_t dictionarySize() const noexcept { return dictionary_.size() / pattern_bytes_; }
  std::uint64_t cellCount() const noexcept { return cell_count_; }
  std::size_t memoryBytes() const noexcept;
  CellState at(const std::array<std::int64_t, 2> &cell) const noexcept;
  CellState atWorld(const std::array<double, 2> &point) const noexcept;

private:
  MapMetadata metadata_;
  std::array<std::uint32_t, 2> block_dimensions_{};
  std::size_t pattern_bytes_ = 0;
  std::uint64_t cell_count_ = 0;
  std::uint8_t index_bit_width_ = 8;
  double cosine_ = 1;
  double sine_ = 0;
  std::vector<std::uint8_t> dictionary_;
  std::vector<std::uint8_t> indices_;
};

}  // namespace vq_map_server::map2d
