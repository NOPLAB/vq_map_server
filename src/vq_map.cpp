// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/vq_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vq_map_server {

VqMap::VqMap(MapMetadata metadata, std::vector<std::uint8_t> dictionary,
             std::vector<std::uint32_t> indices)
    : metadata_(std::move(metadata)), dictionary_(std::move(dictionary)),
      indices_(std::move(indices)) {
  if (metadata_.frame_id.empty() || metadata_.frame_id.size() > 4095 ||
      metadata_.frame_id.find('\0') != std::string::npos ||
      metadata_.frame_id.find_first_of(" \t\r\n") != std::string::npos)
    throw std::invalid_argument("frame_id must have 1..4095 characters and no whitespace");
  if (!std::isfinite(metadata_.voxel_size) || metadata_.voxel_size <= 0)
    throw std::invalid_argument("voxel_size must be finite and positive");
  if (metadata_.block_size == 0 || metadata_.block_size > 64)
    throw std::invalid_argument("block_size must be between 1 and 64");
  const std::uint64_t side = metadata_.block_size;
  pattern_bytes_ = static_cast<std::size_t>((side * side * side + 7) / 8);
  voxel_count_ = 1;
  std::size_t block_count = 1;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto dim = metadata_.dimensions[axis];
    if (!std::isfinite(metadata_.origin[axis]) || dim == 0 ||
        dim > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
      throw std::invalid_argument("origin must be finite and dimensions must be positive int32");
    if (voxel_count_ > std::numeric_limits<std::uint64_t>::max() / dim)
      throw std::invalid_argument("voxel count overflows uint64");
    voxel_count_ *= dim;
    block_dimensions_[axis] = static_cast<std::uint32_t>((dim + side - 1) / side);
    if (block_count > std::numeric_limits<std::size_t>::max() / block_dimensions_[axis])
      throw std::invalid_argument("block count overflows size_t");
    block_count *= block_dimensions_[axis];
  }
  if (dictionary_.empty() || dictionary_.size() % pattern_bytes_ != 0 ||
      dictionarySize() > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("dictionary length must contain complete nonempty patterns");
  if (indices_.size() != block_count)
    throw std::invalid_argument("index count does not match block dimensions");
  for (const auto index : indices_)
    if (index >= dictionarySize())
      throw std::invalid_argument("block index is outside dictionary");
}

std::size_t VqMap::memoryBytes() const noexcept {
  return sizeof(*this) + metadata_.frame_id.capacity() + dictionary_.capacity() +
         indices_.capacity() * sizeof(std::uint32_t);
}

CellState VqMap::at(const std::array<std::int64_t, 3> &cell) const noexcept {
  for (std::size_t axis = 0; axis < 3; ++axis)
    if (cell[axis] < 0 || static_cast<std::uint64_t>(cell[axis]) >= metadata_.dimensions[axis])
      return CellState::OutOfBounds;
  const std::size_t side = metadata_.block_size;
  const std::size_t x = static_cast<std::size_t>(cell[0]);
  const std::size_t y = static_cast<std::size_t>(cell[1]);
  const std::size_t z = static_cast<std::size_t>(cell[2]);
  const auto block = x / side + block_dimensions_[0] *
      (y / side + static_cast<std::size_t>(block_dimensions_[1]) * (z / side));
  const auto bit = x % side + side * (y % side + side * (z % side));
  return (dictionary_[static_cast<std::size_t>(indices_[block]) * pattern_bytes_ + bit / 8] &
          (1U << (bit % 8))) != 0 ? CellState::Occupied : CellState::Free;
}

CellState VqMap::atWorld(const std::array<double, 3> &point) const noexcept {
  std::array<std::int64_t, 3> cell{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto coordinate = std::floor(
        (point[axis] - metadata_.origin[axis]) / metadata_.voxel_size);
    if (!std::isfinite(coordinate) || coordinate < 0 ||
        coordinate >= metadata_.dimensions[axis])
      return CellState::OutOfBounds;
    cell[axis] = static_cast<std::int64_t>(coordinate);
  }
  return at(cell);
}

std::vector<std::array<double, 3>> VqMap::occupiedCenters(
    const std::array<double, 3> &minimum,
    const std::array<double, 3> &maximum, std::size_t max_points) const {
  if (max_points == 0)
    throw std::invalid_argument("max_points must be positive");
  std::array<std::int64_t, 3> begin{}, end{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(minimum[axis]) || !std::isfinite(maximum[axis]) ||
        minimum[axis] >= maximum[axis])
      throw std::invalid_argument("visualization bounds must be finite and increasing");
    const auto lower = std::ceil(
        (minimum[axis] - metadata_.origin[axis]) / metadata_.voxel_size - 0.5);
    const auto upper = std::ceil(
        (maximum[axis] - metadata_.origin[axis]) / metadata_.voxel_size - 0.5);
    begin[axis] = static_cast<std::int64_t>(std::clamp(lower, 0.0,
        static_cast<double>(metadata_.dimensions[axis])));
    end[axis] = static_cast<std::int64_t>(std::clamp(upper, 0.0,
        static_cast<double>(metadata_.dimensions[axis])));
  }
  std::vector<std::array<double, 3>> points;
  for (auto z = begin[2]; z < end[2]; ++z)
    for (auto y = begin[1]; y < end[1]; ++y)
      for (auto x = begin[0]; x < end[0]; ++x) {
        if (at({x, y, z}) != CellState::Occupied)
          continue;
        points.push_back({
            metadata_.origin[0] + (static_cast<double>(x) + 0.5) * metadata_.voxel_size,
            metadata_.origin[1] + (static_cast<double>(y) + 0.5) * metadata_.voxel_size,
            metadata_.origin[2] + (static_cast<double>(z) + 0.5) * metadata_.voxel_size});
        if (points.size() == max_points)
          return points;
      }
  return points;
}

}  // namespace vq_map_server
