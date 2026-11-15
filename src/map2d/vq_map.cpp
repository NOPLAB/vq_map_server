// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/vq_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vq_map_server::map2d {
namespace {

std::uint32_t readIndex(const std::vector<std::uint8_t> &indices,
                        std::size_t block, std::uint8_t width) noexcept {
  std::uint32_t index = 0;
  const std::size_t bytes = width / 8;
  for (std::size_t byte = 0; byte < bytes; ++byte)
    index |= static_cast<std::uint32_t>(indices[block * bytes + byte]) << (8 * byte);
  return index;
}

}  // namespace

VqMap::VqMap(MapMetadata metadata, std::vector<std::uint8_t> dictionary,
             std::vector<std::uint8_t> indices, std::uint8_t index_bit_width)
    : metadata_(std::move(metadata)), index_bit_width_(index_bit_width),
      dictionary_(std::move(dictionary)), indices_(std::move(indices)) {
  if (metadata_.frame_id.empty() || metadata_.frame_id.size() > 4095 ||
      std::any_of(metadata_.frame_id.begin(), metadata_.frame_id.end(),
                  [](unsigned char c) { return c == 0 || std::isspace(c); }))
    throw std::invalid_argument("frame_id must have 1..4095 characters without whitespace or NUL");
  if (!std::isfinite(metadata_.resolution) || metadata_.resolution <= 0 ||
      !std::isfinite(metadata_.yaw))
    throw std::invalid_argument("resolution must be finite and positive; yaw must be finite");
  if (metadata_.block_size == 0 || metadata_.block_size > 64)
    throw std::invalid_argument("block_size must be between 1 and 64");
  if (metadata_.encoding != CellEncoding::Binary && metadata_.encoding != CellEncoding::Ternary)
    throw std::invalid_argument("unsupported cell encoding");
  if (index_bit_width_ != 8 && index_bit_width_ != 16 && index_bit_width_ != 32)
    throw std::invalid_argument("index width must be 8, 16, or 32");
  const std::size_t side = metadata_.block_size;
  const std::size_t bits = static_cast<std::uint8_t>(metadata_.encoding);
  const auto pattern_bits = side * side * bits;
  pattern_bytes_ = (pattern_bits + 7) / 8;
  cell_count_ = 1;
  std::size_t block_count = 1;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const auto dim = metadata_.dimensions[axis];
    if (!std::isfinite(metadata_.origin[axis]) || dim == 0 ||
        dim > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
      throw std::invalid_argument("origin must be finite and dimensions must be positive int32");
    if (cell_count_ > std::numeric_limits<std::uint64_t>::max() / dim)
      throw std::invalid_argument("cell count overflows uint64");
    cell_count_ *= dim;
    block_dimensions_[axis] = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(dim) + side - 1) / side);
    if (block_count > std::numeric_limits<std::size_t>::max() / block_dimensions_[axis])
      throw std::invalid_argument("block count overflows size_t");
    block_count *= block_dimensions_[axis];
  }
  if (dictionary_.empty() || dictionary_.size() % pattern_bytes_ != 0 ||
      dictionarySize() > std::numeric_limits<std::uint32_t>::max() ||
      dictionarySize() > (std::uint64_t{1} << index_bit_width_))
    throw std::invalid_argument("dictionary size is invalid for pattern or index width");
  const std::size_t index_bytes = index_bit_width_ / 8;
  if (block_count > std::numeric_limits<std::size_t>::max() / index_bytes ||
      indices_.size() != block_count * index_bytes)
    throw std::invalid_argument("index byte count does not match block dimensions");
  for (std::size_t pattern = 0; pattern < dictionarySize(); ++pattern) {
    const auto offset = pattern * pattern_bytes_;
    if (pattern_bits % 8 != 0 &&
        (dictionary_[offset + pattern_bytes_ - 1] >> (pattern_bits % 8)) != 0)
      throw std::invalid_argument("unused pattern bits must be zero");
    if (metadata_.encoding == CellEncoding::Ternary)
      for (std::size_t cell = 0; cell < side * side; ++cell)
        if (((dictionary_[offset + cell / 4] >> (2 * (cell % 4))) & 3U) == 3)
          throw std::invalid_argument("invalid ternary cell code");
  }
  for (std::size_t block = 0; block < block_count; ++block) {
    const auto index = readIndex(indices_, block, index_bit_width_);
    if (index >= dictionarySize())
      throw std::invalid_argument("block index is outside dictionary");
    const auto bx = block % block_dimensions_[0];
    const auto by = block / block_dimensions_[0];
    if ((bx + 1) * side <= metadata_.dimensions[0] &&
        (by + 1) * side <= metadata_.dimensions[1])
      continue;
    for (std::size_t y = 0; y < side; ++y)
      for (std::size_t x = 0; x < side; ++x) {
        if (bx * side + x < metadata_.dimensions[0] &&
            by * side + y < metadata_.dimensions[1])
          continue;
        const auto bit = (x + side * y) * bits;
        if (((dictionary_[index * pattern_bytes_ + bit / 8] >> (bit % 8)) &
             ((1U << bits) - 1)) != 0)
          throw std::invalid_argument("padded cells must be free");
      }
  }
  cosine_ = std::cos(metadata_.yaw);
  sine_ = std::sin(metadata_.yaw);
}

std::size_t VqMap::memoryBytes() const noexcept {
  return sizeof(*this) + metadata_.frame_id.capacity() +
         dictionary_.capacity() + indices_.capacity();
}

CellState VqMap::at(const std::array<std::int64_t, 2> &cell) const noexcept {
  for (std::size_t axis = 0; axis < 2; ++axis)
    if (cell[axis] < 0 || static_cast<std::uint64_t>(cell[axis]) >= metadata_.dimensions[axis])
      return CellState::OutOfBounds;
  const std::size_t side = metadata_.block_size;
  const auto x = static_cast<std::size_t>(cell[0]);
  const auto y = static_cast<std::size_t>(cell[1]);
  const auto block = x / side + static_cast<std::size_t>(block_dimensions_[0]) * (y / side);
  const auto index = readIndex(indices_, block, index_bit_width_);
  const auto bits = static_cast<std::uint8_t>(metadata_.encoding);
  const auto bit = (x % side + side * (y % side)) * bits;
  const auto code = (dictionary_[index * pattern_bytes_ + bit / 8] >> (bit % 8)) &
                    ((1U << bits) - 1);
  return code == 0 ? CellState::Free : code == 1 ? CellState::Occupied : CellState::Unknown;
}

CellState VqMap::atWorld(const std::array<double, 2> &point) const noexcept {
  if (!std::isfinite(point[0]) || !std::isfinite(point[1]))
    return CellState::OutOfBounds;
  const double dx = point[0] - metadata_.origin[0];
  const double dy = point[1] - metadata_.origin[1];
  const std::array<double, 2> local{
      (cosine_ * dx + sine_ * dy) / metadata_.resolution,
      (-sine_ * dx + cosine_ * dy) / metadata_.resolution};
  std::array<std::int64_t, 2> cell{};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const auto coordinate = std::floor(local[axis]);
    if (!std::isfinite(coordinate) || coordinate < 0 || coordinate >= metadata_.dimensions[axis])
      return CellState::OutOfBounds;
    cell[axis] = static_cast<std::int64_t>(coordinate);
  }
  return at(cell);
}

}  // namespace vq_map_server::map2d
