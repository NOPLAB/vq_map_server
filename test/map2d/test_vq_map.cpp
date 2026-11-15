// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/vq_map.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using namespace vq_map_server::map2d;

TEST(Map2d, RecoversEveryCellAcrossPartialBlocksInBothEncodings) {
  for (const auto encoding : {CellEncoding::Binary, CellEncoding::Ternary}) {
    MapMetadata metadata;
    metadata.dimensions = {7, 5};
    metadata.block_size = 3;
    metadata.encoding = encoding;
    const unsigned bits = static_cast<unsigned>(encoding);
    const unsigned states = encoding == CellEncoding::Binary ? 2 : 3;
    const std::size_t bytes = (9 * bits + 7) / 8;
    std::vector<std::uint8_t> dictionary(6 * bytes, 0);
    for (unsigned y = 0; y < 5; ++y)
      for (unsigned x = 0; x < 7; ++x) {
        const auto block = x / 3 + 3 * (y / 3);
        const auto bit = (x % 3 + 3 * (y % 3)) * bits;
        dictionary[block * bytes + bit / 8] |=
            static_cast<std::uint8_t>(((x + 2 * y) % states) << (bit % 8));
      }
    const VqMap map(metadata, dictionary, {0, 1, 2, 3, 4, 5}, 8);
    EXPECT_EQ(map.blockDimensions(), (std::array<std::uint32_t, 2>{3, 2}));
    EXPECT_EQ(map.cellCount(), 35U);
    for (int y = 0; y < 5; ++y)
      for (int x = 0; x < 7; ++x) {
        const auto expected = (x + 2 * y) % states;
        EXPECT_EQ(map.at({x, y}), expected == 0 ? CellState::Free :
                  expected == 1 ? CellState::Occupied : CellState::Unknown)
            << x << "," << y;
      }
    EXPECT_EQ(map.at({7, 0}), CellState::OutOfBounds);
    EXPECT_EQ(map.at({0, 5}), CellState::OutOfBounds);
    EXPECT_EQ(map.at({-1, 0}), CellState::OutOfBounds);
  }
}

TEST(Map2d, InverseRotatesBeforeFloorAndExcludesWorldBoundaries) {
  MapMetadata metadata;
  metadata.dimensions = {2, 2};
  metadata.block_size = 2;
  metadata.origin = {-4.25, 3.5};
  metadata.yaw = 0.73;
  metadata.resolution = 0.5F;
  const VqMap map(metadata, {0x24}, {0}, 8);  // free, occupied, unknown, free
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 2; ++x) {
      const auto dx = (x + 0.5) * 0.5;
      const auto dy = (y + 0.5) * 0.5;
      EXPECT_EQ(map.atWorld({metadata.origin[0] + std::cos(metadata.yaw) * dx -
                            std::sin(metadata.yaw) * dy,
                            metadata.origin[1] + std::sin(metadata.yaw) * dx +
                            std::cos(metadata.yaw) * dy}), map.at({x, y}));
    }
  metadata.yaw = 0;
  const VqMap aligned(metadata, {0x24}, {0}, 8);
  EXPECT_EQ(aligned.atWorld({-4.25, 3.5}), CellState::Free);
  EXPECT_EQ(aligned.atWorld({-3.75, 3.5}), CellState::Occupied);
  for (const auto point : std::vector<std::array<double, 2>>{
           {-4.25001, 3.5}, {-3.25, 3.5}, {-4.25, 4.5},
           {std::numeric_limits<double>::max(), 3.5},
           {std::numeric_limits<double>::infinity(), 3.5},
           {0, std::numeric_limits<double>::quiet_NaN()}})
    EXPECT_EQ(aligned.atWorld(point), CellState::OutOfBounds);
}

TEST(Map2d, DecodesLittleEndianCompactIndicesAtEveryWidth) {
  for (const auto width : {8, 16, 32}) {
    const std::size_t count = width == 8 ? 256 : width == 16 ? 65536 : 65537;
    MapMetadata metadata;
    metadata.dimensions = {1, 1};
    metadata.block_size = 1;
    metadata.encoding = CellEncoding::Binary;
    std::vector<std::uint8_t> dictionary(count, 0);
    const std::size_t selected = width == 16 ? 0x1234 : count - 1;
    dictionary[selected] = 1;
    std::vector<std::uint8_t> indices(width / 8);
    for (std::size_t byte = 0; byte < indices.size(); ++byte)
      indices[byte] = static_cast<std::uint8_t>(selected >> (8 * byte));
    const VqMap map(metadata, dictionary, indices, static_cast<std::uint8_t>(width));
    EXPECT_EQ(map.at({0, 0}), CellState::Occupied);
    EXPECT_EQ(map.indices().size(), static_cast<std::size_t>(width / 8));
    EXPECT_EQ(map.dictionarySize(), count);
  }
}

TEST(Map2d, RejectsMalformedGeometryAndEnumsAtConstruction) {
  MapMetadata valid;
  valid.dimensions = {1, 1};
  valid.block_size = 1;
  std::vector<MapMetadata> invalid;
  auto add = [&](const auto &change) { auto m = valid; change(m); invalid.push_back(m); };
  add([](auto &m) { m.frame_id.clear(); });
  add([](auto &m) { m.frame_id = "bad frame"; });
  add([](auto &m) { m.frame_id = std::string("a\0b", 3); });
  add([](auto &m) { m.frame_id = std::string(4096, 'a'); });
  add([](auto &m) { m.resolution = 0; });
  add([](auto &m) { m.resolution = std::numeric_limits<float>::quiet_NaN(); });
  add([](auto &m) { m.origin[0] = std::numeric_limits<double>::infinity(); });
  add([](auto &m) { m.yaw = std::numeric_limits<double>::quiet_NaN(); });
  add([](auto &m) { m.dimensions[0] = 0; });
  add([](auto &m) { m.dimensions[1] = 0xffffffffU; });
  add([](auto &m) { m.block_size = 0; });
  add([](auto &m) { m.block_size = 65; });
  add([](auto &m) { m.encoding = static_cast<CellEncoding>(3); });
  for (const auto &metadata : invalid)
    EXPECT_THROW(VqMap(metadata, {0}, {0}, 8), std::invalid_argument);
}

TEST(Map2d, RejectsMalformedPayloadAndNonzeroPadding) {
  MapMetadata metadata;
  metadata.dimensions = {1, 1};
  metadata.block_size = 1;
  EXPECT_THROW(VqMap(metadata, {}, {0}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {0}, {}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {0}, {1}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {0}, {0}, 24), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {3}, {0}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {4}, {0}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, std::vector<std::uint8_t>(257, 0), {0}, 8),
               std::invalid_argument);
  metadata.block_size = 3;
  EXPECT_THROW(VqMap(metadata, {0}, {0}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {4, 0, 0}, {0}, 8), std::invalid_argument);
  EXPECT_THROW(VqMap(metadata, {0, 0, 128}, {0}, 8), std::invalid_argument);
  metadata.block_size = 64;
  EXPECT_NO_THROW(VqMap(metadata, std::vector<std::uint8_t>(1024, 0), {0}, 8));
}

}  // namespace
