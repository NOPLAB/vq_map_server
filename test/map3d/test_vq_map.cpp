// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map3d/vq_map.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <set>

namespace {
using vq_map_server::map3d::CellState;
using vq_map_server::map3d::MapMetadata;
using vq_map_server::map3d::VqMap;

MapMetadata metadata() {
  MapMetadata value;
  value.voxel_size = 0.5F;
  value.origin = {-1.0F, -2.0F, -3.0F};
  value.dimensions = {5, 3, 2};
  value.block_size = 2;
  return value;
}

VqMap exampleMap() {
  return VqMap(metadata(), {0x81, 0x06}, {0, 1, 0, 1, 0, 1});
}

TEST(VqMap, ResolvesXFastestBlocksAndVoxelBitsWithoutExposingPadding) {
  const auto map = exampleMap();
  const std::set<std::array<std::int64_t, 3>> occupied{
      {0, 0, 0}, {1, 1, 1}, {3, 0, 0}, {2, 1, 0},
      {4, 0, 0}, {1, 2, 0}, {2, 2, 0}};

  for (std::int64_t z = 0; z < 2; ++z)
    for (std::int64_t y = 0; y < 3; ++y)
      for (std::int64_t x = 0; x < 5; ++x) {
        const std::array<std::int64_t, 3> cell{x, y, z};
        EXPECT_EQ(map.at(cell), occupied.count(cell) != 0 ?
            CellState::Occupied : CellState::Free)
            << x << "," << y << "," << z;
      }
  for (const std::array<std::int64_t, 3> cell :
       {std::array<std::int64_t, 3>{-1, 0, 0}, {5, 1, 1}, {0, 3, 0}, {0, 0, 2}})
    EXPECT_EQ(map.at(cell), CellState::OutOfBounds);
}

TEST(VqMap, FloorsWorldCoordinatesAndUsesHalfOpenMapBounds) {
  const auto map = exampleMap();
  struct Case { std::array<double, 3> point; CellState expected; };
  const Case cases[] = {
      {{-1.0, -2.0, -3.0}, CellState::Occupied},
      {{-0.25, -1.25, -2.25}, CellState::Occupied},
      {{-0.5, -2.0, -3.0}, CellState::Free},
      {{std::nextafter(-1.0, -2.0), -2.0, -3.0}, CellState::OutOfBounds},
      {{1.5, -2.0, -3.0}, CellState::OutOfBounds},
      {{-1.0, -0.5, -3.0}, CellState::OutOfBounds},
      {{-1.0, -2.0, -2.0}, CellState::OutOfBounds},
      {{std::numeric_limits<double>::quiet_NaN(), 0, 0}, CellState::OutOfBounds},
      {{std::numeric_limits<double>::infinity(), 0, 0}, CellState::OutOfBounds}};
  for (const auto &item : cases)
    EXPECT_EQ(map.atWorld(item.point), item.expected);
}

TEST(VqMap, VisualizationReturnsCellCentersInsideBoundsAndStopsAtPointLimit) {
  const auto map = exampleMap();

  const auto points = map.occupiedCenters({-0.5, -2, -3}, {1.5, -0.5, -2}, 3);

  const std::vector<std::array<double, 3>> expected{
      {0.75, -1.75, -2.75}, {1.25, -1.75, -2.75}, {0.25, -1.25, -2.75}};
  EXPECT_EQ(points, expected);
  EXPECT_TRUE(map.occupiedCenters({100, 100, 100}, {101, 101, 101}, 3).empty());
  EXPECT_THROW(map.occupiedCenters({0, 0, 0}, {1, 1, 1}, 0), std::invalid_argument);
  EXPECT_THROW(map.occupiedCenters({0, 0, 0}, {0, 1, 1}, 3), std::invalid_argument);
}

TEST(VqMap, RejectsInvalidMetadataAndIncompleteOrOutOfRangePayloads) {
  const auto valid = metadata();
  std::vector<MapMetadata> invalid;
  auto next = valid;
  next.voxel_size = 0;
  invalid.push_back(next);
  next = valid;
  next.origin[2] = std::numeric_limits<float>::infinity();
  invalid.push_back(next);
  next = valid;
  next.dimensions[0] = 0;
  invalid.push_back(next);
  next = valid;
  next.dimensions = {2147483647U, 2147483647U, 2147483647U};
  invalid.push_back(next);
  next = valid;
  next.block_size = 65;
  invalid.push_back(next);
  next = valid;
  next.frame_id = "invalid frame";
  invalid.push_back(next);
  next = valid;
  next.frame_id = std::string(4096, 'x');
  invalid.push_back(next);
  next = valid;
  next.frame_id = std::string("map\0suffix", 10);
  invalid.push_back(next);
  for (const auto &item : invalid)
    EXPECT_THROW(VqMap(item, {0}, {0, 0, 0, 0, 0, 0}), std::invalid_argument);

  EXPECT_THROW(VqMap(valid, {}, {0, 0, 0, 0, 0, 0}), std::invalid_argument);
  EXPECT_THROW(VqMap(valid, {0}, {0}), std::invalid_argument);
  EXPECT_THROW(VqMap(valid, {0}, {0, 0, 0, 0, 0, 1}), std::invalid_argument);
  next = valid;
  next.block_size = 3;
  EXPECT_THROW(VqMap(next, {0, 0, 0}, {0, 0}), std::invalid_argument);
}

}  // namespace
