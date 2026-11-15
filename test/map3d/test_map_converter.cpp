// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map3d/map_converter.hpp"
#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <array>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

class ConverterTest : public ::testing::Test {
protected:
  std::filesystem::path directory;
  void SetUp() override {
    char pattern[] = "/tmp/vq-converter-XXXXXX";
    const auto *created = mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory = created;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }

  template <typename T>
  pcl::PCLPointCloud2 cloud(const std::vector<std::array<T, 3>> &points) {
    pcl::PCLPointCloud2 result;
    result.width = static_cast<std::uint32_t>(points.size());
    result.height = 1;
    result.point_step = 3 * sizeof(T);
    result.row_step = result.width * result.point_step;
    result.is_dense = false;
    const std::uint16_t endian_probe = 1;
    result.is_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
    for (std::size_t a = 0; a < 3; ++a) {
      pcl::PCLPointField field;
      field.name = std::string(1, "xyz"[a]);
      field.offset = a * sizeof(T);
      field.count = 1;
      field.datatype = sizeof(T) == 4 ? pcl::PCLPointField::FLOAT32
                                     : pcl::PCLPointField::FLOAT64;
      result.fields.push_back(field);
    }
    result.data.resize(result.row_step);
    if (!points.empty()) std::memcpy(result.data.data(), points.data(), result.data.size());
    return result;
  }

  std::string save(const pcl::PCLPointCloud2 &points, int format) {
    const auto path = (directory / (std::to_string(format) +
                        (format < 3 ? ".pcd" : ".ply"))).string();
    int status = -1;
    if (format == 0) status = pcl::PCDWriter().writeASCII(path, points);
    if (format == 1) status = pcl::PCDWriter().writeBinary(path, points);
    if (format == 2) status = pcl::PCDWriter().writeBinaryCompressed(path, points);
    if (format == 3) status = pcl::PLYWriter().writeASCII(path, points);
    if (format == 4) status = pcl::PLYWriter().writeBinary(path, points);
    EXPECT_EQ(status, 0);
    return path;
  }

  template <typename T> void formats() {
    const auto points = cloud<T>({{{-1, 2, 3}}, {{0, 2, 3}}, {{1, 3, 4}},
                                 {{1, 3, 4}}});
    vq_map_server::map3d::ConversionOptions options;
    options.voxel_size = 1;
    options.block_size = 2;
    for (int format = 0; format < 5; ++format) {
      SCOPED_TRACE(format);
      const auto map = vq_map_server::map3d::convertPointCloud(save(points, format), options);
      EXPECT_EQ(map.metadata().origin, (std::array<float, 3>{-1, 2, 3}));
      EXPECT_EQ(map.metadata().dimensions, (std::array<std::uint32_t, 3>{3, 2, 2}));
      EXPECT_EQ(map.at({0, 0, 0}), vq_map_server::map3d::CellState::Occupied);
      EXPECT_EQ(map.at({1, 0, 0}), vq_map_server::map3d::CellState::Occupied);
      EXPECT_EQ(map.at({2, 1, 1}), vq_map_server::map3d::CellState::Occupied);
      EXPECT_EQ(map.at({1, 1, 1}), vq_map_server::map3d::CellState::Free);
      EXPECT_EQ(map.at({3, 1, 1}), vq_map_server::map3d::CellState::OutOfBounds);
      EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{3, 64}));
      EXPECT_EQ(map.indices(), (std::vector<std::uint32_t>{0, 1}));
    }
  }
};

TEST_F(ConverterTest, Float32AcrossPcdAndPlyEncodings) { formats<float>(); }
TEST_F(ConverterTest, Float64AcrossPcdAndPlyEncodings) { formats<double>(); }

TEST_F(ConverterTest, Float64CoordinatesAreNotNarrowedBeforeVoxelization) {
  // Float32 cannot distinguish these two coordinates, whereas double can.
  const auto input = save(cloud<double>({{{16777216.0, 0, 0}},
                                         {{16777217.0, 0, 0}}}), 2);
  vq_map_server::map3d::ConversionOptions options;
  options.voxel_size = 0.5F;
  options.block_size = 2;
  const auto map = vq_map_server::map3d::convertPointCloud(input, options);
  EXPECT_EQ(map.metadata().dimensions[0], 3u);
  EXPECT_EQ(map.at({0, 0, 0}), vq_map_server::map3d::CellState::Occupied);
  EXPECT_EQ(map.at({1, 0, 0}), vq_map_server::map3d::CellState::Free);
  EXPECT_EQ(map.at({2, 0, 0}), vq_map_server::map3d::CellState::Occupied);
}

TEST_F(ConverterTest, Float64MinimumRemainsInsideFloatOrigin) {
  const auto input = save(cloud<double>({{{0.1, -0.1, 0.3}},
                                         {{1.1, 0.9, 1.3}}}), 1);
  const auto map = vq_map_server::map3d::convertPointCloud(input);
  EXPECT_LE(static_cast<double>(map.metadata().origin[0]), 0.1);
  EXPECT_EQ(map.atWorld({0.1, -0.1, 0.3}), vq_map_server::map3d::CellState::Occupied);
  EXPECT_EQ(map.atWorld({1.1, 0.9, 1.3}), vq_map_server::map3d::CellState::Occupied);
}

TEST_F(ConverterTest, DiscardsIndividualNonfinitePoints) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto input = save(cloud<float>({{{nan, 0, 0}}, {{0, inf, 0}},
                                       {{0, 0, -inf}}, {{3, 4, 5}}}), 1);
  const auto map = vq_map_server::map3d::convertPointCloud(input);
  EXPECT_EQ(map.metadata().origin, (std::array<float, 3>{3, 4, 5}));
  EXPECT_EQ(map.metadata().dimensions, (std::array<std::uint32_t, 3>{1, 1, 1}));
}

TEST_F(ConverterTest, RejectsNonfiniteOnlyAndMissingAndMalformed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(save(cloud<double>({{{nan, 0, 0}}}), 1)),
               std::exception);
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud((directory / "absent.pcd").string()),
               std::exception);
  const auto path = directory / "malformed.pcd";
  std::ofstream(path) << "not a point cloud\n";
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(path.string()), std::exception);
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud("input.xyz"), std::exception);
  std::ofstream(path) << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                         "COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n";
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(path.string()), std::exception);
}

TEST_F(ConverterTest, RejectsAbsentNonscalarAndIntegerXyzFields) {
  auto points = cloud<float>({{{0, 0, 0}}});
  points.fields[2].name = "intensity";
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(save(points, 1)), std::exception);
  points.fields[2].name = "z";
  points.fields[0].datatype = pcl::PCLPointField::INT32;
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(save(points, 1)), std::exception);
  const auto path = directory / "nonscalar.pcd";
  std::ofstream(path) << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                         "COUNT 2 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\n"
                         "DATA ascii\n0 0 0 0\n";
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(path.string()), std::exception);
}

TEST_F(ConverterTest, FirstOccurrenceDictionaryAndZeroPaddedBlocks) {
  const auto input = save(cloud<float>({{{0, 0, 0}}, {{4, 0, 0}}, {{6, 0, 0}}}), 1);
  vq_map_server::map3d::ConversionOptions options;
  options.voxel_size = 1;
  options.block_size = 2;
  const auto map = vq_map_server::map3d::convertPointCloud(input, options);
  EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{1, 0}));
  EXPECT_EQ(map.indices(), (std::vector<std::uint32_t>{0, 1, 0, 0}));
  EXPECT_EQ(map.dictionary().size() + map.indices().size() * sizeof(std::uint32_t), 18u);
  options.max_map_bytes = 18;
  EXPECT_NO_THROW(vq_map_server::map3d::convertPointCloud(input, options));
  options.max_map_bytes = 17;
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
}

TEST_F(ConverterTest, OddBlockSizePaddingAndMaximumBlockSize) {
  const auto input = save(cloud<float>({{{0, 0, 0}}, {{2, 2, 2}}}), 1);
  vq_map_server::map3d::ConversionOptions options;
  options.voxel_size = 1;
  options.block_size = 3;
  auto map = vq_map_server::map3d::convertPointCloud(input, options);
  EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{1, 0, 0, 4}));
  options.block_size = 64;
  map = vq_map_server::map3d::convertPointCloud(input, options);
  EXPECT_EQ(map.patternBytes(), 32768u);
  EXPECT_EQ(map.at({2, 2, 2}), vq_map_server::map3d::CellState::Occupied);
  options.block_size = 1;
  map = vq_map_server::map3d::convertPointCloud(input, options);
  EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{1, 0}));
  EXPECT_EQ(map.indices().back(), 0u);
}

TEST_F(ConverterTest, RejectsInvalidOptionsAndExcessiveGeometryBeforeIndexAllocation) {
  const auto input = save(cloud<double>({{{0, 0, 0}}, {{1000000, 1000000, 0}}}), 1);
  vq_map_server::map3d::ConversionOptions options;
  options.voxel_size = 1;
  options.block_size = 1;
  options.max_map_bytes = 1024;
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
  for (auto block_size : {0u, 65u}) {
    options.block_size = block_size;
    EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
  }
  options.block_size = 13;
  for (auto size : {0.0F, -1.0F, std::numeric_limits<float>::infinity()}) {
    options.voxel_size = size;
    EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
  }
  options.voxel_size = std::numeric_limits<float>::min();
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
  options.voxel_size = 1;
  options.frame_id.clear();
  EXPECT_THROW(vq_map_server::map3d::convertPointCloud(input, options), std::exception);
}

}  // namespace
