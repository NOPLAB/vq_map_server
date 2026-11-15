// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_io.hpp"
#include "vq_map_server/map3d/map_io.hpp"

#include <gtest/gtest.h>
#include <hdf5.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <unistd.h>

namespace {
using namespace vq_map_server::map2d;

class MapIo2DTest : public ::testing::Test {
protected:
  void SetUp() override {
    char pattern[] = "/tmp/vq-io-2d-XXXXXX";
    const auto *created = mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory = created;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::string path() const { return (directory / "map.h5").string(); }
  std::filesystem::path directory;
};

template <typename T>
void dataset(hid_t file, const char *path, hid_t type, const std::vector<T> &values) {
  const hsize_t count = values.size();
  const auto space = H5Screate_simple(1, &count, nullptr);
  const auto ds = H5Dcreate2(file, path, type, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  if (ds < 0 || H5Dwrite(ds, type, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data()) < 0)
    throw std::runtime_error("fixture dataset write failed");
  H5Dclose(ds);
  H5Sclose(space);
}

void attribute(hid_t group, const char *name, const std::string &value) {
  const auto type = H5Tcopy(H5T_C_S1);
  H5Tset_size(type, value.size() + 1);
  const auto space = H5Screate(H5S_SCALAR);
  const auto attr = H5Acreate2(group, name, type, space, H5P_DEFAULT, H5P_DEFAULT);
  if (attr < 0 || H5Awrite(attr, type, value.c_str()) < 0)
    throw std::runtime_error("fixture attribute write failed");
  H5Aclose(attr);
  H5Sclose(space);
  H5Tclose(type);
}

// Independent v1 fixture: 5x2 cells in three 2x2 blocks, IDs 0,2,1.
// Pattern0 is [free,occupied;unknown,free], packed as 0x24.
void fixture(const std::string &path) {
  const auto file = H5Fcreate(path.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT);
  for (const auto *name : {"/metadata", "/compression_params", "/dictionary", "/compressed_data"}) {
    const auto group = H5Gcreate2(file, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Gclose(group);
  }
  const auto group = H5Gopen2(file, "/metadata", H5P_DEFAULT);
  attribute(group, "version", "1.0.0");
  attribute(group, "frame_id", "survey");
  attribute(group, "compression_method", "grid_dictionary_2d");
  H5Gclose(group);
  dataset<std::uint32_t>(file, "/compression_params/map_dimension", H5T_NATIVE_UINT32, {2});
  dataset<float>(file, "/compression_params/resolution", H5T_NATIVE_FLOAT, {0.25F});
  dataset<double>(file, "/compression_params/grid_origin", H5T_NATIVE_DOUBLE, {-2.125, 3.0625});
  dataset<double>(file, "/compression_params/grid_yaw", H5T_NATIVE_DOUBLE, {0.7});
  dataset<std::uint32_t>(file, "/compression_params/grid_dimensions", H5T_NATIVE_UINT32, {5, 2});
  dataset<std::uint32_t>(file, "/compressed_data/block_dims", H5T_NATIVE_UINT32, {3, 1});
  dataset<std::uint32_t>(file, "/compression_params/block_size", H5T_NATIVE_UINT32, {2});
  dataset<std::uint8_t>(file, "/compression_params/cell_encoding", H5T_NATIVE_UINT8, {2});
  dataset<std::uint32_t>(file, "/compression_params/pattern_bits", H5T_NATIVE_UINT32, {8});
  dataset<std::uint32_t>(file, "/compression_params/dictionary_size", H5T_NATIVE_UINT32, {3});
  dataset<std::uint8_t>(file, "/compression_params/block_index_bit_width", H5T_NATIVE_UINT8, {8});
  dataset<std::uint32_t>(file, "/dictionary/pattern_length", H5T_NATIVE_UINT32, {1});
  dataset<std::uint8_t>(file, "/dictionary/patterns", H5T_NATIVE_UINT8, {0x24, 0x21, 0x89});
  dataset<std::uint8_t>(file, "/compressed_data/block_indices", H5T_NATIVE_UINT8, {0, 2, 1});
  H5Fclose(file);
}

TEST_F(MapIo2DTest, ReadsIndependentTernarySchemaAndExactMetadata) {
  fixture(path());

  const auto map = readHdf5(path(), 6);

  EXPECT_EQ(map.metadata().frame_id, "survey");
  EXPECT_EQ(map.metadata().origin, (std::array<double, 2>{-2.125, 3.0625}));
  EXPECT_DOUBLE_EQ(map.metadata().yaw, 0.7);
  EXPECT_EQ(map.metadata().resolution, 0.25F);
  EXPECT_EQ(map.at({0, 0}), CellState::Free);
  EXPECT_EQ(map.at({1, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({0, 1}), CellState::Unknown);
  EXPECT_EQ(map.at({2, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({3, 0}), CellState::Unknown);
  EXPECT_EQ(map.at({4, 1}), CellState::Unknown);
  EXPECT_EQ(map.at({5, 0}), CellState::OutOfBounds);
  EXPECT_THROW(readHdf5(path(), 5), std::runtime_error);
  EXPECT_THROW(readHdf5(path(), 0), std::invalid_argument);
}

TEST_F(MapIo2DTest, WritesLittleEndianRecordsAndPreservesWideIndices) {
  MapMetadata metadata;
  metadata.dimensions = {5, 2};
  metadata.block_size = 2;
  metadata.origin = {-1.123456789, 2.987654321};
  metadata.yaw = -0.3;
  const VqMap map(metadata, {0x24, 0x21, 0x89}, {0, 0, 2, 0, 1, 0}, 16);

  writeHdf5(path(), map);

  const auto file = H5Fopen(path().c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  const auto ds = H5Dopen2(file, "/compressed_data/block_indices", H5P_DEFAULT);
  std::array<std::uint8_t, 6> bytes{};
  ASSERT_GE(H5Dread(ds, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, bytes.data()), 0);
  H5Dclose(ds);
  H5Fclose(file);
  EXPECT_EQ(bytes, (std::array<std::uint8_t, 6>{0, 0, 2, 0, 1, 0}));
  const auto loaded = readHdf5(path());
  EXPECT_EQ(loaded.indexBitWidth(), 16);
  EXPECT_EQ(loaded.dictionary(), map.dictionary());
  EXPECT_EQ(loaded.metadata().origin, metadata.origin);
  EXPECT_EQ(loaded.metadata().yaw, metadata.yaw);
}

TEST_F(MapIo2DTest, RejectsWrongDimensionsWidthsShapesTypesAndInvalidCells) {
  struct Mutation { const char *path; std::uint32_t value; };
  const Mutation mutations[] = {
      {"/compression_params/map_dimension", 3},
      {"/compression_params/block_index_bit_width", 7},
      {"/compression_params/cell_encoding", 3},
      {"/compression_params/block_size", 0},
      {"/compression_params/pattern_bits", 7},
      {"/compression_params/dictionary_size", 0},
      {"/dictionary/pattern_length", 2}};
  for (const auto &mutation : mutations) {
    fixture(path());
    const auto file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    const auto ds = H5Dopen2(file, mutation.path, H5P_DEFAULT);
    ASSERT_GE(H5Dwrite(ds, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &mutation.value), 0);
    H5Dclose(ds);
    H5Fclose(file);
    EXPECT_THROW(readHdf5(path()), std::exception) << mutation.path;
    std::filesystem::remove(path());
  }
  for (const bool wrong_type : {false, true}) {
    fixture(path());
    const auto file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    H5Ldelete(file, "/compression_params/grid_origin", H5P_DEFAULT);
    if (wrong_type)
      dataset<float>(file, "/compression_params/grid_origin", H5T_NATIVE_FLOAT, {0, 0});
    else
      dataset<double>(file, "/compression_params/grid_origin", H5T_NATIVE_DOUBLE, {0, 0, 0});
    H5Fclose(file);
    EXPECT_THROW(readHdf5(path()), std::runtime_error);
    std::filesystem::remove(path());
  }
  fixture(path());
  const auto file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  const auto ds = H5Dopen2(file, "/dictionary/patterns", H5P_DEFAULT);
  const std::array<std::uint8_t, 3> invalid{3, 0x21, 0x89};
  H5Dwrite(ds, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, invalid.data());
  H5Dclose(ds);
  H5Fclose(file);
  EXPECT_THROW(readHdf5(path()), std::invalid_argument);
}

TEST_F(MapIo2DTest, RefusesCrossDimensionalFilesAndExistingOutput) {
  fixture(path());
  EXPECT_THROW(vq_map_server::map3d::readHdf5(path()), std::runtime_error);
  const auto other = (directory / "3d.h5").string();
  vq_map_server::map3d::MapMetadata metadata3d;
  metadata3d.dimensions = {1, 1, 1};
  metadata3d.block_size = 1;
  vq_map_server::map3d::writeHdf5(other, vq_map_server::map3d::VqMap(metadata3d, {1}, {0}));
  EXPECT_THROW(readHdf5(other), std::runtime_error);
  const auto existing = (directory / "existing.h5").string();
  std::ofstream(existing) << "keep existing";
  MapMetadata metadata;
  metadata.dimensions = {1, 1};
  metadata.block_size = 1;

  EXPECT_THROW(writeHdf5(existing, VqMap(metadata, {1}, {0}, 8)), std::runtime_error);

  std::ifstream stream(existing);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(stream), {}), "keep existing");
}

}  // namespace
