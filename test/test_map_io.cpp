// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map_io.hpp"

#include <gtest/gtest.h>
#include <hdf5.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
using vq_map_server::CellState;
using vq_map_server::MapMetadata;
using vq_map_server::VqMap;
using vq_map_server::readHdf5;
using vq_map_server::writeHdf5;

class MapIoTest : public ::testing::Test {
protected:
  void SetUp() override {
    auto pattern = (std::filesystem::temp_directory_path() / "vq-map-io-XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr)
      throw std::runtime_error("mkdtemp failed");
    directory = pattern;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::string path() const { return (directory / "map.h5").string(); }
  std::filesystem::path directory;
};

template <typename T>
void dataset(hid_t file, const char *path, hid_t type, const std::vector<T> &values) {
  const hsize_t count = values.size();
  const hid_t space = H5Screate_simple(1, &count, nullptr);
  const hid_t ds = H5Dcreate2(file, path, type, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  if (ds < 0 || H5Dwrite(ds, type, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data()) < 0)
    throw std::runtime_error("fixture dataset write failed");
  H5Dclose(ds);
  H5Sclose(space);
}

void attribute(hid_t group, const char *name, const std::string &value) {
  const hid_t type = H5Tcopy(H5T_C_S1);
  H5Tset_size(type, value.size() + 1);
  const hid_t space = H5Screate(H5S_SCALAR);
  const hid_t attr = H5Acreate2(group, name, type, space, H5P_DEFAULT, H5P_DEFAULT);
  if (attr < 0 || H5Awrite(attr, type, value.c_str()) < 0)
    throw std::runtime_error("fixture attribute write failed");
  H5Aclose(attr);
  H5Sclose(space);
  H5Tclose(type);
}

// An independent schema fixture, not produced by writeHdf5.
// Five 2x2x2 blocks have indices 0,1,4,3,2; LSB-first 3-bit packing is 0x08,0x27.
void upstreamFixture(const std::string &path) {
  const hid_t file = H5Fcreate(path.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT);
  for (const char *name : {"/metadata", "/compression_params", "/dictionary", "/compressed_data"}) {
    const auto group = H5Gcreate2(file, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Gclose(group);
  }
  const auto group = H5Gopen2(file, "/metadata", H5P_DEFAULT);
  attribute(group, "version", "1.0.0");
  attribute(group, "frame_id", "survey");
  attribute(group, "compression_method", "voxel_dictionary");
  H5Gclose(group);
  dataset<float>(file, "/compression_params/voxel_size", H5T_NATIVE_FLOAT, {0.5F});
  dataset<float>(file, "/compression_params/grid_origin", H5T_NATIVE_FLOAT, {-1, -2, -3});
  dataset<std::int32_t>(file, "/compression_params/grid_dimensions", H5T_NATIVE_INT32, {9, 2, 2});
  dataset<std::int32_t>(file, "/compressed_data/block_dims", H5T_NATIVE_INT32, {5, 1, 1});
  dataset<std::uint32_t>(file, "/compression_params/block_size", H5T_NATIVE_UINT32, {2});
  dataset<std::uint32_t>(file, "/compression_params/pattern_bits", H5T_NATIVE_UINT32, {8});
  dataset<std::uint32_t>(file, "/compression_params/dictionary_size", H5T_NATIVE_UINT32, {5});
  dataset<std::uint32_t>(file, "/compression_params/block_index_bit_width", H5T_NATIVE_UINT32, {3});
  dataset<std::uint32_t>(file, "/dictionary/pattern_length", H5T_NATIVE_UINT32, {1});
  dataset<std::uint8_t>(file, "/dictionary/patterns", H5T_NATIVE_UINT8, {0, 1, 128, 6, 12});
  dataset<std::uint8_t>(file, "/compressed_data/block_indices", H5T_NATIVE_UINT8, {0x08, 0x27});
  H5Fclose(file);
}

TEST_F(MapIoTest, ReadsIndependentlyPackedUpstreamSchema) {
  upstreamFixture(path());

  const auto map = readHdf5(path());

  EXPECT_EQ(map.indices(), (std::vector<std::uint32_t>{0, 1, 4, 3, 2}));
  EXPECT_EQ(map.metadata().frame_id, "survey");
  EXPECT_EQ(map.at({2, 0, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({5, 1, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({7, 0, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({4, 0, 0}), CellState::Free);
  EXPECT_EQ(map.at({9, 0, 0}), CellState::OutOfBounds);
}

TEST_F(MapIoTest, WritesByteAlignedDictionaryAndBitPackedIndexDataset) {
  MapMetadata metadata;
  metadata.dimensions = {9, 2, 2};
  metadata.block_size = 2;
  const VqMap map(metadata, {0, 1, 128, 6, 12}, {0, 1, 4, 3, 2});

  writeHdf5(path(), map);

  const hid_t file = H5Fopen(path().c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  const hid_t ds = H5Dopen2(file, "/compressed_data/block_indices", H5P_DEFAULT);
  std::array<std::uint8_t, 2> packed{};
  ASSERT_GE(H5Dread(ds, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, packed.data()), 0);
  H5Dclose(ds);
  H5Fclose(file);
  EXPECT_EQ(packed, (std::array<std::uint8_t, 2>{0x08, 0x27}));
  const auto loaded = readHdf5(path());
  EXPECT_EQ(loaded.dictionary(), map.dictionary());
  EXPECT_EQ(loaded.indices(), map.indices());
  EXPECT_EQ(loaded.metadata().dimensions, metadata.dimensions);
}

TEST_F(MapIoTest, PreservesExistingOutputWhenExclusiveCreateFails) {
  std::ofstream(path()) << "existing map must survive";
  MapMetadata metadata;
  metadata.dimensions = {1, 1, 1};
  metadata.block_size = 1;
  const VqMap map(metadata, {1}, {0});

  EXPECT_THROW(writeHdf5(path(), map), std::runtime_error);

  std::ifstream stream(path());
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "existing map must survive");
}

TEST_F(MapIoTest, RejectsOversizedPayloadBeforeAllocation) {
  upstreamFixture(path());

  EXPECT_THROW(readHdf5(path(), 24), std::runtime_error);
  EXPECT_NO_THROW(readHdf5(path(), 25));
}

TEST_F(MapIoTest, RejectsInvalidShapesWidthsIndicesAndVersion) {
  struct Mutation { const char *path; std::uint32_t value; };
  const Mutation mutations[] = {
      {"/compression_params/block_index_bit_width", 0},
      {"/compression_params/block_index_bit_width", 33},
      {"/compression_params/dictionary_size", 2},
      {"/compression_params/pattern_bits", 7},
      {"/dictionary/pattern_length", 2}};
  for (const auto &mutation : mutations) {
    upstreamFixture(path());
    const auto file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    const auto ds = H5Dopen2(file, mutation.path, H5P_DEFAULT);
    H5Dwrite(ds, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &mutation.value);
    H5Dclose(ds);
    H5Fclose(file);
    EXPECT_THROW(readHdf5(path()), std::exception) << mutation.path << "=" << mutation.value;
    std::filesystem::remove(path());
  }
  upstreamFixture(path());
  auto file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  H5Ldelete(file, "/compression_params/grid_origin", H5P_DEFAULT);
  dataset<float>(file, "/compression_params/grid_origin", H5T_NATIVE_FLOAT, {0, 0});
  H5Fclose(file);
  EXPECT_THROW(readHdf5(path()), std::runtime_error);
  std::filesystem::remove(path());
  upstreamFixture(path());
  file = H5Fopen(path().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  const auto group = H5Gopen2(file, "/metadata", H5P_DEFAULT);
  H5Adelete(group, "version");
  attribute(group, "version", "2.0.0");
  H5Gclose(group);
  H5Fclose(file);
  EXPECT_THROW(readHdf5(path()), std::runtime_error);
}

}  // namespace
