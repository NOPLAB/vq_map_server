// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_io.hpp"
#include "../common/hdf5.hpp"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vq_map_server::map2d {
namespace {
using namespace common::hdf5;
}  // namespace

VqMap readHdf5(const std::string &path, std::size_t max_map_bytes) {
  if (!max_map_bytes) throw std::invalid_argument("max_map_bytes must be positive");
  Handle file(H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose, "open input " + path);
  Handle group(H5Gopen2(file.get(), "/metadata", H5P_DEFAULT), H5Gclose, "open metadata");
  if (readString(group.get(), "version") != "1.0.0" ||
      readString(group.get(), "compression_method") != "grid_dictionary_2d" ||
      readScalar<std::uint32_t>(file.get(), "/compression_params/map_dimension") != 2)
    throw std::runtime_error("HDF5: unsupported 2D map version, method, or dimension");
  MapMetadata metadata;
  metadata.frame_id = readString(group.get(), "frame_id");
  metadata.resolution = readScalar<float>(file.get(), "/compression_params/resolution");
  metadata.yaw = readScalar<double>(file.get(), "/compression_params/grid_yaw");
  metadata.block_size = readScalar<std::uint32_t>(file.get(), "/compression_params/block_size");
  const auto encoding = readScalar<std::uint8_t>(file.get(), "/compression_params/cell_encoding");
  if ((encoding != 1 && encoding != 2) || metadata.block_size < 1 || metadata.block_size > 64)
    throw std::runtime_error("HDF5: invalid cell encoding or block_size");
  metadata.encoding = static_cast<CellEncoding>(encoding);
  const auto origin = readData<double>(file.get(), "/compression_params/grid_origin", 2);
  std::copy(origin.begin(), origin.end(), metadata.origin.begin());
  const auto dimensions = readData<std::uint32_t>(file.get(), "/compression_params/grid_dimensions", 2);
  std::copy(dimensions.begin(), dimensions.end(), metadata.dimensions.begin());
  const auto blocks = readData<std::uint32_t>(file.get(), "/compressed_data/block_dims", 2);
  std::size_t block_count = 1;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    if (!dimensions[axis] || dimensions[axis] > std::numeric_limits<std::int32_t>::max() ||
        blocks[axis] != (static_cast<std::uint64_t>(dimensions[axis]) +
                         metadata.block_size - 1) / metadata.block_size)
      throw std::runtime_error("HDF5: invalid grid or block dimensions");
    block_count = checkedMultiply(block_count, blocks[axis]);
  }
  const auto width = readScalar<std::uint8_t>(file.get(), "/compression_params/block_index_bit_width");
  if (width != 8 && width != 16 && width != 32)
    throw std::runtime_error("HDF5: index bit width must be 8, 16, or 32");
  const auto pattern_bits = metadata.block_size * metadata.block_size * encoding;
  const auto stride = (pattern_bits + 7) / 8;
  if (readScalar<std::uint32_t>(file.get(), "/compression_params/pattern_bits") != pattern_bits ||
      readScalar<std::uint32_t>(file.get(), "/dictionary/pattern_length") != stride)
    throw std::runtime_error("HDF5: pattern dimensions do not match encoding and block_size");
  const auto count = readScalar<std::uint32_t>(file.get(), "/compression_params/dictionary_size");
  if (!count || static_cast<std::uint64_t>(count) > (std::uint64_t{1} << width))
    throw std::runtime_error("HDF5: invalid dictionary size");
  const auto dictionary_bytes = checkedMultiply(count, stride);
  const auto index_bytes = checkedMultiply(block_count, width / 8);
  if (dictionary_bytes > max_map_bytes || index_bytes > max_map_bytes - dictionary_bytes)
    throw std::runtime_error("HDF5: map payload exceeds max_map_bytes");
  auto dictionary = readData<std::uint8_t>(file.get(), "/dictionary/patterns", dictionary_bytes);
  auto indices = readData<std::uint8_t>(file.get(), "/compressed_data/block_indices", index_bytes);
  return VqMap(std::move(metadata), std::move(dictionary), std::move(indices), width);
}

void writeHdf5(const std::string &path, const VqMap &map) {
  Handle file(H5Fcreate(path.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT), H5Fclose,
              "exclusively create output " + path);
  try {
    for (const auto *name : {"/metadata", "/compression_params", "/dictionary", "/compressed_data"}) {
      Handle group(H5Gcreate2(file.get(), name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT),
                   H5Gclose, "create group");
    }
    Handle group(H5Gopen2(file.get(), "/metadata", H5P_DEFAULT), H5Gclose, "open metadata");
    const auto &m = map.metadata();
    writeString(group.get(), "version", "1.0.0");
    writeString(group.get(), "compression_method", "grid_dictionary_2d");
    writeString(group.get(), "frame_id", m.frame_id);
    writeScalar(file.get(), "/compression_params/map_dimension", std::uint32_t{2});
    writeScalar(file.get(), "/compression_params/resolution", m.resolution);
    writeScalar(file.get(), "/compression_params/block_size", m.block_size);
    writeScalar(file.get(), "/compression_params/cell_encoding", static_cast<std::uint8_t>(m.encoding));
    writeScalar(file.get(), "/compression_params/grid_yaw", m.yaw);
    writeData(file.get(), "/compression_params/grid_origin", m.origin.data(), 2);
    writeData(file.get(), "/compression_params/grid_dimensions", m.dimensions.data(), 2);
    writeScalar(file.get(), "/compression_params/block_index_bit_width", map.indexBitWidth());
    writeScalar(file.get(), "/compression_params/pattern_bits",
                m.block_size * m.block_size * static_cast<std::uint8_t>(m.encoding));
    writeScalar(file.get(), "/compression_params/dictionary_size",
                static_cast<std::uint32_t>(map.dictionarySize()));
    writeData(file.get(), "/compressed_data/block_dims", map.blockDimensions().data(), 2);
    writeScalar(file.get(), "/dictionary/pattern_length",
                static_cast<std::uint32_t>(map.patternBytes()));
    writeData(file.get(), "/dictionary/patterns", map.dictionary().data(), map.dictionary().size());
    writeData(file.get(), "/compressed_data/block_indices", map.indices().data(), map.indices().size());
    check(H5Fflush(file.get(), H5F_SCOPE_LOCAL), "flush output");
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    throw;
  }
}

}  // namespace vq_map_server::map2d
