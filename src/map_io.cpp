// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
// Modified from vq_occupancy_compressor's HDF5IO.cpp packing convention:
// checked shapes/types, RAII, exclusive creation, and an immutable map API.
#include "vq_map_server/map_io.hpp"

#include <hdf5.h>
#include <algorithm>
#include <array>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace vq_map_server {
namespace {

class Handle {
public:
  Handle(hid_t id, herr_t (*close)(hid_t), const std::string &operation)
      : id_(id), close_(close) {
    if (id < 0)
      throw std::runtime_error("HDF5: " + operation);
  }
  ~Handle() { close_(id_); }
  Handle(const Handle &) = delete;
  Handle &operator=(const Handle &) = delete;
  hid_t get() const noexcept { return id_; }
private:
  hid_t id_;
  herr_t (*close_)(hid_t);
};

void check(herr_t status, const std::string &operation) {
  if (status < 0)
    throw std::runtime_error("HDF5: " + operation);
}

template <typename T> hid_t nativeType();
template <> hid_t nativeType<std::uint8_t>() { return H5T_NATIVE_UINT8; }
template <> hid_t nativeType<std::uint32_t>() { return H5T_NATIVE_UINT32; }
template <> hid_t nativeType<std::int32_t>() { return H5T_NATIVE_INT32; }
template <> hid_t nativeType<float>() { return H5T_NATIVE_FLOAT; }

template <typename T>
std::vector<T> readData(hid_t file, const char *path, std::size_t count) {
  Handle dataset(H5Dopen2(file, path, H5P_DEFAULT), H5Dclose, "open " + std::string(path));
  Handle space(H5Dget_space(dataset.get()), H5Sclose, "get dataspace");
  hsize_t dimension = 0;
  if (H5Sget_simple_extent_ndims(space.get()) != 1 ||
      H5Sget_simple_extent_dims(space.get(), &dimension, nullptr) < 0 ||
      dimension != count)
    throw std::runtime_error("HDF5: invalid shape of " + std::string(path));
  Handle type(H5Dget_type(dataset.get()), H5Tclose, "get datatype");
  constexpr bool floating = std::is_floating_point_v<T>;
  if (H5Tget_class(type.get()) != (floating ? H5T_FLOAT : H5T_INTEGER) ||
      H5Tget_size(type.get()) != sizeof(T))
    throw std::runtime_error("HDF5: invalid datatype of " + std::string(path));
  if constexpr (!floating) {
    if (H5Tget_sign(type.get()) != (std::is_signed_v<T> ? H5T_SGN_2 : H5T_SGN_NONE))
      throw std::runtime_error("HDF5: invalid integer sign of " + std::string(path));
  }
  std::vector<T> result(count);
  if (count != 0)
    check(H5Dread(dataset.get(), nativeType<T>(), H5S_ALL, H5S_ALL, H5P_DEFAULT,
                  result.data()), "read " + std::string(path));
  return result;
}

template <typename T>
T readScalar(hid_t file, const char *path) {
  return readData<T>(file, path, 1).front();
}

template <typename T>
void writeData(hid_t file, const char *path, const T *values, std::size_t count) {
  hsize_t dimension = count;
  Handle space(H5Screate_simple(1, &dimension, nullptr), H5Sclose, "create dataspace");
  Handle dataset(H5Dcreate2(file, path, nativeType<T>(), space.get(), H5P_DEFAULT,
                           H5P_DEFAULT, H5P_DEFAULT), H5Dclose,
                 "create " + std::string(path));
  check(H5Dwrite(dataset.get(), nativeType<T>(), H5S_ALL, H5S_ALL, H5P_DEFAULT, values),
        "write " + std::string(path));
}

template <typename T>
void writeScalar(hid_t file, const char *path, T value) {
  writeData(file, path, &value, 1);
}

std::string readString(hid_t group, const char *name) {
  Handle attribute(H5Aopen(group, name, H5P_DEFAULT), H5Aclose,
                   "open attribute " + std::string(name));
  Handle type(H5Aget_type(attribute.get()), H5Tclose, "get attribute type");
  Handle space(H5Aget_space(attribute.get()), H5Sclose, "get attribute space");
  const auto size = H5Tget_size(type.get());
  if (H5Tget_class(type.get()) != H5T_STRING || H5Tis_variable_str(type.get()) != 0 ||
      H5Sget_simple_extent_type(space.get()) != H5S_SCALAR || size == 0 || size > 4096)
    throw std::runtime_error("HDF5: invalid string attribute " + std::string(name));
  std::string value(size, '\0');
  check(H5Aread(attribute.get(), type.get(), value.data()), "read attribute");
  const auto terminator = value.find('\0');
  if (terminator != std::string::npos)
    value.resize(terminator);
  return value;
}

void writeString(hid_t group, const char *name, const std::string &value) {
  Handle type(H5Tcopy(H5T_C_S1), H5Tclose, "create string type");
  check(H5Tset_size(type.get(), value.size() + 1), "set string size");
  check(H5Tset_strpad(type.get(), H5T_STR_NULLTERM), "set string padding");
  Handle space(H5Screate(H5S_SCALAR), H5Sclose, "create attribute space");
  Handle attribute(H5Acreate2(group, name, type.get(), space.get(), H5P_DEFAULT,
                             H5P_DEFAULT), H5Aclose, "create attribute");
  check(H5Awrite(attribute.get(), type.get(), value.c_str()), "write attribute");
}

std::size_t checkedMultiply(std::size_t a, std::size_t b) {
  if (b != 0 && a > std::numeric_limits<std::size_t>::max() / b)
    throw std::runtime_error("HDF5: dimensions overflow size_t");
  return a * b;
}

}  // namespace

VqMap readHdf5(const std::string &path, std::size_t max_map_bytes) {
  if (max_map_bytes == 0)
    throw std::invalid_argument("max_map_bytes must be positive");
  Handle file(H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose,
              "open input " + path);
  Handle metadata_group(H5Gopen2(file.get(), "/metadata", H5P_DEFAULT), H5Gclose,
                        "open metadata");
  if (readString(metadata_group.get(), "version") != "1.0.0" ||
      readString(metadata_group.get(), "compression_method") != "voxel_dictionary")
    throw std::runtime_error("HDF5: unsupported map version or compression method");
  MapMetadata metadata;
  metadata.frame_id = readString(metadata_group.get(), "frame_id");
  metadata.voxel_size = readScalar<float>(file.get(), "/compression_params/voxel_size");
  metadata.block_size = readScalar<std::uint32_t>(file.get(), "/compression_params/block_size");
  if (metadata.block_size == 0 || metadata.block_size > 64)
    throw std::runtime_error("HDF5: block_size must be between 1 and 64");
  const auto origin = readData<float>(file.get(), "/compression_params/grid_origin", 3);
  std::copy(origin.begin(), origin.end(), metadata.origin.begin());
  const auto dims = readData<std::int32_t>(file.get(), "/compression_params/grid_dimensions", 3);
  const auto block_dims = readData<std::int32_t>(file.get(), "/compressed_data/block_dims", 3);
  std::size_t block_count = 1;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (dims[axis] <= 0 || block_dims[axis] <= 0)
      throw std::runtime_error("HDF5: dimensions must be positive");
    metadata.dimensions[axis] = static_cast<std::uint32_t>(dims[axis]);
    const auto expected = (static_cast<std::uint64_t>(dims[axis]) +
                          metadata.block_size - 1) / metadata.block_size;
    if (static_cast<std::uint64_t>(block_dims[axis]) != expected)
      throw std::runtime_error("HDF5: block dimensions do not match grid dimensions");
    block_count = checkedMultiply(block_count, static_cast<std::size_t>(block_dims[axis]));
  }
  const auto side = static_cast<std::size_t>(metadata.block_size);
  const auto pattern_bits = side * side * side;
  const auto stride = (pattern_bits + 7) / 8;
  if (readScalar<std::uint32_t>(file.get(), "/compression_params/pattern_bits") != pattern_bits ||
      readScalar<std::uint32_t>(file.get(), "/dictionary/pattern_length") != stride)
    throw std::runtime_error("HDF5: pattern dimensions do not match block_size");
  const auto dictionary_size = readScalar<std::uint32_t>(
      file.get(), "/compression_params/dictionary_size");
  if (dictionary_size == 0)
    throw std::runtime_error("HDF5: empty dictionary");
  const auto dictionary_bytes = checkedMultiply(dictionary_size, stride);
  const auto index_bytes = checkedMultiply(block_count, sizeof(std::uint32_t));
  if (dictionary_bytes > max_map_bytes || index_bytes > max_map_bytes - dictionary_bytes)
    throw std::runtime_error("HDF5: map payload exceeds max_map_bytes");
  const auto width = readScalar<std::uint32_t>(
      file.get(), "/compression_params/block_index_bit_width");
  if (width == 0 || width > 32)
    throw std::runtime_error("HDF5: index bit width must be between 1 and 32");
  const auto packed_bits = checkedMultiply(block_count, width);
  const auto packed_bytes = packed_bits / 8 + (packed_bits % 8 != 0 ? 1 : 0);
  auto dictionary = readData<std::uint8_t>(file.get(), "/dictionary/patterns", dictionary_bytes);
  const auto packed = readData<std::uint8_t>(
      file.get(), "/compressed_data/block_indices", packed_bytes);
  std::vector<std::uint32_t> indices(block_count, 0);
  for (std::size_t i = 0; i < block_count; ++i)
    for (std::size_t bit = 0; bit < width; ++bit) {
      const auto absolute_bit = i * width + bit;
      indices[i] |= static_cast<std::uint32_t>(
          (packed[absolute_bit / 8] >> (absolute_bit % 8)) & 1U) << bit;
    }
  return VqMap(std::move(metadata), std::move(dictionary), std::move(indices));
}

void writeHdf5(const std::string &path, const VqMap &map) {
  // Check creation separately: failed exclusive creation must never remove an existing file.
  Handle file(H5Fcreate(path.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT), H5Fclose,
              "exclusively create output " + path);
  try {
    for (const auto *name : {"/metadata", "/compression_params", "/dictionary",
                             "/compressed_data"}) {
      Handle group(H5Gcreate2(file.get(), name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT),
                   H5Gclose, "create group");
    }
    const auto &metadata = map.metadata();
    Handle group(H5Gopen2(file.get(), "/metadata", H5P_DEFAULT), H5Gclose, "open metadata");
    writeString(group.get(), "version", "1.0.0");
    writeString(group.get(), "creation_time", "");
    writeString(group.get(), "frame_id", metadata.frame_id);
    writeString(group.get(), "compression_method", "voxel_dictionary");
    writeScalar(file.get(), "/compression_params/voxel_size", metadata.voxel_size);
    writeScalar(file.get(), "/compression_params/block_size", metadata.block_size);
    const auto side = metadata.block_size;
    writeScalar(file.get(), "/compression_params/pattern_bits", side * side * side);
    writeScalar(file.get(), "/compression_params/dictionary_size",
                static_cast<std::uint32_t>(map.dictionarySize()));
    const auto max_index = *std::max_element(map.indices().begin(), map.indices().end());
    std::uint32_t width = 1;
    while (width < 32 && (static_cast<std::uint64_t>(max_index) >> width) != 0)
      ++width;
    writeScalar(file.get(), "/compression_params/block_index_bit_width", width);
    writeData(file.get(), "/compression_params/grid_origin", metadata.origin.data(), 3);
    std::array<std::int32_t, 3> dimensions{}, block_dimensions{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      dimensions[axis] = static_cast<std::int32_t>(metadata.dimensions[axis]);
      block_dimensions[axis] = static_cast<std::int32_t>(map.blockDimensions()[axis]);
    }
    writeData(file.get(), "/compression_params/grid_dimensions", dimensions.data(), 3);
    writeData(file.get(), "/compressed_data/block_dims", block_dimensions.data(), 3);
    writeScalar(file.get(), "/dictionary/pattern_length",
                static_cast<std::uint32_t>(map.patternBytes()));
    writeData(file.get(), "/dictionary/patterns",
              map.dictionary().data(), map.dictionary().size());
    const auto bit_count = checkedMultiply(map.indices().size(), width);
    std::vector<std::uint8_t> packed(bit_count / 8 + (bit_count % 8 != 0 ? 1 : 0), 0);
    for (std::size_t i = 0; i < map.indices().size(); ++i)
      for (std::size_t bit = 0; bit < width; ++bit) {
        const auto absolute_bit = i * width + bit;
        if (((map.indices()[i] >> bit) & 1U) != 0)
          packed[absolute_bit / 8] |= static_cast<std::uint8_t>(1U << (absolute_bit % 8));
      }
    writeData(file.get(), "/compressed_data/block_indices", packed.data(), packed.size());
    check(H5Fflush(file.get(), H5F_SCOPE_LOCAL), "flush output");
  } catch (...) {
    // Only this newly created, incomplete output belongs to us.
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    throw;
  }
}

}  // namespace vq_map_server
