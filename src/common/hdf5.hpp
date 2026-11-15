// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
// Adapted from vq_occupancy_compressor HDF5IO.cpp at
// 6157adcde9bd94b796f67dfaf1ffbd67492e3827.
// Modified: shared checked I/O, RAII, fixed types/shapes and double metadata.
#pragma once

#include <hdf5.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace vq_map_server::common::hdf5 {

class Handle {
public:
  Handle(hid_t id, herr_t (*close)(hid_t), const std::string &operation)
      : id_(id), close_(close) {
    if (id < 0) throw std::runtime_error("HDF5: " + operation);
  }
  ~Handle() { close_(id_); }
  Handle(const Handle &) = delete;
  Handle &operator=(const Handle &) = delete;
  hid_t get() const noexcept { return id_; }
private:
  hid_t id_;
  herr_t (*close_)(hid_t);
};

inline void check(herr_t status, const std::string &operation) {
  if (status < 0) throw std::runtime_error("HDF5: " + operation);
}

template <typename T> hid_t nativeType();
template <> inline hid_t nativeType<std::uint8_t>() { return H5T_NATIVE_UINT8; }
template <> inline hid_t nativeType<std::uint32_t>() { return H5T_NATIVE_UINT32; }
template <> inline hid_t nativeType<std::int32_t>() { return H5T_NATIVE_INT32; }
template <> inline hid_t nativeType<float>() { return H5T_NATIVE_FLOAT; }
template <> inline hid_t nativeType<double>() { return H5T_NATIVE_DOUBLE; }

template <typename T>
std::vector<T> readData(hid_t file, const char *path, std::size_t count) {
  Handle dataset(H5Dopen2(file, path, H5P_DEFAULT), H5Dclose, "open " + std::string(path));
  Handle space(H5Dget_space(dataset.get()), H5Sclose, "get dataspace");
  hsize_t dimension = 0;
  if (H5Sget_simple_extent_ndims(space.get()) != 1 ||
      H5Sget_simple_extent_dims(space.get(), &dimension, nullptr) < 0 || dimension != count)
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
  if (count)
    check(H5Dread(dataset.get(), nativeType<T>(), H5S_ALL, H5S_ALL, H5P_DEFAULT,
                  result.data()), "read " + std::string(path));
  return result;
}

template <typename T>
T readScalar(hid_t file, const char *path) { return readData<T>(file, path, 1).front(); }

template <typename T>
void writeData(hid_t file, const char *path, const T *values, std::size_t count) {
  hsize_t dimension = count;
  Handle space(H5Screate_simple(1, &dimension, nullptr), H5Sclose, "create dataspace");
  Handle dataset(H5Dcreate2(file, path, nativeType<T>(), space.get(), H5P_DEFAULT,
                           H5P_DEFAULT, H5P_DEFAULT), H5Dclose, "create " + std::string(path));
  if (count)
    check(H5Dwrite(dataset.get(), nativeType<T>(), H5S_ALL, H5S_ALL, H5P_DEFAULT, values),
          "write " + std::string(path));
}

template <typename T>
void writeScalar(hid_t file, const char *path, T value) { writeData(file, path, &value, 1); }

inline std::string readString(hid_t group, const char *name) {
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
  if (terminator != std::string::npos) value.resize(terminator);
  return value;
}

inline void writeString(hid_t group, const char *name, const std::string &value) {
  Handle type(H5Tcopy(H5T_C_S1), H5Tclose, "create string type");
  check(H5Tset_size(type.get(), value.size() + 1), "set string size");
  check(H5Tset_strpad(type.get(), H5T_STR_NULLTERM), "set string padding");
  Handle space(H5Screate(H5S_SCALAR), H5Sclose, "create attribute space");
  Handle attribute(H5Acreate2(group, name, type.get(), space.get(), H5P_DEFAULT,
                             H5P_DEFAULT), H5Aclose, "create attribute");
  check(H5Awrite(attribute.get(), type.get(), value.c_str()), "write attribute");
}

inline std::size_t checkedMultiply(std::size_t a, std::size_t b) {
  if (b && a > std::numeric_limits<std::size_t>::max() / b)
    throw std::runtime_error("HDF5: dimensions overflow size_t");
  return a * b;
}

}  // namespace vq_map_server::common::hdf5
