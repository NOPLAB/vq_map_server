// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "vq_map_server/map3d/vq_map.hpp"
#include <cstddef>
#include <string>

namespace vq_map_server::map3d {

inline constexpr std::size_t kDefaultMaxMapBytes = 1024ULL * 1024ULL * 1024ULL;

// Reads the upstream 1.0.0 voxel_dictionary schema, including packed indices.
VqMap readHdf5(const std::string &path,
               std::size_t max_map_bytes = kDefaultMaxMapBytes);
// Exclusive create: never overwrites an existing file.
void writeHdf5(const std::string &path, const VqMap &map);

}  // namespace vq_map_server::map3d
