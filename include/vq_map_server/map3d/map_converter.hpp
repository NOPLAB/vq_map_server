// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "vq_map_server/map3d/map_io.hpp"
#include <cstddef>
#include <cstdint>
#include <string>

namespace vq_map_server::map3d {

struct ConversionOptions {
  float voxel_size = 0.1F;
  std::uint32_t block_size = 13;
  std::string frame_id = "map";
  std::size_t max_map_bytes = kDefaultMaxMapBytes;
};

// PCD/PLY XYZ scalar FLOAT32 or FLOAT64; non-finite points are discarded.
// No finite points is an error. Coordinates are in metres in frame_id.
// Origin is the componentwise minimum; dimensions include the maximum point.
VqMap convertPointCloud(const std::string &path,
                        const ConversionOptions &options = {});

}  // namespace vq_map_server::map3d
