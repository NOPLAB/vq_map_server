// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "vq_map_server/map2d/map_io.hpp"
#include <cstddef>
#include <cstdint>
#include <string>

namespace vq_map_server::map2d {

enum class UnknownPolicy { Reject, Free, Occupied };

struct ConversionOptions {
  std::uint32_t block_size = 24;
  std::string frame_id = "map";
  CellEncoding encoding = CellEncoding::Ternary;
  UnknownPolicy unknown_policy = UnknownPolicy::Reject;
  std::size_t max_map_bytes = kDefaultMaxMapBytes;
};

// Nav2 trinary YAML + PGM/PNG. Resolution and XY/yaw origin come from YAML.
// Ternary preserves unknown. Binary rejects it unless a replacement is explicit.
VqMap convertOccupancyGrid(const std::string &path,
                          const ConversionOptions &options = {});

}  // namespace vq_map_server::map2d
