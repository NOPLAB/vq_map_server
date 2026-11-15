// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
// Adapted from vq_occupancy_compressor PatternDictionaryBuilder.cpp at
// 6157adcde9bd94b796f67dfaf1ffbd67492e3827: first-occurrence deduplication.
// Modified to store byte patterns with a bounded flat payload for 2D and 3D.
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vq_map_server::common {

class PatternDictionary {
public:
  PatternDictionary(std::size_t pattern_bytes, std::size_t max_bytes)
      : pattern_bytes_(pattern_bytes), max_bytes_(max_bytes) {
    if (pattern_bytes == 0)
      throw std::invalid_argument("pattern size must be positive");
  }

  std::uint32_t add(const std::vector<std::uint8_t> &pattern) {
    if (pattern.size() != pattern_bytes_)
      throw std::invalid_argument("pattern has incorrect byte length");
    const auto found = lookup_.find(pattern);
    if (found != lookup_.end()) return found->second;
    const auto next = bytes_.size() / pattern_bytes_;
    if (next >= std::numeric_limits<std::uint32_t>::max() ||
        bytes_.size() > max_bytes_ || pattern_bytes_ > max_bytes_ - bytes_.size())
      throw std::runtime_error("dictionary exceeds max_map_bytes or uint32 range");
    const auto index = static_cast<std::uint32_t>(next);
    lookup_.emplace(pattern, index);
    bytes_.insert(bytes_.end(), pattern.begin(), pattern.end());
    return index;
  }

  std::size_t size() const noexcept { return bytes_.size() / pattern_bytes_; }
  std::vector<std::uint8_t> take() && { return std::move(bytes_); }

private:
  std::size_t pattern_bytes_;
  std::size_t max_bytes_;
  std::map<std::vector<std::uint8_t>, std::uint32_t> lookup_;
  std::vector<std::uint8_t> bytes_;
};

}  // namespace vq_map_server::common
