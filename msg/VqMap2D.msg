# SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
# SPDX-License-Identifier: Apache-2.0
uint8 BINARY=1
uint8 TERNARY=2
std_msgs/Header header
uint32 format_version
float32 resolution
float64[2] origin
float64 yaw
uint32[2] dimensions
uint32 block_size
uint32[2] block_dimensions
uint8 cell_encoding
uint8 index_bit_width
uint32 dictionary_size
uint8[] dictionary
uint8[] indices
