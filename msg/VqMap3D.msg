# SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
# SPDX-License-Identifier: Apache-2.0
std_msgs/Header header
uint32 format_version
float32 voxel_size
float32[3] origin
uint32[3] dimensions
uint32 block_size
uint32[3] block_dimensions
uint32 dictionary_size
uint8[] dictionary
uint32[] indices
