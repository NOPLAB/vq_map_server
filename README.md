# vq_map_server

[![Humble CI](https://img.shields.io/github/check-runs/NOPLAB/vq_map_server/main?nameFilter=ROS%202%20humble%20build%20and%20test%20%28ubuntu-24.04%29&label=Humble)](https://github.com/NOPLAB/vq_map_server/actions/workflows/ci.yaml?query=branch%3Amain)
[![Jazzy CI](https://img.shields.io/github/check-runs/NOPLAB/vq_map_server/main?nameFilter=ROS%202%20jazzy%20build%20and%20test%20%28ubuntu-24.04%29&label=Jazzy)](https://github.com/NOPLAB/vq_map_server/actions/workflows/ci.yaml?query=branch%3Amain)
[![Lyrical CI](https://img.shields.io/github/check-runs/NOPLAB/vq_map_server/main?nameFilter=ROS%202%20lyrical%20build%20and%20test%20%28ubuntu-24.04%29&label=Lyrical)](https://github.com/NOPLAB/vq_map_server/actions/workflows/ci.yaml?query=branch%3Amain)

## Overview

Static vector-quantized occupancy maps for ROS 2. The package converts,
stores, and distributes compressed 2D and 3D maps for immutable local C++
queries. Repeated blocks share a dictionary entry, so consumers can query
compressed maps without expanding the entire grid.

The two dimensions have separate converters, inspectors, servers, launch
files, configuration, and ROS interfaces. The package does not perform
localization or update maps online.

## Requirements

CI covers these ROS 2 distributions on x86_64 and ARM64:

| ROS 2 | Container OS |
| --- | --- |
| Humble | Ubuntu 22.04 |
| Jazzy | Ubuntu 24.04 |
| Lyrical | Ubuntu 26.04 |

Install ROS 2, `colcon`, and `rosdep`, and initialize rosdep before building.
[package.xml](package.xml) declares the dependencies, including rclcpp,
ROS messages and rosidl, Nav2 MapIO, PCL common/I/O, HDF5 C, and yaml-cpp.
Tests use GTest. OpenMP is not required.

Supported source maps are Nav2 trinary YAML with PGM/PNG for 2D and PCD/PLY
point clouds for 3D. Servers also load the matching compressed HDF5 format.

## Build

The examples use Humble. Replace `humble` with `jazzy` or `lyrical` to use another
supported distribution.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NOPLAB/vq_map_server.git
cd ~/ros2_ws
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --packages-select vq_map_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

Keep build artifacts and experimental maps outside the source directory.
The package layout separates 2D and 3D:

```text
include/vq_map_server/{map2d,map3d}/   public C++ headers
src/{map2d,map3d}/                   geometry, conversion, I/O, CLI, server
src/common/                         dictionary and HDF5 primitives
config/{map2d,map3d}/                conversion.yaml and server.yaml
launch/vq_map_server_{2d,3d}.launch.xml
test/{map2d,map3d,common}/
```

Exported libraries are `vq_map_core_2d/3d`, `vq_map_io_2d/3d`, and
`vq_map_conversion_2d/3d`. Public namespaces are `vq_map_server::map2d` and
`vq_map_server::map3d`. Core map queries do not depend on ROS, PCL, or HDF5.

## Run

From the workspace root, source the installed package before running commands:

```bash
source install/setup.bash
```

Use `ros2 run` for conversion and inspection, and `ros2 launch` for servers.

### Convert a 2D map

```bash
ros2 run vq_map_server vq_map_convert_2d \
  --input /maps/map.yaml --output /maps/map_2d.h5

# Paper-style binary compression; unknown cells cause an error by default.
ros2 run vq_map_server vq_map_convert_2d \
  --input /maps/map.yaml --output /maps/map_binary.h5 \
  --cell-encoding binary --unknown-policy occupied
```

Input is a Nav2 YAML/YML map referencing a PGM or PNG image, including relative
image paths. Only trinary mode is supported; an absent mode means trinary.
The default ternary encoding preserves unknown cells. Binary encoding requires
an explicit replacement policy when unknown cells exist.

### Convert a 3D map

```bash
ros2 run vq_map_server vq_map_convert_3d \
  --input /maps/map.pcd --output /maps/map_3d.h5 \
  --voxel-size 0.1 --block-size 13 --frame-id map
```

PCL reads ASCII, binary, and binary-compressed PCD files, as well as PLY.
XYZ fields must be scalar FLOAT32 or FLOAT64 in metres in the requested frame.
Non-finite points are discarded; input with no finite XYZ points is rejected.
No TF conversion is performed.

All HDF5 output creation is exclusive: existing files are never overwritten.
Both converters report `elapsed_ms`, `memory_bytes`, and `peak_rss_kib`.

### Start a server

```bash
ros2 launch vq_map_server vq_map_server_2d.launch.xml \
  map_path:=/maps/map.yaml
ros2 launch vq_map_server vq_map_server_3d.launch.xml \
  map_path:=/maps/map.pcd

# Direct HDF5 loading uses authoritative stored metadata.
ros2 launch vq_map_server vq_map_server_2d.launch.xml \
  map_path:=/maps/map_2d.h5
ros2 launch vq_map_server vq_map_server_3d.launch.xml \
  map_path:=/maps/map_3d.h5
```

Both servers can convert their source format at startup using the same library
as the CLI. No implicit cache or output file is created. Optional
`converted_map_path:=/maps/new.h5` saves the converted result exclusively;
this option is valid only with source input, not HDF5 input.
A wrong-dimensional HDF5 file, unreadable input, conversion error, failed
requested save, or oversized requested expansion fails startup before map
endpoints are advertised. The server exits nonzero on startup failure.

Other launch arguments and defaults are listed under
[Configuration](#configuration). Stored HDF5 metadata overrides conversion
geometry and frame settings. Neither server publishes TF, reloads maps, or
performs online updates.

## ROS interfaces

Default node names are `vq_map_server_2d` and `vq_map_server_3d`. Each exposes:

| Private endpoint | 2D type | 3D type |
| --- | --- | --- |
| `~/map` | `vq_map_server/msg/VqMap2D` | `vq_map_server/msg/VqMap3D` |
| `~/get_map` | `vq_map_server/srv/GetVqMap2D` | `vq_map_server/srv/GetVqMap3D` |

The map topic uses reliable, transient-local, depth-1 QoS. Late subscribers
must request transient-local durability. The empty-request service returns
the identical snapshot, including the original load-completion stamp.
With `use_sim_time` before the first clock sample, this stamp can be zero.
Different node names keep both dimensions' endpoints distinct when run together.

2D snapshots contain float32 resolution, float64 XY/yaw, dimensions,
block dimensions, cell encoding, index width, dictionary, and byte indices.
3D snapshots contain float32 voxel size/XYZ origin, dimensions, cubic block
metadata, binary dictionary, and unpacked uint32 indices. Both have a header
and `format_version=1`. Use the matching C++ `VqMap` constructor to validate a
received payload; do not issue per-cell ROS service calls during localization.

## Configuration

### Conversion and launch settings

Conversion settings live in
[config/map2d/conversion.yaml](config/map2d/conversion.yaml) and
[config/map3d/conversion.yaml](config/map3d/conversion.yaml):

| Parameter | 2D default | 3D default | Meaning |
| --- | --- | --- | --- |
| `block_size` | `24` | `13` | Square or cubic block side; 1 through 64 cells |
| `frame_id` | `map` | `map` | Map frame; inputs must already use this frame |
| `cell_encoding` | `ternary` | Not configurable | 2D `ternary` or `binary`; 3D is always binary |
| `unknown_policy` | `reject` | Not applicable | For 2D binary conversion: `reject`, `free`, or `occupied` |
| `voxel_size` | Not applicable | `0.1` | 3D voxel size in metres; 2D resolution comes from the source YAML |
| `max_map_bytes` | `1073741824` | `1073741824` | Final dictionary and resident-index memory limit |

Both converters accept `--config FILE` using the ROS wildcard YAML structure:

```yaml
/**:
  ros__parameters:
    block_size: 24
    frame_id: map
    cell_encoding: ternary
    unknown_policy: reject
    max_map_bytes: 1073741824
```

Explicit CLI flags override configuration regardless of order.
Unknown or duplicate options and unknown configuration parameters are errors.
An unknown-cell replacement policy is invalid with ternary encoding.

Server launch files accept:

| Argument | Default | Meaning |
| --- | --- | --- |
| `map_path` | Required | Source map or matching HDF5 file |
| `converted_map_path` | Empty | Save a newly converted source map to a new HDF5 file |
| `params_file` | `config/map2d/server.yaml` or `config/map3d/server.yaml` | Server settings |
| `conversion_params_file` | Matching dimension's `conversion.yaml` | Startup conversion settings |
| `use_sim_time` | `false` | Use the ROS simulation clock |

Default configuration paths are resolved from the installed package.
Parameters are read-only at startup; restart to change the map or settings.

### Standard output and visualization

2D `publish_occupancy_grid` defaults to `false`. Enabling it publishes a complete
retained `nav_msgs/msg/OccupancyGrid` on `~/occupancy_grid`, with `0/100/-1` values,
matching frame/stamp, and the full XY/yaw origin. `max_occupancy_grid_cells`
(default 1,000,000, allowed 1 through 100,000,000) limits the expanded grid;
exceeding it fails startup. Remap this topic for standard Nav2 consumers.
This path expands the map and incurs dense-grid/DDS memory costs. Existing
localizers need integration to query compressed maps directly.

3D `publish_occupied_voxels` defaults to `false`. Enabling it publishes retained
`sensor_msgs/msg/PointCloud2` cell centres on `~/occupied_voxels`. The half-open
world-space ROI defaults to `visualization_min=[-10,-10,-2]` and
`visualization_max=[10,10,3]`.
`max_visualization_points` defaults to 100,000, allowed 1 through 1,000,000.
The preview stops at the cap in x-fastest order; it is not an unbiased sample.
Server defaults are in [config/map2d/server.yaml](config/map2d/server.yaml) and
[config/map3d/server.yaml](config/map3d/server.yaml).

## How it works

### 2D representation

Nav2 MapIO handles image row inversion, thresholds, negate, and alpha.
Scale/raw modes are rejected rather than reducing probabilities.
Resolution and XY/yaw origin come from the source YAML: resolution is stored
as float32, matching OccupancyGrid, while XY/yaw use float64. No TF is performed.
Transparent PNG pixels can decode differently across Nav2 versions; saved
HDF5 snapshots preserve the converted cell grid.

Ternary encoding preserves free, occupied, and unknown cells in two bits;
binary encoding uses one bit. Replacing unknown cells changes the map.
VQ is lossless for the resulting cell grid, not the original grayscale image.

The default square block side is 24 cells, following the RSJ2025 example.
Edge padding is free and outside map bounds. Dictionary patterns are assigned
in first-occurrence x-fastest block order. Block IDs use the smallest sufficient
8/16/32-bit representation in RAM, HDF5, and ROS messages; up to 256 patterns
therefore need one byte per ID.

### 3D representation

Other point-cloud fields are ignored. The origin is the componentwise minimum,
stored as float32 and rounded downward when needed to retain a FLOAT64 minimum.
Coordinates are `floor((point-origin)/voxel_size)`; dimensions include the
maximum point. Blocks are cubic with a default side of 13 cells.

Only binary occupancy is stored: cells containing points are occupied;
others are free. Free does not mean observed free space. Unknown states,
probabilities, ray tracing, and online updates are outside the 3D format.
VQ is lossless for the binary voxel grid; voxelization discretizes point data.

### Local C++ queries

`VqMap::at()` and `atWorld()` return `Free`, `Occupied`, or `OutOfBounds`;
2D also returns `Unknown`. Queries are immutable and support concurrent readers.
World queries floor local coordinates; 2D first applies inverse origin yaw.
Bounds are half-open, and padded cells are never queryable.

Both formats use x-fastest layout: `x+width*y` in 2D and
`x+width_x*(y+width_y*z)` in 3D. Patterns are byte-aligned and LSB-first.
2D codes are `0` free, `1` occupied, and `2` unknown; `3` is invalid.
2D indices are little-endian byte records.

### Storage and memory limits

2D uses a separate grid_dictionary_2d HDF5 schema, version 1.0.0.
/metadata has fixed scalar string attributes version, compression_method,
and frame_id. /compression_params stores map_dimension=2, float32 resolution,
float64 grid_origin[2] and grid_yaw, uint32 grid_dimensions[2], block_size,
pattern_bits, dictionary_size, and uint8 cell_encoding and
block_index_bit_width (8,16,32). /dictionary contains uint32 pattern_length
and flat uint8 patterns. /compressed_data contains uint32 block_dims[2]
and uint8 block_indices. Numeric scalars have shape [1].

The 3D schema remains compatible with cafeline/vq_occupancy_compressor
revision 6157adcde9bd94b796f67dfaf1ffbd67492e3827, version 1.0.0,
compression_method=voxel_dictionary. Its compression_params use voxel_size,
block_size, pattern_bits, dictionary_size, block_index_bit_width (1..32),
float32 grid_origin[3], and int32 grid_dimensions[3]. The dictionary contains
pattern_length and flat uint8 patterns; compressed_data contains int32
block_dims[3] and LSB-first bit-packed indices. Optional statistics are ignored.
Other schemas/versions, raw-grid HDF5, and malformed shapes/types are rejected.

`max_map_bytes` (default 1 GiB) caps final dictionary plus resident indices,
not total process RSS. Conversion also temporarily holds the decoded source,
dictionary lookup, and scratch block IDs. 3D HDF5 loading temporarily holds
packed and uint32 indices. Servers retain a compressed snapshot, not both a
VqMap and message or a dense map by default. DDS history/transport buffers
add memory. `memoryBytes()` reports retained C++ object/vector storage.
For large maps, convert offline and load HDF5 on the robot.

### Migration from the original 3D-only interface

The ROS package name stays `vq_map_server`. Generic old names are removed:

| Old | Replacement |
| --- | --- |
| vq_map_convert / vq_map_inspect | vq_map_convert_3d / vq_map_inspect_3d |
| vq_map_server_node and node vq_map_server | vq_map_server_3d |
| vq_map_server.launch.xml | vq_map_server_3d.launch.xml |
| config/conversion.yaml / config/vq_map_server.yaml | config/map3d/conversion.yaml / server.yaml |
| include/vq_map_server/*.hpp | include/vq_map_server/map3d/*.hpp |
| namespace vq_map_server | namespace vq_map_server::map3d |
| VqMap / GetVqMap ROS types | VqMap3D / GetVqMap3D |
| vq_map_core/io/conversion libraries | vq_map_core_3d / vq_map_io_3d / vq_map_conversion_3d |

Update dependent source, remappings, node-keyed parameter files, and launch
commands; perform a clean build/install to remove stale installed old names.
Existing 3D HDF5 files need no conversion. No compatibility wrappers remain.

## Testing

CI builds and tests both dimensions on Humble, Jazzy, and Lyrical, on x86_64
and ARM64. The badges at the top track each distribution's x86_64 build-and-test
job on `main`; follow them to view the complete matrix.

From the workspace root after building:

```bash
source install/setup.bash
colcon test --packages-select vq_map_server \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

Tests cover compressed queries, HDF5 I/O, conversion, CLI behavior, and both ROS
servers. CI uploads the test results and checks both installed launch files
and all converter and inspector entry points.

### Inspect and measure

```bash
ros2 run vq_map_server vq_map_inspect_2d /maps/map_2d.h5 --queries 1000000
ros2 run vq_map_server vq_map_inspect_3d /maps/map_3d.h5 --queries 1000000
```

Inspectors report metadata, storage, load time, and RSS. Optional queries use
fixed-seed uniform random in-map access; timing includes query generation.
This measures lookup, not localization accuracy or a full observation cycle.

## License and attribution

Author: nop (noplab90@gmail.com). Copyright 2026 nop.
Code is licensed under [Apache-2.0](LICENSE).

The 2D implementation follows Funai, Hayashibara, and Ueda, RSJ2025AC3Q3-08:
block deduplication and direct compressed queries. Ternary support extends the
paper's binary representation. The 3D implementation follows their 31st
Robotics Symposia paper (2026, pp.15-21). See [NOTICE](NOTICE) for full attribution
and the upstream source revision.

No emcl2/emcl3d code is included or linked; localization integration is outside
this package. Apache-2.0 applies to this package's code, not all system
libraries. Retain dependency licenses when redistributing binaries and inspect
the actual packaged dependency closure, including Nav2 MapIO and PCL I/O.
