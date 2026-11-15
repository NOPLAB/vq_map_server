# vq_map_server

Static vector-quantized 2D and 3D occupancy maps for ROS 2 Humble.
Both servers distribute compressed maps for immutable local C++ queries.
Conversion, inspection, launch, configuration, interfaces, and source directories
are explicitly separated by dimension. Code is licensed under Apache-2.0;
see LICENSE and NOTICE for upstream adaptations and paper attribution.

## Build and layout

Place the package in a ROS 2 Humble workspace's src directory:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select vq_map_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vq_map_server --return-code-on-test-failure
colcon test-result --verbose
```

Dependencies include rclcpp, standard/sensor/navigation messages, rosidl,
Nav2 MapIO (nav2_map_server), PCL common/I/O, HDF5 C, and yaml-cpp.
GTest is used for tests. No shell scripts or OpenMP dependency are added.
Keep build artifacts and experimental maps outside the source directory.

```text
include/vq_map_server/{map2d,map3d}/   public C++ headers
src/{map2d,map3d}/                   geometry, conversion, I/O, CLI, server
src/common/                         dictionary and HDF5 primitives
config/{map2d,map3d}/                conversion.yaml and server.yaml
launch/vq_map_server_{2d,3d}.launch.xml
test/{map2d,map3d,common}/
```

Exported libraries are vq_map_core_2d/3d, vq_map_io_2d/3d, and
vq_map_conversion_2d/3d. Public namespaces are vq_map_server::map2d and
vq_map_server::map3d. Core map queries do not depend on ROS, PCL, or HDF5.

## 2D conversion

```bash
ros2 run vq_map_server vq_map_convert_2d \
  --input /maps/map.yaml --output /maps/map_2d.h5

# Paper-style binary compression; unknown cells cause an error by default.
ros2 run vq_map_server vq_map_convert_2d \
  --input /maps/map.yaml --output /maps/map_binary.h5 \
  --cell-encoding binary --unknown-policy occupied
```

Input is a Nav2 map YAML/YML referencing PGM or PNG, including relative image
paths. Only trinary mode is supported; an absent mode means trinary.
Nav2 MapIO handles image row inversion, thresholds, negate, and alpha.
Scale/raw modes are rejected rather than silently reducing probabilities.
Resolution and XY/yaw origin come from the source YAML. Resolution is stored
as float32, matching OccupancyGrid; XY/yaw are float64. No TF is performed.

Ternary encoding is the default and preserves free, occupied, and unknown
cells in two bits. Binary encoding uses one bit. For binary input containing
unknown cells, unknown_policy must explicitly choose free or occupied;
reject is the default. A replacement policy is invalid with ternary encoding.
Replacing unknown cells changes the map; VQ is lossless for the resulting
cell grid, not for the original grayscale image.

The default square block side is 24 cells, following the RSJ2025 example.
Sides 1 through 64 are supported; edge padding is free and outside map bounds.
Dictionary patterns are assigned in first-occurrence x-fastest block order.
Block IDs use the smallest sufficient 8/16/32-bit representation in RAM,
HDF5, and ROS messages. For example, up to 256 patterns use one byte per ID.

Both conversion CLIs accept --config FILE with the installed ROS wildcard
YAML structure. Explicit flags override configuration regardless of order.
Unknown/duplicate CLI options and unknown configuration parameters are errors.
All HDF5 output creation is exclusive: existing files are never overwritten.
Both CLIs report elapsed_ms, memory_bytes, and peak_rss_kib.

```yaml
/**:
  ros__parameters:
    block_size: 24
    frame_id: map
    cell_encoding: ternary
    unknown_policy: reject
    max_map_bytes: 1073741824
```

## 3D conversion

```bash
ros2 run vq_map_server vq_map_convert_3d \
  --input /maps/map.pcd --output /maps/map_3d.h5 \
  --voxel-size 0.1 --block-size 13 --frame-id map
```

PCD (ASCII, binary, binary-compressed) and PLY are read through PCL.
XYZ must be scalar FLOAT32 or FLOAT64 in metres in the requested frame.
Other fields are ignored. Non-finite points are discarded; no finite XYZ
points is an error. Origin is the componentwise minimum stored as float32,
rounded downward when necessary to retain a FLOAT64 minimum. Coordinates are
floor((point-origin)/voxel_size); dimensions include the maximum point.
Blocks are cubic, default side 13, supported range 1 through 64.

Only binary occupancy is stored: cells containing points are occupied;
others are free. Free does not mean observed free space. Unknown states,
probabilities, ray tracing, and online updates are not part of the 3D format.
VQ is lossless for the binary voxel grid; voxelization discretizes point data.
The 3D conversion configuration contains voxel_size, block_size, frame_id,
and max_map_bytes under the same wildcard ROS parameter mapping.

## Start either server

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
converted_map_path:=/maps/new.h5 saves the converted result exclusively;
this option is valid only with source input, not HDF5 input.
A wrong-dimensional HDF5 file, unreadable input, conversion error, failed
requested save, or oversized requested expansion fails startup before map
endpoints are advertised. The server exits nonzero on startup failure.

Other launch arguments are params_file, conversion_params_file, and
use_sim_time. Default files are config/map2d or config/map3d as appropriate.
Parameters are read-only at startup; restart to change the map/settings.
Stored HDF5 metadata overrides conversion geometry/frame settings.
No server publishes TF, reloads maps, or performs online updates.

## ROS interfaces and C++ queries

Default node names are vq_map_server_2d and vq_map_server_3d. Each has:

| Private endpoint | 2D type | 3D type |
| --- | --- | --- |
| ~/map | msg/VqMap2D | msg/VqMap3D |
| ~/get_map | srv/GetVqMap2D | srv/GetVqMap3D |

The map topic uses reliable, transient-local, depth-1 QoS. Late subscribers
must request transient-local durability. The empty-request service returns
the identical snapshot, including the original load-completion stamp.
With use_sim_time before the first clock sample, this stamp can be zero.
Different node names keep both dimensions' endpoints distinct when run together.

2D snapshots contain float32 resolution, float64 XY/yaw, dimensions,
block dimensions, cell encoding, index width, dictionary, and byte indices.
3D snapshots contain float32 voxel size/XYZ origin, dimensions, cubic block
metadata, binary dictionary, and unpacked uint32 indices. Both have a header
and format_version=1. Use the matching C++ VqMap constructor to validate a
received payload; do not issue per-cell ROS service calls during localization.

VqMap::at() and atWorld() return Free, Occupied, or OutOfBounds; 2D also
returns Unknown. Queries are immutable and support concurrent readers.
World queries floor local coordinates; 2D first applies inverse origin yaw.
Bounds are half-open, and padded cells are never queryable. Both formats use
x-fastest layout (x+width*y in 2D; x+width_x*(y+width_y*z) in 3D).
Patterns are byte-aligned and LSB-first. 2D codes are 0 free, 1 occupied,
2 unknown; 3 is invalid. Indices in 2D are little-endian byte records.

## Optional standard output and visualization

2D publish_occupancy_grid defaults to false. Enabling it publishes a complete
retained nav_msgs/OccupancyGrid on ~/occupancy_grid, with 0/100/-1 values,
matching frame/stamp, and the full XY/yaw origin. max_occupancy_grid_cells
(default 1000000, allowed 1..100000000) limits the entire expanded grid;
exceeding it fails startup. Remap this topic for standard Nav2 consumers.
This path expands the map and incurs dense-grid/DDS memory costs. Existing
localizers need integration to query compressed maps directly.

3D publish_occupied_voxels defaults to false. Enabling it publishes retained
PointCloud2 cell centers on ~/occupied_voxels. The half-open world-space ROI
defaults to visualization_min=[-10,-10,-2], visualization_max=[10,10,3].
max_visualization_points defaults to 100000, allowed 1..1000000.
The preview stops at the cap in x-fastest order; it is not an unbiased sample.

## Storage and memory limits

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

max_map_bytes (default 1 GiB) caps final dictionary plus resident indices,
not total process RSS. Conversion also temporarily holds the decoded source,
dictionary lookup, and scratch block IDs. 3D HDF5 loading temporarily holds
packed and uint32 indices. Servers retain a compressed snapshot, not both a
VqMap and message or a dense map by default. DDS history/transport buffers
add memory. memoryBytes() reports retained C++ object/vector storage.
For large maps, convert offline and load HDF5 on the robot.

## Inspect and measure

```bash
ros2 run vq_map_server vq_map_inspect_2d /maps/map_2d.h5 --queries 1000000
ros2 run vq_map_server vq_map_inspect_3d /maps/map_3d.h5 --queries 1000000
```

Inspectors report map metadata, storage, load time, and RSS. Optional queries
use fixed-seed uniform random in-map access; timing includes query generation.
This measures lookup, not localization accuracy or a full observation cycle.
CI builds/tests both dimensions on ROS 2 Humble / Ubuntu 22.04.

## Migration from the original 3D-only interface

The ROS package name stays vq_map_server. Generic old names are removed:

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

## References and licensing

The 2D implementation follows Funai, Hayashibara, and Ueda, RSJ2025AC3Q3-08:
block deduplication and direct compressed queries. Ternary support extends the
paper's binary representation. The 3D implementation follows their 31st
Robotics Symposia paper (2026, pp.15-21). See NOTICE for full attribution.

No emcl2/emcl3d code is included or linked; localization integration is outside
this package. Apache-2.0 applies to this package's code, not all system
libraries. Retain dependency licenses when redistributing binaries and inspect
the actual packaged dependency closure, including Nav2 MapIO and PCL I/O.
