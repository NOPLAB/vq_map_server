# vq_map_server

Static, vector-quantized 3D occupancy maps for ROS 2 Humble. The package
provides PCD/PLY conversion, HDF5 storage, compressed map distribution, and
an immutable C++ query library.

New code is licensed under Apache-2.0. `LICENSE` and `NOTICE` include the
license and the attribution for the adapted upstream compression code.
The package layout follows emcl3d; it does not include or link emcl3d code.

## Build

Place this directory in a ROS 2 Humble workspace's `src` directory:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select vq_map_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vq_map_server --return-code-on-test-failure
colcon test-result --verbose
```

Dependencies are ROS 2 `rclcpp`, `rcl_interfaces`, standard/sensor messages, rosidl, PCL common
and I/O, HDF5 C, and yaml-cpp. GTest is used for tests. No OpenMP dependency
is added. Build artifacts and experimental maps should stay outside the
source directory.

## Convert a point cloud

```bash
ros2 run vq_map_server vq_map_convert \
  --input /absolute/path/map.pcd \
  --output /absolute/path/map.h5 \
  --voxel-size 0.1 --block-size 13 --frame-id map
```

PCD and PLY inputs are supported. PCD supports ASCII, binary, and
binary-compressed storage through PCL. XYZ fields must be scalar FLOAT32 or
FLOAT64. Coordinates are metres in the requested frame; no TF transformation
is performed. Non-finite points are discarded; no finite XYZ points is an
error. Other fields are not used.

The origin is the componentwise minimum point coordinate stored as float32.
Cell coordinates are `floor((point - origin) / voxel_size)`. Dimensions
include the maximum input point. The map uses cubic blocks with side length
`block_size`, between 1 and 64 cells. End blocks are padded with free cells.

Use a shared conversion configuration instead of individual options:

```bash
ros2 run vq_map_server vq_map_convert \
  --config /path/to/conversion.yaml \
  --input /path/to/map.ply --output /path/to/map.h5
```

The configuration has the same ROS wildcard structure as the installed
`config/conversion.yaml`:

```yaml
/**:
  ros__parameters:
    voxel_size: 0.1
    block_size: 13
    frame_id: map
    max_map_bytes: 1073741824
```

Explicit CLI options override the configuration regardless of argument order.
`--max-map-bytes N` sets the maximum dictionary-plus-uint32-index payload.
Output creation is exclusive: an existing output is never overwritten.
Conversion reports elapsed time, retained map storage, and process peak RSS.
`--help` lists the command-line options.

## Start the server

Read a preconverted map:

```bash
ros2 launch vq_map_server vq_map_server.launch.xml \
  map_path:=/absolute/path/map.h5
```

Or convert a point cloud at startup:

```bash
ros2 launch vq_map_server vq_map_server.launch.xml \
  map_path:=/absolute/path/map.pcd
```

The server calls the same conversion library as the CLI. It only exposes
the map after conversion succeeds. There is no implicit cache or output
file. To also save the result:

```bash
ros2 launch vq_map_server vq_map_server.launch.xml \
  map_path:=/absolute/path/map.pcd \
  converted_map_path:=/absolute/path/map.h5
```

`converted_map_path` is only valid for point-cloud input and must not exist.
An unreadable/invalid map, a failed conversion, or a failed requested save
terminates startup with a nonzero exit code.

Other launch arguments are `params_file`, `conversion_params_file`, and
`use_sim_time`. Both parameter files can be replaced:

```bash
ros2 launch vq_map_server vq_map_server.launch.xml \
  map_path:=/absolute/path/map.ply \
  conversion_params_file:=/path/to/conversion.yaml \
  params_file:=/path/to/server.yaml
```

Parameters are read at startup. Restart the node to change the map or its
settings. HDF5 metadata, including frame, origin, dimensions, and resolution,
is authoritative; HDF5 maps are not revoxelized. The server does not publish
TF or transform input data.

## ROS interfaces

The default node name is `vq_map_server`; `~` denotes its private namespace.

| Interface | Type | Behavior |
| --- | --- | --- |
| `~/map` | `vq_map_server/msg/VqMap` | Reliable, transient-local, depth 1 compressed snapshot. |
| `~/get_map` | `vq_map_server/srv/GetVqMap` | Empty request; returns the same snapshot. |
| `~/occupied_voxels` | `sensor_msgs/msg/PointCloud2` | Optional bounded visualization, also retained for late subscribers. |

Subscribers requiring the retained map must request transient-local
durability. The snapshot header is stamped at load completion and retained
unchanged, including in service responses. With `use_sim_time`, startup
before the first clock sample can produce stamp zero; this is a static
snapshot, not a TF lookup.

`VqMap` contains format version 1, frame/stamp, float32 resolution/origin,
uint32 grid/block dimensions, cubic block size, dictionary size, byte-packed
dictionary patterns, and unpacked uint32 block indices.

Consumers should receive the compressed snapshot once, construct a validated
`VqMap`, and query it locally. Per-point ROS service calls are not part of
the observation path.

## Occupancy and memory contract

Only two states are stored: occupied and free. A cell containing a finite
input point is occupied; other cells are free. **Free does not mean observed
free space.** Unknown cells, ray tracing, probabilities, and online map
updates are outside this format.

VQ deduplication is lossless for this binary voxel grid. Voxelizing a point
cloud is discretization and does not preserve the original point positions.

Blocks and voxels use x-fastest order:

```text
linear = x + width_x * (y + width_y * z)
```

Each dictionary pattern starts at a byte boundary. Its first voxel is the
least significant bit of its first byte. `VqMap::at()` and `atWorld()` return
`Free`, `Occupied`, or `OutOfBounds`. Original bounds are half-open; padded
cells are never queryable as valid map cells. The immutable core supports
concurrent queries without locks.

The server retains the compressed message, not a dense voxel array or a
KD-tree. Reading HDF5 temporarily holds packed and unpacked indices.
Conversion temporarily holds the source point cloud, occupied block patterns,
and dictionary lookup data. `max_map_bytes` limits final payload, **not
total process RSS**. Convert large maps offline, then load HDF5 on the robot.

`memoryBytes()` reports retained C++ map object/vector storage. Serialized
ROS messages, DDS history and transport buffers, PCL, HDF5, and temporary
conversion allocations are additional memory.

## Visualization

`publish_occupied_voxels` defaults to false. When enabled, the server emits
occupied cell centers inside the half-open `visualization_min` /
`visualization_max` world-space box. Defaults are `[-10,-10,-2]` and
`[10,10,3]` metres. `max_visualization_points` defaults to 100000 and may be
between 1 and 1000000. The result stops at the limit in x-fastest order; it
is a bounded preview, not an unbiased sample of the map.

No full-map point cloud is created for normal map distribution.

## Inspect and measure

```bash
ros2 run vq_map_server vq_map_inspect /absolute/path/map.h5
ros2 run vq_map_server vq_map_inspect /absolute/path/map.h5 --queries 1000000
```

Inspection reports metadata, voxel/block/dictionary counts, storage bytes,
and load time. The optional query benchmark uses a fixed seed for repeatable
uniform random in-map cell access. It measures the core lookup path, not
self-localization accuracy or an end-to-end localization cycle.

## HDF5 compatibility

The schema follows `cafeline/vq_occupancy_compressor` revision
`6157adcde9bd94b796f67dfaf1ffbd67492e3827`:

| Group | Required data |
| --- | --- |
| `/metadata` | Scalar fixed-length string attributes `version=1.0.0`, `compression_method=voxel_dictionary`, `frame_id`. |
| `/compression_params` | `voxel_size`, `block_size`, `pattern_bits`, `dictionary_size`, `block_index_bit_width`, `grid_origin`, `grid_dimensions`. |
| `/dictionary` | `pattern_length` in bytes and flattened uint8 `patterns`. |
| `/compressed_data` | int32 `block_dims` and LSB-first bit-packed uint8 `block_indices`. |

Numeric scalar datasets have shape `[1]`, vectors `[3]`, and dictionary /
index data one dimension. Types match upstream float32, int32, and uint32.
Index widths 1 through 32 are accepted. Indices must refer to existing
dictionary patterns. Optional upstream statistics are not required.
Raw-grid HDF5, other schema versions/methods, and malformed shapes/types
are rejected. Compatibility with other upstream revisions is not implied.

## Scope and attribution

Core queries, HDF5 interoperability, point-cloud conversion, command-line
failure behavior, retained ROS delivery, and visualization are covered by
tests. CI builds and tests ROS 2 Humble on Ubuntu 22.04.

This package does not change emcl3d. Its nearest-point distance observation
model is different from binary occupancy queries; integrating localization
requires a separate observation-model decision.

The design is based on Funai, Hayashibara, and Ueda, "Compression of 3D
Occupancy Grid Map for Mobile Robot Localization using Vector Quantization",
31st Robotics Symposia, 2026, pp. 15-21. See `NOTICE` for upstream source
attribution and modifications.

Apache-2.0 applies to this package's code, not to all system dependencies.
When redistributing binaries, retain the applicable dependency licenses.
PCL I/O can pull in additional system libraries; check the actual packaged
dependency closure rather than assuming it is entirely Apache-2.0.
