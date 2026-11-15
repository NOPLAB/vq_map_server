// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map3d/map_converter.hpp"
#include "vq_map_server/msg/vq_map3_d.hpp"
#include "vq_map_server/srv/get_vq_map3_d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace vq_map_server::map3d {

class VqMapServer final : public rclcpp::Node {
public:
  VqMapServer() : Node("vq_map_server_3d") {
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    const auto path = declare_parameter<std::string>("map_path", "", immutable);
    const auto converted_path =
        declare_parameter<std::string>("converted_map_path", "", immutable);
    const auto voxel_size = declare_parameter<double>("voxel_size", 0.1, immutable);
    const auto block_size =
        declare_parameter<std::int64_t>("block_size", 13, immutable);
    const auto frame_id = declare_parameter<std::string>("frame_id", "map", immutable);
    const auto max_bytes =
        declare_parameter<std::int64_t>("max_map_bytes", 1073741824, immutable);
    const auto visualize =
        declare_parameter<bool>("publish_occupied_voxels", false, immutable);
    const auto minimum = declare_parameter<std::vector<double>>(
        "visualization_min", {-10.0, -10.0, -2.0}, immutable);
    const auto maximum = declare_parameter<std::vector<double>>(
        "visualization_max", {10.0, 10.0, 3.0}, immutable);
    const auto max_points =
        declare_parameter<std::int64_t>("max_visualization_points", 100000, immutable);

    if (path.empty()) {
      throw std::invalid_argument("map_path is required");
    }
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0 ||
        voxel_size > std::numeric_limits<float>::max() ||
        static_cast<float>(voxel_size) <= 0.0F) {
      throw std::invalid_argument("voxel_size must be positive and representable as float32");
    }
    if (block_size <= 0 || block_size > 64) {
      throw std::invalid_argument("block_size must be in [1, 64]");
    }
    if (max_bytes <= 0 ||
        static_cast<std::uint64_t>(max_bytes) > std::numeric_limits<std::size_t>::max()) {
      throw std::invalid_argument("max_map_bytes must fit a positive size_t");
    }
    if (max_points <= 0 || max_points > 1000000) {
      throw std::invalid_argument("max_visualization_points must be in [1, 1000000]");
    }
    if (minimum.size() != 3 || maximum.size() != 3) {
      throw std::invalid_argument("visualization bounds require three coordinates");
    }
    std::array<double, 3> lower{}, upper{};
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(minimum[i]) || !std::isfinite(maximum[i]) ||
          minimum[i] >= maximum[i]) {
        throw std::invalid_argument("visualization bounds must be finite and increasing");
      }
      lower[i] = minimum[i];
      upper[i] = maximum[i];
    }
    auto extension = std::filesystem::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool point_input = extension == ".pcd" || extension == ".ply";
    if (!point_input && extension != ".h5" && extension != ".hdf5") {
      throw std::invalid_argument("map_path must be PCD, PLY, H5, or HDF5");
    }
    if (!point_input && !converted_path.empty()) {
      throw std::invalid_argument("converted_map_path is only valid for pointcloud input");
    }
    ConversionOptions options;
    options.voxel_size = static_cast<float>(voxel_size);
    options.block_size = static_cast<std::uint32_t>(block_size);
    options.frame_id = frame_id;
    options.max_map_bytes = static_cast<std::size_t>(max_bytes);
    const auto map = point_input ? convertPointCloud(path, options)
                                 : readHdf5(path, options.max_map_bytes);
    snapshot_.header.frame_id = map.metadata().frame_id;
    snapshot_.header.stamp = now();
    snapshot_.format_version = 1;
    snapshot_.voxel_size = map.metadata().voxel_size;
    snapshot_.origin = map.metadata().origin;
    snapshot_.dimensions = map.metadata().dimensions;
    snapshot_.block_size = map.metadata().block_size;
    snapshot_.block_dimensions = map.blockDimensions();
    snapshot_.dictionary_size = static_cast<std::uint32_t>(map.dictionarySize());
    snapshot_.dictionary = map.dictionary();
    snapshot_.indices = map.indices();

    sensor_msgs::msg::PointCloud2 cloud;
    if (visualize) {
      const auto centers = map.occupiedCenters(
          lower, upper, static_cast<std::size_t>(max_points));
      cloud.header = snapshot_.header;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(centers.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      std::size_t count = 0;
      for (const auto &center : centers) {
        const auto representable = [](double value) {
          return std::isfinite(value) &&
                 std::abs(value) <= std::numeric_limits<float>::max();
        };
        if (!std::all_of(center.begin(), center.end(), representable)) {
          continue;
        }
        *x = static_cast<float>(center[0]);
        *y = static_cast<float>(center[1]);
        *z = static_cast<float>(center[2]);
        ++x;
        ++y;
        ++z;
        ++count;
      }
      modifier.resize(count);
      cloud.is_dense = true;
    }
    if (point_input && !converted_path.empty()) {
      writeHdf5(converted_path, map);
    }
    // Endpoints are created only after loading, validation, and optional saving succeed.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<vq_map_server::msg::VqMap3D>("~/map", qos);
    service_ = create_service<vq_map_server::srv::GetVqMap3D>(
        "~/get_map",
        [this](const std::shared_ptr<vq_map_server::srv::GetVqMap3D::Request>,
               std::shared_ptr<vq_map_server::srv::GetVqMap3D::Response> response) {
          response->map = snapshot_;
        });
    if (visualize) {
      cloud_publisher_ =
          create_publisher<sensor_msgs::msg::PointCloud2>("~/occupied_voxels", qos);
      cloud_publisher_->publish(cloud);
    }
    publisher_->publish(snapshot_);
    RCLCPP_INFO(get_logger(), "Map ready: %zu blocks, %zu patterns, %zu payload bytes",
                map.indices().size(), map.dictionarySize(),
                map.dictionary().size() + map.indices().size() * sizeof(std::uint32_t));
  }

private:
  vq_map_server::msg::VqMap3D snapshot_;
  rclcpp::Publisher<vq_map_server::msg::VqMap3D>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Service<vq_map_server::srv::GetVqMap3D>::SharedPtr service_;
};
}  // namespace vq_map_server::map3d

int main(int argc, char **argv) {
  int result = 0;
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vq_map_server::map3d::VqMapServer>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("vq_map_server_3d"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
