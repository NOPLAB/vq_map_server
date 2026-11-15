// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_converter.hpp"
#include "vq_map_server/msg/vq_map2_d.hpp"
#include "vq_map_server/srv/get_vq_map2_d.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace vq_map_server::map2d {

class VqMapServer final : public rclcpp::Node {
public:
  VqMapServer() : Node("vq_map_server_2d") {
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    const auto path = declare_parameter<std::string>("map_path", "", immutable);
    const auto converted_path =
        declare_parameter<std::string>("converted_map_path", "", immutable);
    const auto block_size =
        declare_parameter<std::int64_t>("block_size", 24, immutable);
    const auto frame_id = declare_parameter<std::string>("frame_id", "map", immutable);
    const auto encoding =
        declare_parameter<std::string>("cell_encoding", "ternary", immutable);
    const auto unknown =
        declare_parameter<std::string>("unknown_policy", "reject", immutable);
    const auto max_bytes =
        declare_parameter<std::int64_t>("max_map_bytes", 1073741824, immutable);
    const auto expand =
        declare_parameter<bool>("publish_occupancy_grid", false, immutable);
    const auto max_cells =
        declare_parameter<std::int64_t>("max_occupancy_grid_cells", 1000000, immutable);

    if (path.empty()) {
      throw std::invalid_argument("map_path is required");
    }
    if (block_size <= 0 || block_size > 64) {
      throw std::invalid_argument("block_size must be in [1, 64]");
    }
    if (max_bytes <= 0 ||
        static_cast<std::uint64_t>(max_bytes) > std::numeric_limits<std::size_t>::max()) {
      throw std::invalid_argument("max_map_bytes must fit a positive size_t");
    }
    if (max_cells <= 0 || max_cells > 100000000) {
      throw std::invalid_argument("max_occupancy_grid_cells must be in [1, 100000000]");
    }
    ConversionOptions options;
    options.block_size = static_cast<std::uint32_t>(block_size);
    options.frame_id = frame_id;
    options.max_map_bytes = static_cast<std::size_t>(max_bytes);
    if (encoding == "ternary") {
      options.encoding = CellEncoding::Ternary;
    } else if (encoding == "binary") {
      options.encoding = CellEncoding::Binary;
    } else {
      throw std::invalid_argument("cell_encoding must be ternary or binary");
    }
    if (unknown == "reject") {
      options.unknown_policy = UnknownPolicy::Reject;
    } else if (unknown == "free") {
      options.unknown_policy = UnknownPolicy::Free;
    } else if (unknown == "occupied") {
      options.unknown_policy = UnknownPolicy::Occupied;
    } else {
      throw std::invalid_argument("unknown_policy must be reject, free, or occupied");
    }
    auto extension = std::filesystem::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool yaml_input = extension == ".yaml" || extension == ".yml";
    if (!yaml_input && extension != ".h5" && extension != ".hdf5") {
      throw std::invalid_argument("map_path must be YAML, YML, H5, or HDF5");
    }
    if (!converted_path.empty()) {
      if (!yaml_input) {
        throw std::invalid_argument("converted_map_path is only valid for YAML input");
      }
      auto output_extension = std::filesystem::path(converted_path).extension().string();
      std::transform(output_extension.begin(), output_extension.end(),
                     output_extension.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (output_extension != ".h5" && output_extension != ".hdf5") {
        throw std::invalid_argument("converted_map_path must be H5 or HDF5");
      }
    }

    nav_msgs::msg::OccupancyGrid grid;
    {
      const auto map = yaml_input ? convertOccupancyGrid(path, options)
                                  : readHdf5(path, options.max_map_bytes);
      if (expand && map.cellCount() > static_cast<std::uint64_t>(max_cells)) {
        throw std::invalid_argument("map exceeds max_occupancy_grid_cells");
      }
      snapshot_.header.frame_id = map.metadata().frame_id;
      snapshot_.header.stamp = now();
      snapshot_.format_version = 1;
      snapshot_.resolution = map.metadata().resolution;
      snapshot_.origin = map.metadata().origin;
      snapshot_.yaw = map.metadata().yaw;
      snapshot_.dimensions = map.metadata().dimensions;
      snapshot_.block_size = map.metadata().block_size;
      snapshot_.block_dimensions = map.blockDimensions();
      snapshot_.cell_encoding = static_cast<std::uint8_t>(map.metadata().encoding);
      snapshot_.index_bit_width = map.indexBitWidth();
      snapshot_.dictionary_size = static_cast<std::uint32_t>(map.dictionarySize());
      snapshot_.dictionary = map.dictionary();
      snapshot_.indices = map.indices();
      if (expand) {
        grid.header = snapshot_.header;
        grid.info.map_load_time = snapshot_.header.stamp;
        grid.info.resolution = snapshot_.resolution;
        grid.info.width = snapshot_.dimensions[0];
        grid.info.height = snapshot_.dimensions[1];
        grid.info.origin.position.x = snapshot_.origin[0];
        grid.info.origin.position.y = snapshot_.origin[1];
        grid.info.origin.orientation.z = std::sin(snapshot_.yaw / 2.0);
        grid.info.origin.orientation.w = std::cos(snapshot_.yaw / 2.0);
        grid.data.resize(static_cast<std::size_t>(map.cellCount()));
        for (std::uint32_t y = 0; y < grid.info.height; ++y) {
          for (std::uint32_t x = 0; x < grid.info.width; ++x) {
            const auto state = map.at({x, y});
            grid.data[static_cast<std::size_t>(y) * grid.info.width + x] =
                state == CellState::Free ? 0 : state == CellState::Occupied ? 100 : -1;
          }
        }
      }
      if (!converted_path.empty()) {
        writeHdf5(converted_path, map);
      }
    }
    // Loading, expansion, and exclusive saving must succeed before advertising.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<vq_map_server::msg::VqMap2D>("~/map", qos);
    service_ = create_service<vq_map_server::srv::GetVqMap2D>(
        "~/get_map",
        [this](const std::shared_ptr<vq_map_server::srv::GetVqMap2D::Request>,
               std::shared_ptr<vq_map_server::srv::GetVqMap2D::Response> response) {
          response->map = snapshot_;
        });
    if (expand) {
      grid_publisher_ =
          create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid", qos);
      grid_publisher_->publish(grid);
    }
    publisher_->publish(snapshot_);
    const auto blocks = static_cast<std::size_t>(snapshot_.block_dimensions[0]) *
                        snapshot_.block_dimensions[1];
    RCLCPP_INFO(get_logger(), "Map ready: %zu blocks, %zu patterns, %zu payload bytes",
                blocks, static_cast<std::size_t>(snapshot_.dictionary_size),
                snapshot_.dictionary.size() + snapshot_.indices.size());
  }

private:
  vq_map_server::msg::VqMap2D snapshot_;
  rclcpp::Publisher<vq_map_server::msg::VqMap2D>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Service<vq_map_server::srv::GetVqMap2D>::SharedPtr service_;
};
}  // namespace vq_map_server::map2d

int main(int argc, char **argv) {
  int result = 0;
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vq_map_server::map2d::VqMapServer>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("vq_map_server_2d"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
