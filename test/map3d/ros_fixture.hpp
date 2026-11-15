// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "../common/server_process.hpp"
#include "vq_map_server/map3d/map_io.hpp"
#include "vq_map_server/msg/vq_map3_d.hpp"
#include "vq_map_server/srv/get_vq_map3_d.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <poll.h>
#include <signal.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace std::chrono_literals;
using Map = vq_map_server::msg::VqMap3D;
using GetMap = vq_map_server::srv::GetVqMap3D;
using Cloud = sensor_msgs::msg::PointCloud2;

// The child performs only async-signal-safe operations between fork and exec.
class RosNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    char directory[] = "/tmp/vq_map_ros_XXXXXX";
    const auto created = mkdtemp(directory);
    ASSERT_NE(created, nullptr);
    directory_ = created;
    static unsigned sequence = 0;
    namespace_ = "/vq_test_3d_" + std::to_string(getpid()) + "_" +
                 std::to_string(++sequence);
    node_ = std::make_shared<rclcpp::Node>("test_client", namespace_);
    executor_.add_node(node_);
    client_ = node_->create_client<GetMap>(namespace_ + "/vq_map_server_3d/get_map");
    point_path_ = directory_ / "points.pcd";
    std::ofstream points(point_path_);
    points << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
              "TYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\n"
              "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
              "0 0 0\n1 0 0\n1 1 0\n";
    ASSERT_TRUE(points.good());
  }

  void TearDown() override {
    server_.reset();
    client_.reset();
    executor_.remove_node(node_);
    node_.reset();
    std::filesystem::remove_all(directory_);
  }

  void start(std::vector<std::string> parameters) {
    std::vector<std::string> arguments{"-r", "__ns:=" + namespace_};
    for (const auto &parameter : parameters) {
      arguments.push_back("-p");
      arguments.push_back(parameter);
    }
    server_ = std::make_unique<ServerProcess>(arguments);
  }

  void startPointCloud(std::vector<std::string> extra = {}) {
    std::vector<std::string> parameters{
        "map_path:=" + point_path_.string(), "voxel_size:=1.0",
        "block_size:=2", "frame_id:=test_map"};
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    start(parameters);
  }

  bool ready() { return client_->wait_for_service(10s); }

  template <class Message>
  std::shared_ptr<Message> retained(const std::string &suffix) {
    std::promise<std::shared_ptr<Message>> promise;
    auto future = promise.get_future();
    bool received = false;
    auto subscription = node_->create_subscription<Message>(
        namespace_ + "/vq_map_server_3d/" + suffix,
        rclcpp::QoS(1).reliable().transient_local(),
        [&promise, &received](std::shared_ptr<Message> message) {
          if (!received) {
            received = true;
            promise.set_value(std::move(message));
          }
        });
    if (executor_.spin_until_future_complete(future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<GetMap::Response> request() {
    auto result = client_->async_send_request(std::make_shared<GetMap::Request>());
    if (executor_.spin_until_future_complete(result, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return result.get();
  }

  void expectPointMap(const Map &map) {
    EXPECT_EQ(map.format_version, 1U);
    EXPECT_EQ(map.header.frame_id, "test_map");
    EXPECT_FLOAT_EQ(map.voxel_size, 1.0F);
    EXPECT_EQ(map.origin, (std::array<float, 3>{0, 0, 0}));
    EXPECT_EQ(map.dimensions, (std::array<std::uint32_t, 3>{2, 2, 1}));
    EXPECT_EQ(map.block_size, 2U);
    EXPECT_EQ(map.block_dimensions, (std::array<std::uint32_t, 3>{1, 1, 1}));
    EXPECT_EQ(map.dictionary_size, 1U);
    EXPECT_EQ(map.dictionary, (std::vector<std::uint8_t>{0x0b}));
    EXPECT_EQ(map.indices, (std::vector<std::uint32_t>{0}));
    vq_map_server::map3d::MapMetadata metadata;
    metadata.frame_id = map.header.frame_id;
    metadata.voxel_size = map.voxel_size;
    metadata.origin = map.origin;
    metadata.dimensions = map.dimensions;
    metadata.block_size = map.block_size;
    const vq_map_server::map3d::VqMap decoded(metadata, map.dictionary, map.indices);
    EXPECT_EQ(decoded.at({0, 0, 0}), vq_map_server::map3d::CellState::Occupied);
    EXPECT_EQ(decoded.at({1, 0, 0}), vq_map_server::map3d::CellState::Occupied);
    EXPECT_EQ(decoded.at({0, 1, 0}), vq_map_server::map3d::CellState::Free);
    EXPECT_EQ(decoded.at({1, 1, 0}), vq_map_server::map3d::CellState::Occupied);
  }

  std::filesystem::path directory_, point_path_;
  std::string namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<GetMap>::SharedPtr client_;
  std::unique_ptr<ServerProcess> server_;
};

}  // namespace
