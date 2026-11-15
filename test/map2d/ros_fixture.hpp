// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "../common/server_process.hpp"
#include "vq_map_server/map2d/map_io.hpp"
#include "vq_map_server/msg/vq_map2_d.hpp"
#include "vq_map_server/srv/get_vq_map2_d.hpp"

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <poll.h>
#include <signal.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace std::chrono_literals;
using Map = vq_map_server::msg::VqMap2D;
using GetMap = vq_map_server::srv::GetVqMap2D;
using Grid = nav_msgs::msg::OccupancyGrid;

// Only async-signal-safe operations run between fork and exec.
class RosNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto previous = std::getenv("ROS_DOMAIN_ID");
    had_domain_ = previous != nullptr;
    previous_domain_ = previous ? previous : "";
    const auto domain = std::to_string(20 + getpid() % 180);
    ASSERT_EQ(setenv("ROS_DOMAIN_ID", domain.c_str(), 1), 0);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
    if (had_domain_) {
      setenv("ROS_DOMAIN_ID", previous_domain_.c_str(), 1);
    } else {
      unsetenv("ROS_DOMAIN_ID");
    }
  }

  void SetUp() override {
    char directory[] = "/tmp/vq_map_2d_ros_XXXXXX";
    const auto created = mkdtemp(directory);
    ASSERT_NE(created, nullptr);
    directory_ = created;
    static unsigned sequence = 0;
    namespace_ = "/vq_test_2d_" + std::to_string(getpid()) + "_" +
                 std::to_string(++sequence);
    node_ = std::make_shared<rclcpp::Node>("test_client", namespace_);
    executor_.add_node(node_);
    client_ = node_->create_client<GetMap>(endpoint("get_map"));
    yaml_path_ = directory_ / "map.yaml";
    {
      // Image rows are top-down; the delivered occupancy grid is bottom-up.
      std::ofstream image(directory_ / "map.pgm", std::ios::binary);
      image << "P5\n3 2\n255\n";
      const std::array<unsigned char, 6> pixels{0, 255, 205, 255, 205, 0};
      image.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
      ASSERT_TRUE(image.good());
    }
    {
      std::ofstream yaml(yaml_path_);
      yaml << "image: map.pgm\nresolution: 0.25\norigin: [-2.0, 3.0, 0.7]\n"
              "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
              "mode: trinary\n";
      ASSERT_TRUE(yaml.good());
    }
  }

  void TearDown() override {
    server_.reset();
    client_.reset();
    if (node_) {
      executor_.remove_node(node_);
      node_.reset();
    }
    std::filesystem::remove_all(directory_);
  }

  std::string endpoint(const std::string &suffix) const {
    return namespace_ + "/vq_map_server_2d/" + suffix;
  }

  void start(const std::vector<std::string> &parameters) {
    std::vector<std::string> arguments{"-r", "__ns:=" + namespace_};
    for (const auto &parameter : parameters) {
      arguments.push_back("-p");
      arguments.push_back(parameter);
    }
    server_ = std::make_unique<ServerProcess>(arguments);
  }

  void startYaml(const std::vector<std::string> &extra = {}) {
    std::vector<std::string> parameters{
        "map_path:=" + yaml_path_.string(), "block_size:=2", "frame_id:=test_map"};
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
        endpoint(suffix), rclcpp::QoS(1).reliable().transient_local(),
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

  void expectNoEndpoints() {
    EXPECT_FALSE(client_->wait_for_service(100ms));
    EXPECT_EQ(node_->count_publishers(endpoint("map")), 0U);
    EXPECT_EQ(node_->count_publishers(endpoint("occupancy_grid")), 0U);
  }

  void expectYamlMap(const Map &map) {
    EXPECT_EQ(map.format_version, 1U);
    EXPECT_EQ(map.header.frame_id, "test_map");
    EXPECT_FLOAT_EQ(map.resolution, 0.25F);
    EXPECT_EQ(map.origin, (std::array<double, 2>{-2.0, 3.0}));
    EXPECT_DOUBLE_EQ(map.yaw, 0.7);
    EXPECT_EQ(map.dimensions, (std::array<std::uint32_t, 2>{3, 2}));
    EXPECT_EQ(map.block_size, 2U);
    EXPECT_EQ(map.block_dimensions, (std::array<std::uint32_t, 2>{2, 1}));
    EXPECT_EQ(map.cell_encoding, Map::TERNARY);
    EXPECT_EQ(map.index_bit_width, 8U);
    // Two 2x2 ternary patterns, LSB-first, including free edge padding.
    EXPECT_EQ(map.dictionary_size, 2U);
    EXPECT_EQ(map.dictionary, (std::vector<std::uint8_t>{0x18, 0x21}));
    EXPECT_EQ(map.indices, (std::vector<std::uint8_t>{0, 1}));
  }

  inline static bool had_domain_ = false;
  inline static std::string previous_domain_;
  std::filesystem::path directory_, yaml_path_;
  std::string namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<GetMap>::SharedPtr client_;
  std::unique_ptr<ServerProcess> server_;
};

}  // namespace
