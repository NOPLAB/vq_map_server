// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map_io.hpp"
#include "vq_map_server/msg/vq_map.hpp"
#include "vq_map_server/srv/get_vq_map.hpp"

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
using Map = vq_map_server::msg::VqMap;
using GetMap = vq_map_server::srv::GetVqMap;
using Cloud = sensor_msgs::msg::PointCloud2;

// The child performs only async-signal-safe operations between fork and exec.
class ServerProcess {
public:
  explicit ServerProcess(const std::vector<std::string> &arguments) {
    std::vector<std::string> command{TEST_SERVER_PATH, "--ros-args"};
    command.insert(command.end(), arguments.begin(), arguments.end());
    std::vector<char *> argv;
    for (auto &part : command) {
      argv.push_back(part.data());
    }
    argv.push_back(nullptr);
    pid_ = fork();
    if (pid_ < 0) {
      throw std::runtime_error("fork failed");
    }
    if (pid_ == 0) {
      execv(argv[0], argv.data());
      _exit(127);
    }
  }
  ServerProcess(const ServerProcess &) = delete;
  ServerProcess &operator=(const ServerProcess &) = delete;
  ~ServerProcess() {
    if (pid_ > 0) {
      kill(pid_, SIGKILL);
      while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
    }
  }

  bool waitForFailure() {
    // pidfd makes process termination an event rather than a sleep/poll loop.
    const int fd = static_cast<int>(syscall(SYS_pidfd_open, pid_, 0));
    if (fd < 0) {
      throw std::runtime_error("pidfd_open failed");
    }
    pollfd event{fd, POLLIN, 0};
    int ready;
    do {
      ready = poll(&event, 1, 10000);
    } while (ready < 0 && errno == EINTR);
    close(fd);
    if (ready != 1) {
      return false;
    }
    int status = 0;
    pid_t reaped;
    do {
      reaped = waitpid(pid_, &status, 0);
    } while (reaped < 0 && errno == EINTR);
    if (reaped != pid_) {
      return false;
    }
    pid_ = -1;
    return WIFEXITED(status) && WEXITSTATUS(status) != 0;
  }

private:
  pid_t pid_ = -1;
};

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
    namespace_ = "/vq_test_" + std::to_string(getpid()) + "_" +
                 std::to_string(++sequence);
    node_ = std::make_shared<rclcpp::Node>("test_client", namespace_);
    executor_.add_node(node_);
    client_ = node_->create_client<GetMap>(namespace_ + "/vq_map_server/get_map");
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
        namespace_ + "/vq_map_server/" + suffix,
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
    vq_map_server::MapMetadata metadata;
    metadata.frame_id = map.header.frame_id;
    metadata.voxel_size = map.voxel_size;
    metadata.origin = map.origin;
    metadata.dimensions = map.dimensions;
    metadata.block_size = map.block_size;
    const vq_map_server::VqMap decoded(metadata, map.dictionary, map.indices);
    EXPECT_EQ(decoded.at({0, 0, 0}), vq_map_server::CellState::Occupied);
    EXPECT_EQ(decoded.at({1, 0, 0}), vq_map_server::CellState::Occupied);
    EXPECT_EQ(decoded.at({0, 1, 0}), vq_map_server::CellState::Free);
    EXPECT_EQ(decoded.at({1, 1, 0}), vq_map_server::CellState::Occupied);
  }

  std::filesystem::path directory_, point_path_;
  std::string namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<GetMap>::SharedPtr client_;
  std::unique_ptr<ServerProcess> server_;
};

TEST_F(RosNodeTest, PointCloudRetainedSnapshotMatchesServiceAndSavesHdf5) {
  const auto saved = directory_ / "converted.h5";
  startPointCloud({"converted_map_path:=" + saved.string()});
  ASSERT_TRUE(ready());
  // Subscription is intentionally created after the service becomes available.
  const auto map = retained<Map>("map");
  ASSERT_NE(map, nullptr);
  expectPointMap(*map);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->map, *map);  // Includes the original stamp.
  EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server/occupied_voxels"), 0U);
  ASSERT_TRUE(std::filesystem::exists(saved));
  const auto disk = vq_map_server::readHdf5(saved.string());
  EXPECT_EQ(disk.metadata().frame_id, "test_map");
  EXPECT_EQ(disk.dictionary(), map->dictionary);
  EXPECT_EQ(disk.indices(), map->indices);
}

TEST_F(RosNodeTest, Hdf5MetadataOverridesConversionParameters) {
  vq_map_server::MapMetadata metadata;
  metadata.frame_id = "stored_frame";
  metadata.voxel_size = 0.5F;
  metadata.origin = {-2.0F, 3.0F, 4.0F};
  metadata.dimensions = {2, 2, 1};
  metadata.block_size = 2;
  const auto path = directory_ / "input.hdf5";
  vq_map_server::writeHdf5(path.string(), vq_map_server::VqMap(metadata, {9}, {0}));
  start({"map_path:=" + path.string(), "voxel_size:=4.0",
         "block_size:=13", "frame_id:=ignored"});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(map->header.frame_id, metadata.frame_id);
  EXPECT_EQ(map->voxel_size, metadata.voxel_size);
  EXPECT_EQ(map->origin, metadata.origin);
  EXPECT_EQ(map->dimensions, metadata.dimensions);
  EXPECT_EQ(map->block_size, metadata.block_size);
  EXPECT_EQ(map->dictionary, (std::vector<std::uint8_t>{9}));
  EXPECT_EQ(map->indices, (std::vector<std::uint32_t>{0}));
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->map, *map);
}

TEST_F(RosNodeTest, PlyInputUsesTheSameConversionProtocol) {
  const auto path = directory_ / "points.ply";
  {
    std::ofstream points(path);
    points << "ply\nformat ascii 1.0\nelement vertex 3\n"
              "property float x\nproperty float y\nproperty float z\n"
              "end_header\n0 0 0\n1 0 0\n1 1 0\n";
  }
  start({"map_path:=" + path.string(), "voxel_size:=1.0",
         "block_size:=2", "frame_id:=test_map"});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  ASSERT_NE(map, nullptr);
  expectPointMap(*map);
}

TEST_F(RosNodeTest, VisualizationHonorsHalfOpenBoundsAndSharesHeader) {
  startPointCloud({"publish_occupied_voxels:=true",
                  "visualization_min:=[0.0, 0.0, 0.0]",
                  "visualization_max:=[1.0, 2.0, 1.0]"});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  const auto cloud = retained<Cloud>("occupied_voxels");
  ASSERT_NE(map, nullptr);
  ASSERT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->header, map->header);
  ASSERT_EQ(cloud->width * cloud->height, 1U);
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"),
      z(*cloud, "z");
  EXPECT_FLOAT_EQ(*x, 0.5F);
  EXPECT_FLOAT_EQ(*y, 0.5F);
  EXPECT_FLOAT_EQ(*z, 0.5F);
  EXPECT_TRUE(cloud->is_dense);
}

TEST_F(RosNodeTest, VisualizationHonorsPointCap) {
  startPointCloud({"publish_occupied_voxels:=true", "max_visualization_points:=2"});
  ASSERT_TRUE(ready());
  const auto cloud = retained<Cloud>("occupied_voxels");
  ASSERT_NE(cloud, nullptr);
  ASSERT_EQ(cloud->width * cloud->height, 2U);
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"),
      z(*cloud, "z");
  for (unsigned i = 0; i < 2; ++i, ++x, ++y, ++z) {
    EXPECT_TRUE(std::isfinite(*x));
    EXPECT_TRUE(std::isfinite(*y));
    EXPECT_TRUE(std::isfinite(*z));
    EXPECT_GE(*x, 0.0F);
    EXPECT_LT(*x, 2.0F);
  }
}

TEST_F(RosNodeTest, InvalidInputFailsWithoutExposingMapEndpoints) {
  const auto bad = directory_ / "bad.pcd";
  std::ofstream(bad) << "not a pointcloud\n";
  const auto empty = directory_ / "empty.pcd";
  std::ofstream(empty) << "";
  const std::vector<std::vector<std::string>> cases{
      {},
      {"map_path:=" + (directory_ / "unsupported.xyz").string()},
      {"map_path:=" + (directory_ / "missing.pcd").string()},
      {"map_path:=" + bad.string()},
      {"map_path:=" + empty.string()},
      {"map_path:=" + point_path_.string(), "voxel_size:=0.0"},
      {"map_path:=" + point_path_.string(), "voxel_size:=1.0e-100"},
      {"map_path:=" + point_path_.string(), "voxel_size:=1.0e100"},
      {"map_path:=" + point_path_.string(), "block_size:=-1"},
      {"map_path:=" + point_path_.string(), "block_size:=65"},
      {"map_path:=" + point_path_.string(), "block_size:=4294967296"},
      {"map_path:=" + point_path_.string(), "max_map_bytes:=-1"},
      {"map_path:=" + point_path_.string(), "max_visualization_points:=0"},
      {"map_path:=" + point_path_.string(), "max_visualization_points:=1000001"},
      {"map_path:=" + point_path_.string(), "visualization_min:=[0.0, 0.0]"},
      {"map_path:=" + point_path_.string(), "visualization_max:=[-20.0, 10.0, 3.0]"},
      {"map_path:=" + (directory_ / "missing.h5").string(),
       "converted_map_path:=" + (directory_ / "output.h5").string()}};
  for (const auto &parameters : cases) {
    SCOPED_TRACE(parameters.empty() ? "missing map_path" : parameters.front());
    start(parameters);
    ASSERT_TRUE(server_->waitForFailure());
    EXPECT_FALSE(client_->wait_for_service(100ms));
    EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server/map"), 0U);
    server_.reset();
  }
}

TEST_F(RosNodeTest, ExistingConversionOutputIsNeverOverwritten) {
  const auto output = directory_ / "existing.h5";
  {
    std::ofstream file(output);
    file << "keep existing contents";
  }
  startPointCloud({"converted_map_path:=" + output.string()});
  ASSERT_TRUE(server_->waitForFailure());
  EXPECT_FALSE(client_->wait_for_service(100ms));
  EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server/map"), 0U);
  std::ifstream file(output);
  const std::string contents((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "keep existing contents");
}
}  // namespace
