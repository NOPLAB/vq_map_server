// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "ros_fixture.hpp"

namespace {
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
  EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server_3d/occupied_voxels"), 0U);
  ASSERT_TRUE(std::filesystem::exists(saved));
  const auto disk = vq_map_server::map3d::readHdf5(saved.string());
  EXPECT_EQ(disk.metadata().frame_id, "test_map");
  EXPECT_EQ(disk.dictionary(), map->dictionary);
  EXPECT_EQ(disk.indices(), map->indices);
}

TEST_F(RosNodeTest, Hdf5MetadataOverridesConversionParameters) {
  vq_map_server::map3d::MapMetadata metadata;
  metadata.frame_id = "stored_frame";
  metadata.voxel_size = 0.5F;
  metadata.origin = {-2.0F, 3.0F, 4.0F};
  metadata.dimensions = {2, 2, 1};
  metadata.block_size = 2;
  const auto path = directory_ / "input.hdf5";
  vq_map_server::map3d::writeHdf5(path.string(), vq_map_server::map3d::VqMap(metadata, {9}, {0}));
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
    EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server_3d/map"), 0U);
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
  EXPECT_EQ(node_->count_publishers(namespace_ + "/vq_map_server_3d/map"), 0U);
  std::ifstream file(output);
  const std::string contents((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "keep existing contents");
}
}  // namespace
