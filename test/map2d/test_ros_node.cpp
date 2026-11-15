// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "ros_fixture.hpp"

namespace {
TEST_F(RosNodeTest, LateCompressedSubscriberAndServiceReceiveIdenticalTernarySnapshot) {
  startYaml({"max_occupancy_grid_cells:=1"});
  ASSERT_TRUE(ready());
  // Creating the subscription after readiness exercises retained DDS delivery.
  const auto map = retained<Map>("map");
  ASSERT_NE(map, nullptr);
  expectYamlMap(*map);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->map, *map);
  EXPECT_EQ(node_->count_publishers(endpoint("occupancy_grid")), 0U);
}

TEST_F(RosNodeTest, DefaultYamlStartupPublishesOnlyCompressedMapWithDefaultMetadata) {
  start({"map_path:=" + yaml_path_.string()});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(map->header.frame_id, "map");
  EXPECT_EQ(map->block_size, 24U);
  EXPECT_EQ(map->cell_encoding, Map::TERNARY);
  EXPECT_EQ(map->dimensions, (std::array<std::uint32_t, 2>{3, 2}));
  EXPECT_EQ(node_->count_publishers(endpoint("occupancy_grid")), 0U);
}

TEST_F(RosNodeTest, ExplicitExpansionPublishesCompleteRetainedGridWithSharedHeaderAndPose) {
  startYaml({"publish_occupancy_grid:=true", "max_occupancy_grid_cells:=6"});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  const auto grid = retained<Grid>("occupancy_grid");
  ASSERT_NE(map, nullptr);
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(grid->header, map->header);
  EXPECT_EQ(grid->info.map_load_time, map->header.stamp);
  EXPECT_FLOAT_EQ(grid->info.resolution, 0.25F);
  EXPECT_EQ(grid->info.width, 3U);
  EXPECT_EQ(grid->info.height, 2U);
  EXPECT_DOUBLE_EQ(grid->info.origin.position.x, -2.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.position.y, 3.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.position.z, 0.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.z, std::sin(0.35));
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.w, std::cos(0.35));
  EXPECT_EQ(grid->data, (std::vector<std::int8_t>{0, -1, 100, 100, 0, -1}));
}

TEST_F(RosNodeTest, YamlStartupSavesExclusiveHdf5ThatReloadsWithAuthoritativeMetadata) {
  const auto saved = directory_ / "converted.hdf5";
  startYaml({"converted_map_path:=" + saved.string()});
  ASSERT_TRUE(ready());
  const auto original = retained<Map>("map");
  ASSERT_NE(original, nullptr);
  expectYamlMap(*original);
  ASSERT_TRUE(std::filesystem::exists(saved));
  server_.reset();
  // A new namespace avoids depending on DDS removal propagation for the old process.
  namespace_ += "_reload";
  client_ = node_->create_client<GetMap>(endpoint("get_map"));
  start({"map_path:=" + saved.string(), "frame_id:=ignored", "block_size:=24",
         "cell_encoding:=binary", "unknown_policy:=occupied"});
  ASSERT_TRUE(ready());
  const auto loaded = retained<Map>("map");
  ASSERT_NE(loaded, nullptr);
  expectYamlMap(*loaded);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->map, *loaded);
}

TEST_F(RosNodeTest, BinaryUnknownPolicyIsAppliedByYamlStartup) {
  startYaml({"cell_encoding:=binary", "unknown_policy:=occupied",
             "publish_occupancy_grid:=true"});
  ASSERT_TRUE(ready());
  const auto map = retained<Map>("map");
  const auto grid = retained<Grid>("occupancy_grid");
  ASSERT_NE(map, nullptr);
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(map->cell_encoding, Map::BINARY);
  EXPECT_EQ(grid->data, (std::vector<std::int8_t>{0, 100, 100, 100, 0, 100}));
}

TEST_F(RosNodeTest, StartupFailuresExposeNeitherCompressedNorExpandedEndpoints) {
  const auto bad_yaml = directory_ / "bad.yaml";
  std::ofstream(bad_yaml) << "image: missing.pgm\n";
  const auto bad_hdf5 = directory_ / "bad.h5";
  std::ofstream(bad_hdf5) << "not HDF5";
  const auto valid_hdf5 = directory_ / "valid.h5";
  vq_map_server::map2d::MapMetadata metadata;
  metadata.dimensions = {3, 2};
  metadata.block_size = 2;
  vq_map_server::map2d::writeHdf5(
      valid_hdf5.string(),
      vq_map_server::map2d::VqMap(metadata, {0x18, 0x21}, {0, 1}, 8));
  const std::vector<std::vector<std::string>> cases{
      {},
      {"map_path:=" + (directory_ / "unsupported.xyz").string()},
      {"map_path:=" + (directory_ / "missing.yaml").string()},
      {"map_path:=" + bad_yaml.string()},
      {"map_path:=" + bad_hdf5.string()},
      {"map_path:=" + yaml_path_.string(), "block_size:=0"},
      {"map_path:=" + yaml_path_.string(), "block_size:=65"},
      {"map_path:=" + yaml_path_.string(), "block_size:=4294967296"},
      {"map_path:=" + yaml_path_.string(), "cell_encoding:=invalid"},
      {"map_path:=" + yaml_path_.string(), "unknown_policy:=invalid"},
      {"map_path:=" + yaml_path_.string(), "cell_encoding:=binary"},
      {"map_path:=" + yaml_path_.string(), "max_map_bytes:=0"},
      {"map_path:=" + yaml_path_.string(), "max_map_bytes:=1"},
      {"map_path:=" + yaml_path_.string(), "max_occupancy_grid_cells:=0"},
      {"map_path:=" + yaml_path_.string(), "max_occupancy_grid_cells:=100000001"},
      {"map_path:=" + yaml_path_.string(), "publish_occupancy_grid:=true",
       "max_occupancy_grid_cells:=5"},
      {"map_path:=" + valid_hdf5.string(), "publish_occupancy_grid:=true",
       "max_occupancy_grid_cells:=5"},
      {"map_path:=" + yaml_path_.string(),
       "converted_map_path:=" + (directory_ / "missing" / "output.h5").string()},
      {"map_path:=" + yaml_path_.string(),
       "converted_map_path:=" + (directory_ / "output.yaml").string()},
      {"map_path:=" + bad_hdf5.string(),
       "converted_map_path:=" + (directory_ / "output.h5").string()}};
  for (const auto &parameters : cases) {
    SCOPED_TRACE(::testing::PrintToString(parameters));
    start(parameters);
    ASSERT_TRUE(server_->waitForFailure());
    expectNoEndpoints();
    server_.reset();
  }
}

TEST_F(RosNodeTest, FailedExclusiveSavePreservesExistingFileAndExposesNoEndpoints) {
  const auto output = directory_ / "existing.h5";
  {
    std::ofstream file(output);
    file << "keep existing contents";
  }
  startYaml({"converted_map_path:=" + output.string(), "publish_occupancy_grid:=true"});
  ASSERT_TRUE(server_->waitForFailure());
  expectNoEndpoints();
  std::ifstream file(output);
  const std::string contents((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "keep existing contents");
}
}  // namespace
