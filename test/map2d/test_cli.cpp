// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "cli_fixture.hpp"

namespace vq_map_server::test2d {
TEST_F(CliTest, HelpUsesExplicit2dNamesAndRejectsMissingOrUnknownArguments) {
  const auto convert_help = convert({"--help"});
  ASSERT_EQ(convert_help.status, 0) << convert_help.err;
  EXPECT_NE(convert_help.out.find("Usage: vq_map_convert_2d"), std::string::npos);
  const auto inspect_help = inspect({"--help"});
  ASSERT_EQ(inspect_help.status, 0) << inspect_help.err;
  EXPECT_NE(inspect_help.out.find("Usage: vq_map_inspect_2d"), std::string::npos);
  for (const auto &arguments : std::vector<std::vector<std::string>>{
           {}, {"--input"}, {"--input", input}, {"--unknown", "1"},
           {"--input", input, "--output"}, {"--input", "", "--output", output},
           {"--input", input, "--output", ""},
           {"--input", input, "--output", output, "--block-size", "--frame-id", "map"},
           {"--input", input, "--output", output, "--resolution", "1"},
           {"--input", input, "--output", output, "--voxel-size", "1"}}) {
    expectFailure(convert(arguments), "vq_map_convert_2d");
    EXPECT_FALSE(std::filesystem::exists(output));
  }
  for (const auto &arguments : std::vector<std::vector<std::string>>{
           {}, {"--queries"}, {"--bad"}, {"absent.h5"}, {"a.h5", "b.h5"}, {""}}) {
    expectFailure(inspect(arguments), "vq_map_inspect_2d");
  }
}

TEST_F(CliTest, RoundTripPreservesYamlGeometryAndPrintsMetadataAndRepeatableQueries) {
  const auto converted = convert({"--input", input, "--output", output,
                                  "--block-size", "2", "--frame-id", "survey"});
  ASSERT_EQ(converted.status, 0) << converted.err;
  EXPECT_EQ(value(converted.out, "output"), output);
  for (const auto &name : {"memory_bytes", "peak_rss_kib", "elapsed_ms"})
    EXPECT_FALSE(value(converted.out, name).empty()) << converted.out;
  const auto map = readHdf5(output);
  EXPECT_EQ(map.metadata().frame_id, "survey");
  EXPECT_FLOAT_EQ(map.metadata().resolution, 0.25F);
  EXPECT_EQ(map.metadata().origin, (std::array<double, 2>{-2, 3}));
  EXPECT_DOUBLE_EQ(map.metadata().yaw, 0.7);
  EXPECT_EQ(map.metadata().dimensions, (std::array<std::uint32_t, 2>{3, 2}));
  EXPECT_EQ(map.metadata().encoding, CellEncoding::Ternary);
  EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{0x18, 0x21}));
  EXPECT_EQ(map.indices(), (std::vector<std::uint8_t>{0, 1}));
  EXPECT_EQ(map.at({1, 0}), CellState::Unknown);
  EXPECT_EQ(map.at({2, 0}), CellState::Occupied);

  const auto result = inspect({output, "--queries", "1000"});
  ASSERT_EQ(result.status, 0) << result.err;
  EXPECT_EQ(value(result.out, "frame_id"), "survey");
  EXPECT_EQ(value(result.out, "resolution"), "0.25");
  EXPECT_EQ(value(result.out, "origin"), "-2 3");
  EXPECT_EQ(value(result.out, "yaw"), "0.7");
  EXPECT_EQ(value(result.out, "dimensions"), "3 2");
  EXPECT_EQ(value(result.out, "block_size"), "2");
  EXPECT_EQ(value(result.out, "cell_encoding"), "ternary");
  EXPECT_EQ(value(result.out, "index_bit_width"), "8");
  EXPECT_EQ(value(result.out, "blocks"), "2");
  EXPECT_EQ(value(result.out, "dictionary_patterns"), "2");
  EXPECT_EQ(value(result.out, "cell_count"), "6");
  EXPECT_EQ(value(result.out, "queries"), "1000");
  for (const auto &name : {"memory_bytes", "peak_rss_kib", "elapsed_ms", "query_elapsed_ms"})
    EXPECT_FALSE(value(result.out, name).empty()) << result.out;
  ASSERT_FALSE(value(result.out, "query_occupied").empty());
  ASSERT_FALSE(value(result.out, "query_unknown").empty());
  const auto again = inspect({"--queries", "1000", "--max-map-bytes", "1024", output});
  ASSERT_EQ(again.status, 0) << again.err;
  EXPECT_EQ(value(again.out, "query_occupied"), value(result.out, "query_occupied"));
  EXPECT_EQ(value(again.out, "query_unknown"), value(result.out, "query_unknown"));
  const auto zero = inspect({output, "--queries", "0"});
  ASSERT_EQ(zero.status, 0) << zero.err;
  EXPECT_EQ(value(zero.out, "query_occupied"), "0");
  EXPECT_EQ(value(zero.out, "query_unknown"), "0");
  const auto metadata_only = inspect({output});
  ASSERT_EQ(metadata_only.status, 0) << metadata_only.err;
  EXPECT_TRUE(value(metadata_only.out, "queries").empty());
  expectFailure(inspect({output, "--max-map-bytes", "1"}), "vq_map_inspect_2d");
}

TEST_F(CliTest, ConfigAppliesBeforeFlagsRegardlessOfArgumentOrder) {
  std::ofstream(config) << "/**:\n  ros__parameters:\n"
                          "    block_size: 3\n    frame_id: config_frame\n"
                          "    cell_encoding: binary\n    unknown_policy: free\n"
                          "    max_map_bytes: 1024\n";
  auto result = convert({"--input", input, "--output", output, "--config", config});
  ASSERT_EQ(result.status, 0) << result.err;
  auto map = readHdf5(output);
  EXPECT_EQ(map.metadata().block_size, 3U);
  EXPECT_EQ(map.metadata().frame_id, "config_frame");
  EXPECT_EQ(map.metadata().encoding, CellEncoding::Binary);
  EXPECT_EQ(map.at({1, 0}), CellState::Free);
  EXPECT_FLOAT_EQ(map.metadata().resolution, 0.25F);
  const auto before_config = (directory / "before_config.h5").string();
  result = convert({"--block-size", "2", "--frame-id", "flag_frame",
                    "--cell-encoding", "ternary", "--unknown-policy", "reject",
                    "--input", input, "--config", config, "--output", before_config});
  ASSERT_EQ(result.status, 0) << result.err;
  map = readHdf5(before_config);
  EXPECT_EQ(map.metadata().block_size, 2U);
  EXPECT_EQ(map.metadata().frame_id, "flag_frame");
  EXPECT_EQ(map.metadata().encoding, CellEncoding::Ternary);
  EXPECT_EQ(map.at({1, 0}), CellState::Unknown);
  const auto after_config = (directory / "after_config.h5").string();
  result = convert({"--input", input, "--config", config, "--output", after_config,
                    "--unknown-policy", "reject", "--cell-encoding", "ternary",
                    "--frame-id", "flag_frame", "--block-size", "2"});
  ASSERT_EQ(result.status, 0) << result.err;
  const auto again = readHdf5(after_config);
  EXPECT_EQ(again.dictionary(), map.dictionary());
  EXPECT_EQ(again.indices(), map.indices());
  EXPECT_EQ(again.metadata().frame_id, map.metadata().frame_id);
  std::ofstream(config) << "/**:\n  ros__parameters:\n    max_map_bytes: 1\n";
  const auto overridden_limit = (directory / "overridden_limit.h5").string();
  result = convert({"--max-map-bytes", "1024", "--input", input,
                    "--output", overridden_limit, "--config", config});
  ASSERT_EQ(result.status, 0) << result.err;
}

TEST_F(CliTest, BinaryRequiresAnExplicitReplacementForUnknownCells) {
  for (const auto &extra : std::vector<std::vector<std::string>>{
           {"--cell-encoding", "binary"},
           {"--cell-encoding", "binary", "--unknown-policy", "reject"},
           {"--cell-encoding", "ternary", "--unknown-policy", "free"},
           {"--unknown-policy", "occupied"}}) {
    std::vector<std::string> arguments{"--input", input, "--output", output};
    arguments.insert(arguments.end(), extra.begin(), extra.end());
    expectFailure(convert(arguments), "vq_map_convert_2d");
    EXPECT_FALSE(std::filesystem::exists(output));
  }
  for (const auto &policy : {"free", "occupied"}) {
    const auto path = (directory / (std::string(policy) + ".h5")).string();
    const auto result = convert({"--input", input, "--output", path, "--block-size", "2",
                                 "--cell-encoding", "binary", "--unknown-policy", policy});
    ASSERT_EQ(result.status, 0) << result.err;
    const auto map = readHdf5(path);
    EXPECT_EQ(map.metadata().encoding, CellEncoding::Binary);
    const auto expected = std::string(policy) == "free" ? CellState::Free : CellState::Occupied;
    EXPECT_EQ(map.at({1, 0}), expected);
    EXPECT_EQ(map.at({2, 1}), expected);
    EXPECT_EQ(map.at({0, 0}), CellState::Free);
    EXPECT_EQ(map.at({2, 0}), CellState::Occupied);
    const auto inspected = inspect({path, "--queries", "100"});
    ASSERT_EQ(inspected.status, 0) << inspected.err;
    EXPECT_EQ(value(inspected.out, "cell_encoding"), "binary");
    EXPECT_EQ(value(inspected.out, "query_unknown"), "0");
  }
}

TEST_F(CliTest, InvalidNumbersEnumsDuplicateFlagsAndMalformedConfigsFail) {
  for (const auto &option : {"--block-size", "--max-map-bytes"}) {
    for (const auto &number : {"0", "-1", "+1", "", "1junk", "1.5", "nan", "inf",
                               " 1", "1 ", "999999999999999999999999"}) {
      SCOPED_TRACE(std::string(option) + " " + number);
      expectFailure(convert({"--input", input, "--output", output, option, number}),
                    "vq_map_convert_2d");
      EXPECT_FALSE(std::filesystem::exists(output));
    }
  }
  for (const auto &extra : std::vector<std::vector<std::string>>{
           {"--block-size", "65"}, {"--block-size", "4294967296"},
           {"--max-map-bytes", "1"}, {"--cell-encoding", "invalid"},
           {"--unknown-policy", "invalid"}, {"--frame-id", ""},
           {"--frame-id", "has space"}}) {
    std::vector<std::string> arguments{"--input", input, "--output", output};
    arguments.insert(arguments.end(), extra.begin(), extra.end());
    expectFailure(convert(arguments), "vq_map_convert_2d");
    EXPECT_FALSE(std::filesystem::exists(output));
  }
  std::ofstream(config) << "/**:\n  ros__parameters: {}\n";
  for (const auto &option : std::vector<std::vector<std::string>>{
           {"--input", input}, {"--output", output}, {"--config", config},
           {"--block-size", "2"}, {"--frame-id", "map"},
           {"--cell-encoding", "ternary"}, {"--unknown-policy", "reject"},
           {"--max-map-bytes", "1024"}}) {
    std::vector<std::string> arguments{"--input", input, "--output", output};
    if (option[0] != "--input" && option[0] != "--output")
      arguments.insert(arguments.end(), option.begin(), option.end());
    arguments.insert(arguments.end(), option.begin(), option.end());
    expectFailure(convert(arguments), "vq_map_convert_2d");
    EXPECT_FALSE(std::filesystem::exists(output));
  }
  for (const auto &text : {
           "not: [valid", "", "block_size: 2\n", "[]",
           "/other:\n  ros__parameters: {}\n",
           "/**:\n  other: {}\n",
           "/**:\n  ros__parameters: []\n",
           "/**:\n  ros__parameters:\n    resolution: 0.5\n",
           "/**:\n  ros__parameters:\n    unknown: 1\n",
           "/**:\n  ros__parameters:\n    block_size: 1junk\n",
           "/**:\n  ros__parameters:\n    block_size: [1, 2]\n",
           "/**:\n  ros__parameters:\n    frame_id: null\n",
           "/**:\n  ros__parameters:\n    cell_encoding: other\n",
           "/**:\n  ros__parameters:\n    unknown_policy: other\n",
           "/**:\n  ros__parameters:\n    block_size: 2\n    block_size: 3\n",
           "/**:\n  ros__parameters: {}\n  ros__parameters: {}\n",
           "/**:\n  ros__parameters: {}\n/**:\n  ros__parameters: {}\n",
           "/**:\n  ros__parameters: {}\n  other: 1\n",
           "/**:\n  ros__parameters: {}\n/other: {}\n"}) {
    SCOPED_TRACE(text);
    std::ofstream(config) << text;
    expectFailure(convert({"--input", input, "--output", output, "--config", config}),
                  "vq_map_convert_2d");
    EXPECT_FALSE(std::filesystem::exists(output));
  }
  expectFailure(convert({"--input", input, "--output", output, "--config",
                         (directory / "absent.yaml").string()}), "vq_map_convert_2d");
  const auto malformed = (directory / "bad.yaml").string();
  std::ofstream(malformed) << "image: absent.pgm\n";
  expectFailure(convert({"--input", malformed, "--output", output}), "vq_map_convert_2d");
}

TEST_F(CliTest, ExclusiveCreatePreservesExistingBytesAndConvertedMap) {
  const std::string sentinel = "existing map must survive";
  std::ofstream(output) << sentinel;
  expectFailure(convert({"--input", input, "--output", output}), "vq_map_convert_2d");
  std::ifstream original(output);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(original), {}), sentinel);
  const auto other = (directory / "converted.h5").string();
  const auto converted = convert({"--input", input, "--output", other});
  ASSERT_EQ(converted.status, 0) << converted.err;
  const auto before = readHdf5(other);
  expectFailure(convert({"--input", input, "--output", other, "--block-size", "1"}),
                "vq_map_convert_2d");
  const auto after = readHdf5(other);
  EXPECT_EQ(after.dictionary(), before.dictionary());
  EXPECT_EQ(after.indices(), before.indices());
  EXPECT_EQ(after.metadata().block_size, before.metadata().block_size);
}
}  // namespace vq_map_server::test2d
