// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "cli_fixture.hpp"

namespace vq_map_server::test2d {
TEST_F(CliTest, InspectRejectsInvalidNumbersDuplicateFlagsAndMalformedMaps) {
  const auto converted = convert({"--input", input, "--output", output});
  ASSERT_EQ(converted.status, 0) << converted.err;
  for (const auto &option : {"--queries", "--max-map-bytes"}) {
    for (const auto &number : {"-1", "+1", "", "1junk", "1.5", "nan", "inf", " 1", "1 ",
                               "184467440737095516160"}) {
      expectFailure(inspect({output, option, number}), "vq_map_inspect_2d");
    }
    expectFailure(inspect({output, option, "1", option, "2"}), "vq_map_inspect_2d");
    expectFailure(inspect({output, option}), "vq_map_inspect_2d");
    expectFailure(inspect({output, option, "--queries"}), "vq_map_inspect_2d");
  }
  expectFailure(inspect({output, "--max-map-bytes", "0"}), "vq_map_inspect_2d");
  const auto malformed = (directory / "bad.h5").string();
  std::ofstream(malformed) << "not HDF5";
  expectFailure(inspect({malformed}), "vq_map_inspect_2d");
}

TEST_F(CliTest, InspectCountsIndexRecordsForEverySupportedWidth) {
  for (const std::uint8_t width : {8, 16, 32}) {
    MapMetadata metadata;
    metadata.dimensions = {2, 1};
    metadata.block_size = 1;
    metadata.encoding = CellEncoding::Binary;
    std::vector<std::uint8_t> indices(2 * (width / 8), 0);
    indices[width / 8] = 1;
    const auto path = (directory / ("width_" + std::to_string(width) + ".h5")).string();
    writeHdf5(path, VqMap(metadata, {0, 1}, indices, width));
    const auto result = inspect({path});
    ASSERT_EQ(result.status, 0) << result.err;
    EXPECT_EQ(value(result.out, "index_bit_width"), std::to_string(width));
    EXPECT_EQ(value(result.out, "blocks"), "2");
    EXPECT_EQ(value(result.out, "dictionary_patterns"), "2");
    EXPECT_EQ(value(result.out, "cell_count"), "2");
  }
}


TEST_F(CliTest, InspectCountsEveryQueryForUniformSingleCellMaps) {
  for (const std::uint8_t code : {0, 1, 2}) {
    MapMetadata metadata;
    metadata.dimensions = {1, 1};
    metadata.block_size = 1;
    const auto path = (directory / (std::to_string(code) + ".h5")).string();
    writeHdf5(path, VqMap(metadata, {code}, {0}, 8));
    const auto result = inspect({path, "--queries", "31"});
    ASSERT_EQ(result.status, 0) << result.err;
    EXPECT_EQ(value(result.out, "query_occupied"), code == 1 ? "31" : "0");
    EXPECT_EQ(value(result.out, "query_unknown"), code == 2 ? "31" : "0");
  }
}
}  // namespace vq_map_server::test2d
