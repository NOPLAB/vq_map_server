// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map2d/map_converter.hpp"
#include <gtest/gtest.h>
#include <nav2_map_server/map_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace vq_map_server::map2d;

class Converter2dTest : public ::testing::Test {
protected:
  std::filesystem::path directory;
  void SetUp() override {
    char pattern[] = "/tmp/vq-map2d-XXXXXX";
    const auto *created = mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory = created;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::string yaml(const std::string &image, const std::string &extra = "",
                   const std::string &settings =
                       "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n") {
    const auto path = directory / "map.yaml";
    std::ofstream output(path);
    output << "image: " << image << "\nresolution: 0.25\n"
              "origin: [-12.125, 8.75, 0.73]\n" << settings << extra;
    output.close();
    return path.string();
  }
  void pgm(unsigned width, unsigned height, const std::vector<unsigned char> &pixels) {
    std::ofstream output(directory / "image.pgm", std::ios::binary);
    output << "P5\n" << width << " " << height << "\n255\n";
    output.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
  }
};

TEST_F(Converter2dTest, PreservesYamlGeometryAndEveryCellAcrossDefaultPartialBlocks) {
  std::vector<unsigned char> pixels(53 * 27);
  for (unsigned y = 0; y < 27; ++y)
    for (unsigned x = 0; x < 53; ++x)
      pixels[y * 53 + x] = (x + 2 * y) % 3 == 0 ? 255 : (x + 2 * y) % 3 == 1 ? 0 : 128;
  pgm(53, 27, pixels);
  const auto map = convertOccupancyGrid(yaml("image.pgm"));
  EXPECT_EQ(map.metadata().block_size, 24U);
  EXPECT_EQ(map.metadata().origin, (std::array<double, 2>{-12.125, 8.75}));
  EXPECT_DOUBLE_EQ(map.metadata().yaw, 0.73);
  EXPECT_FLOAT_EQ(map.metadata().resolution, 0.25F);
  EXPECT_EQ(map.blockDimensions(), (std::array<std::uint32_t, 2>{3, 2}));
  for (int y = 0; y < 27; ++y)
    for (int x = 0; x < 53; ++x) {
      const auto value = pixels[(26 - y) * 53 + x];
      EXPECT_EQ(map.at({x, y}), value == 255 ? CellState::Free :
                value == 0 ? CellState::Occupied : CellState::Unknown) << x << "," << y;
    }
  EXPECT_EQ(map.at({53, 26}), CellState::OutOfBounds);
}

TEST_F(Converter2dTest, BinaryUnknownRequiresExplicitReplacement) {
  pgm(3, 1, {255, 128, 0});
  const auto path = yaml("image.pgm");
  ConversionOptions options;
  options.block_size = 2;
  options.encoding = CellEncoding::Binary;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
  for (const auto policy : {UnknownPolicy::Free, UnknownPolicy::Occupied}) {
    options.unknown_policy = policy;
    const auto map = convertOccupancyGrid(path, options);
    EXPECT_EQ(map.at({0, 0}), CellState::Free);
    EXPECT_EQ(map.at({1, 0}), policy == UnknownPolicy::Free ? CellState::Free : CellState::Occupied);
    EXPECT_EQ(map.at({2, 0}), CellState::Occupied);
  }
  options.encoding = CellEncoding::Ternary;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
}

TEST_F(Converter2dTest, PngPreservesNativeNav2TrinaryCellsAndImageInversion) {
  // RGBA rows: opaque black/white/gray; transparent white/black, opaque white.
  const unsigned char png[] = {
      137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82,0,0,0,3,0,0,0,2,8,6,0,0,0,
      157,116,102,26,0,0,0,24,73,68,65,84,120,156,99,96,96,96,248,15,2,13,13,13,
      255,65,44,6,6,168,8,0,180,125,14,116,89,118,244,15,0,0,0,0,73,69,78,68,174,66,96,130};
  std::ofstream output(directory / "image.png", std::ios::binary);
  output.write(reinterpret_cast<const char *>(png), sizeof(png));
  output.close();
  const auto path = yaml("image.png");
  nav_msgs::msg::OccupancyGrid native;
  nav2_map_server::loadMapFromFile(nav2_map_server::loadMapYaml(path), native);
  const auto map = convertOccupancyGrid(path);
  // Nav2 releases differ in transparent-pixel handling. Preserve the installed
  // decoder's cells, and independently pin opaque pixels and image row order.
  ASSERT_EQ(native.data.size(), 6U);
  EXPECT_EQ(map.at({0, 1}), CellState::Occupied);
  EXPECT_EQ(map.at({1, 1}), CellState::Free);
  EXPECT_EQ(map.at({2, 1}), CellState::Unknown);
  EXPECT_EQ(map.at({2, 0}), CellState::Free);
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 3; ++x) {
      const auto cell = native.data[x + 3 * y];
      const auto expected = cell == 0 ? CellState::Free :
          cell == 100 ? CellState::Occupied : CellState::Unknown;
      EXPECT_EQ(map.at({x, y}), expected) << x << "," << y;
    }
}

TEST_F(Converter2dTest, NegateAndStrictThresholdsComeFromYaml) {
  pgm(4, 1, {0, 255, 85, 170});
  const auto map = convertOccupancyGrid(yaml(
      "image.pgm", "mode: trinary\n",
      "negate: 1\noccupied_thresh: 0.8\nfree_thresh: 0.4\n"));
  EXPECT_EQ(map.at({0, 0}), CellState::Free);
  EXPECT_EQ(map.at({1, 0}), CellState::Occupied);
  EXPECT_EQ(map.at({2, 0}), CellState::Free);
  EXPECT_EQ(map.at({3, 0}), CellState::Unknown);
}

TEST_F(Converter2dTest, DeduplicatesInFirstOccurrenceOrderAndCapsFinalCompactPayload) {
  pgm(6, 1, {0, 255, 0, 255, 0, 255});
  const auto path = yaml("image.pgm");
  ConversionOptions options;
  options.block_size = 1;
  options.encoding = CellEncoding::Binary;
  options.max_map_bytes = 8;  // Two byte patterns and six one-byte indices.
  const auto map = convertOccupancyGrid(path, options);
  EXPECT_EQ(map.dictionary(), (std::vector<std::uint8_t>{1, 0}));
  EXPECT_EQ(map.indices(), (std::vector<std::uint8_t>{0, 1, 0, 1, 0, 1}));
  EXPECT_EQ(map.indexBitWidth(), 8);
  options.max_map_bytes = 7;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::runtime_error);
  options.max_map_bytes = 0;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::runtime_error);
}

TEST_F(Converter2dTest, SelectsSixteenBitIndicesWhenDictionaryOutgrowsOneByte) {
  std::vector<unsigned char> pixels(257 * 16, 255);
  for (unsigned pattern = 0; pattern < 257; ++pattern)
    for (unsigned y = 0; y < 4; ++y)
      for (unsigned x = 0; x < 4; ++x)
        pixels[(3 - y) * 1028 + pattern * 4 + x] =
            (pattern & (1U << (x + 4 * y))) ? 0 : 255;
  pgm(1028, 4, pixels);
  const auto yaml_path = yaml("image.pgm");
  const auto path = (directory / "map.yml").string();
  std::filesystem::rename(yaml_path, path);
  ConversionOptions options;
  options.block_size = 4;
  options.encoding = CellEncoding::Binary;
  options.max_map_bytes = 1028;  // 257 two-byte patterns + 257 two-byte indices.
  const auto map = convertOccupancyGrid(path, options);
  EXPECT_EQ(map.dictionarySize(), 257U);
  EXPECT_EQ(map.indexBitWidth(), 16);
  EXPECT_EQ(map.indices()[512], 0);
  EXPECT_EQ(map.indices()[513], 1);
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 1028; ++x)
      EXPECT_EQ(map.at({x, y}), pixels[(3 - y) * 1028 + x] == 0 ?
                CellState::Occupied : CellState::Free);
  options.max_map_bytes = 1027;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::runtime_error);
}

TEST_F(Converter2dTest, RejectsUnsupportedModesExtensionsAndMalformedFiles) {
  pgm(1, 1, {255});
  for (const auto &mode : {"scale", "raw", "nonsense"})
    EXPECT_ANY_THROW(convertOccupancyGrid(yaml("image.pgm", std::string("mode: ") + mode + "\n")));
  EXPECT_ANY_THROW(convertOccupancyGrid(yaml("image.jpg")));
  EXPECT_ANY_THROW(convertOccupancyGrid(yaml("missing.png")));
  std::ofstream(directory / "bad.png") << "invalid PNG";
  EXPECT_ANY_THROW(convertOccupancyGrid(yaml("bad.png")));
  EXPECT_ANY_THROW(convertOccupancyGrid(yaml("image.pgm", "",
      "negate: 0\noccupied_thresh: 0.1\nfree_thresh: 0.8\n")));
  std::ofstream(directory / "broken.yaml") << "image: [";
  EXPECT_ANY_THROW(convertOccupancyGrid((directory / "broken.yaml").string()));
  EXPECT_ANY_THROW(convertOccupancyGrid((directory / "missing.yaml").string()));
  EXPECT_ANY_THROW(convertOccupancyGrid((directory / "image.pgm").string()));
}

TEST_F(Converter2dTest, RejectsInvalidConversionOptions) {
  pgm(1, 1, {255});
  const auto path = yaml("image.pgm");
  ConversionOptions options;
  options.block_size = 0;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
  options.block_size = 65;
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
  options.block_size = 1;
  options.encoding = static_cast<CellEncoding>(0);
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
  options.encoding = CellEncoding::Binary;
  options.unknown_policy = static_cast<UnknownPolicy>(99);
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
  options.unknown_policy = UnknownPolicy::Reject;
  options.frame_id = "invalid frame";
  EXPECT_THROW(convertOccupancyGrid(path, options), std::invalid_argument);
}

}  // namespace
