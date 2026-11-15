// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "vq_map_server/map_io.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

class CliTest : public ::testing::Test {
protected:
  std::filesystem::path directory;
  std::string input, output, config;
  struct Result { int status; std::string out, err; };
  void SetUp() override {
    char pattern[] = "/tmp/vq-cli-XXXXXX";
    const auto *created = mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory = created;
    input = (directory / "points.pcd").string();
    output = (directory / "map.h5").string();
    config = (directory / "config.yaml").string();
    std::ofstream(input) << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                            "COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n"
                            "DATA ascii\n0 0 0\n2 0 0\n";
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  Result run(const std::string &exe, const std::vector<std::string> &arguments) {
    const auto stdout_path = (directory / "stdout").string();
    const auto stderr_path = (directory / "stderr").string();
    const auto pid = fork();
    if (pid < 0) return {-1, "", "fork failed"};
    if (pid == 0) {
      const int out = open(stdout_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
      const int err = open(stderr_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
      if (out < 0 || err < 0 || dup2(out, STDOUT_FILENO) < 0 ||
          dup2(err, STDERR_FILENO) < 0) _exit(126);
      close(out);
      close(err);
      std::vector<char *> argv;
      argv.push_back(const_cast<char *>(exe.c_str()));
      for (const auto &argument : arguments)
        argv.push_back(const_cast<char *>(argument.c_str()));
      argv.push_back(nullptr);
      execv(exe.c_str(), argv.data());
      _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) != pid) return {-1, "", "waitpid failed"};
    std::ifstream out(stdout_path), err(stderr_path);
    return {WIFEXITED(status) ? WEXITSTATUS(status) : -1,
            std::string(std::istreambuf_iterator<char>(out), {}),
            std::string(std::istreambuf_iterator<char>(err), {})};
  }
  Result convert(const std::vector<std::string> &arguments) {
    return run(TEST_CONVERT_PATH, arguments);
  }
  Result inspect(const std::vector<std::string> &arguments) {
    return run(TEST_INSPECT_PATH, arguments);
  }
};

TEST_F(CliTest, HelpAndMissingArguments) {
  EXPECT_EQ(convert({"--help"}).status, 0);
  EXPECT_EQ(inspect({"--help"}).status, 0);
  for (const auto &arguments : std::vector<std::vector<std::string>>{
           {}, {"--input"}, {"--input", input}, {"--unknown", "1"},
           {"--input", input, "--output"}, {"--input", input, "--output", output,
                                           "--block-size", "--frame-id", "map"}}) {
    const auto result = convert(arguments);
    EXPECT_NE(result.status, 0);
    EXPECT_FALSE(result.err.empty());
  }
  for (const auto &arguments : std::vector<std::vector<std::string>>{
           {}, {"--queries"}, {"--bad"}, {"absent.h5"}, {"a.h5", "b.h5"}}) {
    const auto result = inspect(arguments);
    EXPECT_NE(result.status, 0);
    EXPECT_FALSE(result.err.empty());
  }
}

TEST_F(CliTest, RoundTripMetadataExclusiveOutputAndBenchmark) {
  const auto converted = convert({"--input", input, "--output", output,
                                  "--voxel-size", "1", "--block-size", "2"});
  ASSERT_EQ(converted.status, 0) << converted.err;
  EXPECT_NE(converted.out.find("memory_bytes:"), std::string::npos);
  EXPECT_NE(converted.out.find("peak_rss_kib:"), std::string::npos);
  EXPECT_NE(converted.out.find("elapsed_ms:"), std::string::npos);
  const auto before = vq_map_server::readHdf5(output);
  const auto duplicate = convert({"--input", input, "--output", output});
  EXPECT_NE(duplicate.status, 0);
  EXPECT_FALSE(duplicate.err.empty());
  EXPECT_EQ(vq_map_server::readHdf5(output).dictionary(), before.dictionary());
  const auto result = inspect({output, "--queries", "100"});
  ASSERT_EQ(result.status, 0) << result.err;
  EXPECT_NE(result.out.find("dimensions: 3 1 1"), std::string::npos);
  EXPECT_NE(result.out.find("dictionary_patterns: 1"), std::string::npos);
  EXPECT_NE(result.out.find("queries: 100"), std::string::npos);
  const auto again = inspect({"--queries", "100", output});
  ASSERT_EQ(again.status, 0) << again.err;
  const auto count = result.out.substr(result.out.find("query_occupied:"));
  const auto repeat_count = again.out.substr(again.out.find("query_occupied:"));
  EXPECT_EQ(count.substr(0, count.find('\n')), repeat_count.substr(0, repeat_count.find('\n')));
  EXPECT_NE(inspect({output, "--max-map-bytes", "1"}).status, 0);
}

TEST_F(CliTest, ConfigAndFlagsOverrideRegardlessOfOrder) {
  std::ofstream(config) << "/**:\n  ros__parameters:\n"
                          "    voxel_size: 0.5\n    block_size: 3\n"
                          "    frame_id: config_frame\n    max_map_bytes: 1024\n";
  auto result = convert({"--voxel-size", "1", "--frame-id", "flag_frame",
                        "--input", input, "--config", config, "--output", output});
  ASSERT_EQ(result.status, 0) << result.err;
  auto map = vq_map_server::readHdf5(output);
  EXPECT_EQ(map.metadata().voxel_size, 1);
  EXPECT_EQ(map.metadata().block_size, 3u);
  EXPECT_EQ(map.metadata().frame_id, "flag_frame");
  const auto other = (directory / "other.h5").string();
  result = convert({"--input", input, "--output", other, "--config", config,
                    "--voxel-size", "1", "--frame-id", "flag_frame"});
  ASSERT_EQ(result.status, 0) << result.err;
  EXPECT_EQ(vq_map_server::readHdf5(other).dictionary(), map.dictionary());
  const auto only_config = (directory / "config.h5").string();
  result = convert({"--input", input, "--output", only_config, "--config", config});
  ASSERT_EQ(result.status, 0) << result.err;
  map = vq_map_server::readHdf5(only_config);
  EXPECT_EQ(map.metadata().voxel_size, 0.5F);
  EXPECT_EQ(map.metadata().frame_id, "config_frame");
}

TEST_F(CliTest, StrictNumbersUnknownConfigAndInvalidValues) {
  for (const auto &option : {"--voxel-size", "--block-size", "--max-map-bytes"}) {
    for (const auto &value : {"1junk", "-1", "", "nan", "inf", "1e1000"}) {
      const auto result = convert({"--input", input, "--output", output, option, value});
      EXPECT_NE(result.status, 0) << option << " " << value;
      EXPECT_FALSE(result.err.empty());
      EXPECT_FALSE(std::filesystem::exists(output));
    }
  }
  for (const auto &option : {"--block-size", "--max-map-bytes"}) {
    EXPECT_NE(convert({"--input", input, "--output", output, option,
                       "999999999999999999999999"}).status, 0);
  }
  for (const auto &option : {"--voxel-size", "--block-size", "--max-map-bytes"}) {
    EXPECT_NE(convert({"--input", input, "--output", output, option, "0"}).status, 0);
  }
  EXPECT_NE(convert({"--input", input, "--output", output, "--block-size", "65"}).status, 0);
  EXPECT_NE(convert({"--input", input, "--output", output, "--frame-id", ""}).status, 0);
  EXPECT_NE(convert({"--input", input, "--output", output, "--input", input}).status, 0);
  for (const auto &text : {"not: [valid", "voxel_size: 1\n",
                           "/**:\n  ros__parameters:\n    unknown: 1\n",
                           "/**:\n  ros__parameters:\n    voxel_size: 1junk\n",
                           "/**:\n  ros__parameters:\n    block_size: [1, 2]\n"}) {
    std::ofstream(config) << text;
    const auto result = convert({"--input", input, "--output", output, "--config", config});
    EXPECT_NE(result.status, 0) << text;
    EXPECT_FALSE(result.err.empty());
  }
  for (const auto &value : {"1x", "-1", "", "184467440737095516160"}) {
    EXPECT_NE(inspect({output, "--queries", value}).status, 0);
    EXPECT_NE(inspect({output, "--max-map-bytes", value}).status, 0);
  }
}

}  // namespace
