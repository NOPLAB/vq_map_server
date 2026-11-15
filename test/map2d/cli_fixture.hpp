// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "vq_map_server/map2d/map_io.hpp"
#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace vq_map_server::test2d {
using vq_map_server::map2d::CellEncoding;
using vq_map_server::map2d::CellState;
using vq_map_server::map2d::MapMetadata;
using vq_map_server::map2d::VqMap;
using vq_map_server::map2d::readHdf5;
using vq_map_server::map2d::writeHdf5;

class CliTest : public ::testing::Test {
protected:
  struct Result { int status; std::string out, err; };
  std::filesystem::path directory;
  std::string input, output, config;

  void SetUp() override {
    char pattern[] = "/tmp/vq-cli-2d-XXXXXX";
    const auto created = mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory = created;
    input = (directory / "map.yaml").string();
    output = (directory / "map.h5").string();
    config = (directory / "config.yaml").string();
    {
      // Top-down image rows become bottom-up occupancy rows.
      std::ofstream image(directory / "map.pgm", std::ios::binary);
      image << "P5\n3 2\n255\n";
      const std::array<unsigned char, 6> pixels{0, 255, 205, 255, 205, 0};
      image.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
      ASSERT_TRUE(image.good());
    }
    std::ofstream yaml(input);
    yaml << "image: map.pgm\nresolution: 0.25\norigin: [-2.0, 3.0, 0.7]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            "mode: trinary\n";
    ASSERT_TRUE(yaml.good());
  }

  void TearDown() override { std::filesystem::remove_all(directory); }

  Result run(const std::string &exe, const std::vector<std::string> &arguments) {
    const auto stdout_path = (directory / "stdout").string();
    const auto stderr_path = (directory / "stderr").string();
    std::vector<char *> argv{const_cast<char *>(exe.c_str())};
    for (const auto &argument : arguments)
      argv.push_back(const_cast<char *>(argument.c_str()));
    argv.push_back(nullptr);
    const auto pid = fork();
    if (pid < 0) return {-1, "", "fork failed"};
    if (pid == 0) {
      const int out = open(stdout_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
      const int err = open(stderr_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
      if (out < 0 || err < 0 || dup2(out, STDOUT_FILENO) < 0 ||
          dup2(err, STDERR_FILENO) < 0) _exit(126);
      close(out);
      close(err);
      execv(exe.c_str(), argv.data());
      _exit(127);
    }
    int status = 0;
    pid_t reaped;
    do { reaped = waitpid(pid, &status, 0); } while (reaped < 0 && errno == EINTR);
    if (reaped != pid) return {-1, "", "waitpid failed"};
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

  void expectFailure(const Result &result, const std::string &program) {
    EXPECT_GT(result.status, 0) << result.err;
    EXPECT_NE(result.status, 126);
    EXPECT_NE(result.status, 127);
    EXPECT_NE(result.err.find(program + ":"), std::string::npos) << result.err;
  }

  std::string value(const std::string &text, const std::string &name) {
    const auto begin = text.find(name + ": ");
    if (begin == std::string::npos) return "";
    const auto first = begin + name.size() + 2;
    return text.substr(first, text.find('\n', first) - first);
  }
};

}  // namespace vq_map_server::test2d
