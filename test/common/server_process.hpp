// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <poll.h>
#include <signal.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <vector>

// Only async-signal-safe operations run between fork and exec.
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
    return WIFEXITED(status) && WEXITSTATUS(status) != 0 &&
           WEXITSTATUS(status) != 127;
  }

private:
  pid_t pid_ = -1;
};
