#pragma once

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unistd.h>

namespace fast_livo_logs {
inline std::filesystem::path root() {
  if (const char *value = std::getenv("NAV_FASTLIVO_LOG_ROOT"); value && *value) return value;
  if (const char *value = std::getenv("XDG_STATE_HOME"); value && *value)
    return std::filesystem::path(value) / "nav/fast-livo";
  if (const char *value = std::getenv("HOME"); value && *value)
    return std::filesystem::path(value) / ".local/state/nav/fast-livo";
  return std::filesystem::path("/tmp") / ("nav-" + std::to_string(getuid())) / "fast-livo";
}

inline const std::filesystem::path &sessionDirectory() {
  // Path selection only: directory creation and file I/O belong to the log
  // worker, or the one-time initialization of an explicitly enabled export.
  static const std::filesystem::path directory = root() /
      ("run-" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count()) +
       "-" + std::to_string(getpid()));
  return directory;
}

inline std::string path(const std::string &relative) {
  return (sessionDirectory() / relative).string();
}
}  // namespace fast_livo_logs
