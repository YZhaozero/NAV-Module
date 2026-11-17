#pragma once
#include <cstdlib>
#include <cstring>

// Per-frame ASCII tables are useful at a terminal, but wake the device GUI
// dozens of times per second. Structured performance diagnostics stay enabled.
inline bool verboseFrameLogs() {
  static const bool enabled = [] {
    const char *value = std::getenv("NAV_FASTLIVO_VERBOSE_FRAMES");
    return value && std::strcmp(value, "1") == 0;
  }();
  return enabled;
}
