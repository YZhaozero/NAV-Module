#pragma once

#include <csignal>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>

extern volatile std::sig_atomic_t g_stop_requested;

namespace fast_livo_stop {
inline volatile std::sig_atomic_t signal_number = 0;
inline volatile std::sig_atomic_t sender_pid = 0;
inline volatile std::sig_atomic_t sender_uid = 0;
inline volatile std::sig_atomic_t signal_code = 0;

// Only signal-safe scalar assignments here. Logging and /proc reads happen
// after the mapper has left its processing loop, with the ROS context alive.
inline void handle(int number, siginfo_t *info, void *)
{
  if (signal_number != 0) return;
  sender_pid = info ? info->si_pid : 0;
  sender_uid = info ? info->si_uid : 0;
  signal_code = info ? info->si_code : 0;
  signal_number = number;
  g_stop_requested = 1;
}

inline void install()
{
  struct sigaction action {};
  action.sa_sigaction = handle;
  action.sa_flags = SA_SIGINFO | SA_RESTART;
  sigemptyset(&action.sa_mask);
  sigaddset(&action.sa_mask, SIGINT);
  sigaddset(&action.sa_mask, SIGTERM);
  if (sigaction(SIGINT, &action, nullptr) != 0 ||
      sigaction(SIGTERM, &action, nullptr) != 0)
    throw std::runtime_error("Cannot install FAST-LIVO2 stop signal handlers");
}

inline void report(std::ostream &out, bool context_valid)
{
  const int number = signal_number;
  out << "[NAV_STOP] reason=" << (number ? "signal" : context_valid ? "run_returned" : "context_shutdown")
      << " signal=" << (number == SIGINT ? "SIGINT" : number == SIGTERM ? "SIGTERM" : "none")
      << " signal_number=" << number << " sender_pid=" << sender_pid
      << " sender_uid=" << sender_uid << " signal_code=" << signal_code
      << " context_valid=" << (context_valid ? "true" : "false");
  std::string command;
  if (sender_pid > 0) {
    std::ifstream source("/proc/" + std::to_string(sender_pid) + "/comm");
    std::getline(source, command);
  }
  // A short-lived sender may already be gone; /proc is only a best-effort name.
  // Do not log its full command line, which may contain account credentials.
  out << " sender_comm_at_report=" << (command.empty() ? "unavailable" : command) << std::endl;
}
}  // namespace fast_livo_stop
