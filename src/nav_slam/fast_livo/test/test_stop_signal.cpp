// Exercise actual kernel-delivered stop signals without requiring sensor input.
#include "stop_signal.h"
#include <sys/wait.h>
#include <unistd.h>
#include <sstream>
#include <iostream>

volatile std::sig_atomic_t g_stop_requested = 0;

void require(bool passed, const char *message)
{
  if (!passed) throw std::runtime_error(message);
}

void checkSignal(int number)
{
  int ready[2], result[2];
  require(pipe(ready) == 0 && pipe(result) == 0, "pipe failed");
  const pid_t sender = getpid();
  const pid_t child = fork();
  require(child >= 0, "fork failed");
  if (child == 0) {
    close(ready[0]); close(result[0]);
    alarm(5);
    fast_livo_stop::install();
    const char marker = 'R';
    if (write(ready[1], &marker, 1) != 1) _exit(2);
    close(ready[1]);
    while (!g_stop_requested) usleep(1000);
    std::ostringstream report;
    fast_livo_stop::report(report, true);
    const std::string text = report.str();
    if (write(result[1], text.data(), text.size()) != static_cast<ssize_t>(text.size())) _exit(3);
    close(result[1]);
    _exit(0);
  }
  close(ready[1]); close(result[1]);
  char marker = 0;
  require(read(ready[0], &marker, 1) == 1 && marker == 'R', "child not ready");
  close(ready[0]);
  require(kill(child, number) == 0, "signal delivery failed");
  char buffer[1024];
  const ssize_t size = read(result[0], buffer, sizeof(buffer));
  close(result[0]);
  int status = 0;
  require(waitpid(child, &status, 0) == child, "wait failed");
  require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "stop bypassed orderly cleanup");
  require(size > 0, "missing stop report");
  const std::string text(buffer, size);
  require(text.find("reason=signal") != std::string::npos, "missing signal reason");
  require(text.find("signal_number=" + std::to_string(number)) != std::string::npos, "wrong signal");
  require(text.find("sender_pid=" + std::to_string(sender)) != std::string::npos, "wrong sender PID");
  require(text.find("sender_uid=" + std::to_string(getuid())) != std::string::npos, "wrong sender UID");
  require(text.find("context_valid=true") != std::string::npos, "context prematurely invalidated");
  std::cout << text;
}

int main()
{
  checkSignal(SIGINT);
  checkSignal(SIGTERM);
  std::ostringstream context_stop;
  fast_livo_stop::report(context_stop, false);
  require(context_stop.str().find("reason=context_shutdown signal=none") != std::string::npos,
          "context shutdown misreported as a signal");
  std::cout << "stop signal regression passed\n";
}
