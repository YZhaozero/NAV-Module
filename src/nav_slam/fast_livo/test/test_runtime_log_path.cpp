#include "runtime_log_path.h"
#include "async_state_log.h"
#include <iostream>
#include <stdexcept>
#include <sys/wait.h>

static void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--print-session") {
    std::cout << fast_livo_logs::sessionDirectory().string() << std::endl;
    return 0;
  }
  namespace fs = std::filesystem;
  const auto test_root = fs::temp_directory_path() /
      ("nav-runtime-logs-" + std::to_string(getpid()) + "-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  unsetenv("NAV_FASTLIVO_LOG_ROOT"); unsetenv("XDG_STATE_HOME");
  setenv("HOME", (test_root / "home").c_str(), 1);
  require(fast_livo_logs::root() == test_root / "home/.local/state/nav/fast-livo", "HOME default changed");
  setenv("XDG_STATE_HOME", (test_root / "state").c_str(), 1);
  require(fast_livo_logs::root() == test_root / "state/nav/fast-livo", "XDG precedence changed");
  setenv("NAV_FASTLIVO_LOG_ROOT", (test_root / "explicit").c_str(), 1);
  require(fast_livo_logs::root() == test_root / "explicit", "explicit root ignored");
  const auto session = fast_livo_logs::sessionDirectory();
  require(session.parent_path() == test_root / "explicit", "session escaped log root");
  require(fast_livo_logs::path("imu.txt") == (session / "imu.txt").string(), "session changed between calls");
  require(!fs::exists(test_root), "path selection performed directory I/O");
  {
    AsyncStateLog log(std::array<std::string,2>{fast_livo_logs::path("imu.txt"), ""});
    require(log.append(0, "recorded\n"), "runtime log append failed");
  }
  std::ifstream file(session / "imu.txt");
  std::string line; std::getline(file, line);
  require(line == "recorded", "runtime worker did not create and write the session");
  file.close();
  int channel[2]; require(pipe(channel) == 0, "pipe failed");
  const auto child = fork(); require(child >= 0, "fork failed");
  if (!child) {
    close(channel[0]); dup2(channel[1], STDOUT_FILENO); close(channel[1]);
    execl(argv[0], argv[0], "--print-session", static_cast<char *>(nullptr));
    _exit(127);
  }
  close(channel[1]);
  std::string child_path;
  char buffer[512]; ssize_t count;
  while ((count = read(channel[0], buffer, sizeof(buffer))) > 0) child_path.append(buffer, count);
  close(channel[0]);
  int status = 0; require(waitpid(child, &status, 0) == child && WIFEXITED(status) && !WEXITSTATUS(status), "child launch failed");
  if (!child_path.empty() && child_path.back() == '\n') child_path.pop_back();
  require(fs::path(child_path).parent_path() == session.parent_path(), "child lost explicit root");
  require(child_path != session.string(), "new process reused earlier evidence directory");
  fs::remove_all(test_root);
  std::cout << "runtime logs: root precedence, lazy directory creation and process isolation passed\n";
}
