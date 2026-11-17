#pragma once

// POSIX spawn/exec keeps a potentially unbounded TEASER solve out of ROS
// threads. Input and output use anonymous, close-on-exec temporary files; only
// descriptors 3 and 4 are exposed to the owned worker process.
#include <teaser/registration.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <signal.h>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char **environ;

namespace bounded_teaser {

constexpr uint64_t kMagic = 0x47534e4156545331ULL;

struct Request {
    uint64_t identity = kMagic;
    uint64_t source_count = 0;
    uint64_t target_count = 0;
    uint64_t pair_count = 0;
    double noise_bound = 0.;
    double cbar2 = 2.;
    double clique_seconds = 2.;
    int32_t clique_threads = 2;
    int32_t rotation_iterations = 200;
    double rotation_factor = 1.4;
    double rotation_threshold = 1e-6;
};

struct Response {
    uint64_t identity = kMagic;
    int32_t valid = 0;
    double rotation[9]{};
    double translation[3]{};
};

inline bool transfer(int fd, void *data, size_t bytes, bool writing) {
    auto *cursor = static_cast<char *>(data);
    while (bytes) {
        const auto count = writing ? ::write(fd, cursor, bytes) :
                                     ::read(fd, cursor, bytes);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return false;
        cursor += count;
        bytes -= static_cast<size_t>(count);
    }
    return true;
}

struct AnonymousFile {
    int fd = -1;

    AnonymousFile() {
        char name[] = "/tmp/nav-solver-XXXXXX";
        const int original = mkstemp(name);
        if (original < 0) return;
        unlink(name);
        fd = fcntl(original, F_DUPFD_CLOEXEC, 10);
        close(original);
    }

    ~AnonymousFile() {
        if (fd >= 0) close(fd);
    }

    AnonymousFile(const AnonymousFile &) = delete;
    AnonymousFile &operator=(const AnonymousFile &) = delete;
};

enum class State { completed, timed_out, failed };

struct Result {
    State state = State::failed;
    teaser::RegistrationSolution solution;
    double elapsed_s = 0.;
    pid_t child = -1;
};

inline Result solve(
    const teaser::PointCloud &source,
    const teaser::PointCloud &target,
    const std::vector<std::pair<int, int>> &pairs,
    const teaser::RobustRegistrationSolver::Params &parameters,
    double timeout_s,
    std::string worker_path = "")
{
    Result result;
    result.solution.valid = false;
    if (!std::isfinite(timeout_s) || timeout_s <= 0. || source.size() == 0 ||
        target.size() == 0 || pairs.empty()) return result;

    AnonymousFile input;
    AnonymousFile output;
    if (input.fd < 0 || output.fd < 0) return result;

    Request request;
    request.source_count = source.size();
    request.target_count = target.size();
    request.pair_count = pairs.size();
    request.noise_bound = parameters.noise_bound;
    request.cbar2 = parameters.cbar2;
    request.clique_seconds = parameters.max_clique_time_limit;
    request.clique_threads = parameters.max_clique_num_threads;
    request.rotation_iterations = parameters.rotation_max_iterations;
    request.rotation_factor = parameters.rotation_gnc_factor;
    request.rotation_threshold = parameters.rotation_cost_threshold;

    std::vector<teaser::PointXYZ> source_points(source.begin(), source.end());
    std::vector<teaser::PointXYZ> target_points(target.begin(), target.end());
    std::vector<int32_t> matches;
    matches.reserve(pairs.size() * 2);
    for (const auto &pair : pairs) {
        matches.push_back(pair.first);
        matches.push_back(pair.second);
    }
    if (!transfer(input.fd, &request, sizeof(request), true) ||
        !transfer(input.fd, source_points.data(),
                  source_points.size() * sizeof(source_points[0]), true) ||
        !transfer(input.fd, target_points.data(),
                  target_points.size() * sizeof(target_points[0]), true) ||
        !transfer(input.fd, matches.data(),
                  matches.size() * sizeof(matches[0]), true) ||
        lseek(input.fd, 0, SEEK_SET) < 0) return result;

    if (worker_path.empty()) {
        std::array<char, 4096> executable{};
        const auto length = readlink(
            "/proc/self/exe", executable.data(), executable.size() - 1);
        if (length <= 0) return result;
        worker_path = (std::filesystem::path(
            std::string(executable.data(), length)).parent_path() /
            "teaser_solve_worker").string();
    }

    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0) return result;
    const int input_action = posix_spawn_file_actions_adddup2(
        &actions, input.fd, 3);
    const int output_action = posix_spawn_file_actions_adddup2(
        &actions, output.fd, 4);
    char option[] = "--teaser-solve";
    char *arguments[] = {worker_path.data(), option, nullptr};
    const auto start = std::chrono::steady_clock::now();
    const int error = (input_action || output_action) ? EINVAL :
        posix_spawn(&result.child, worker_path.c_str(), &actions, nullptr,
                    arguments, environ);
    posix_spawn_file_actions_destroy(&actions);
    if (error) {
        result.child = -1;
        return result;
    }

    int status = 0;
    bool timed_out = false;
    for (;;) {
        const auto waited = waitpid(result.child, &status, WNOHANG);
        if (waited == result.child) break;
        if (waited < 0 && errno == EINTR) continue;
        if (waited < 0) return result;
        if (std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count() >= timeout_s) {
            timed_out = true;
            kill(result.child, SIGKILL);
            while (waitpid(result.child, &status, 0) < 0 && errno == EINTR) {}
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    result.elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (timed_out) {
        result.state = State::timed_out;
        return result;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 ||
        lseek(output.fd, 0, SEEK_SET) < 0) return result;

    Response response;
    if (!transfer(output.fd, &response, sizeof(response), false) ||
        response.identity != kMagic) return result;
    for (int row = 0; row < 3; ++row) {
        if (!std::isfinite(response.translation[row])) return result;
        result.solution.translation(row) = response.translation[row];
        for (int column = 0; column < 3; ++column) {
            if (!std::isfinite(response.rotation[3 * row + column]))
                return result;
            result.solution.rotation(row, column) =
                response.rotation[3 * row + column];
        }
    }
    result.solution.valid = response.valid == 1;
    result.solution.scale = 1.;
    result.state = State::completed;
    return result;
}

}  // namespace bounded_teaser
