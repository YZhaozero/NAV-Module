#include "../src/localizers/teaser_process.h"

#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--teaser-solve") {
        sleep(60);
        return 0;
    }
    teaser::PointCloud source;
    teaser::PointCloud target;
    for (const auto &point : std::vector<teaser::PointXYZ>{
             {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
             {0, 0, 1}, {2, 1, 0}, {1, 0, 2}}) {
        source.push_back(point);
        target.push_back(point);
    }
    std::vector<std::pair<int, int>> pairs;
    for (int index = 0; index < 6; ++index)
        pairs.emplace_back(index, index);
    teaser::RobustRegistrationSolver::Params parameters;
    parameters.noise_bound = .01;
    parameters.max_clique_num_threads = 1;

    const auto good = bounded_teaser::solve(
        source, target, pairs, parameters, 3.);
    assert(good.state == bounded_teaser::State::completed);
    assert(good.solution.valid);
    assert(good.solution.translation.norm() < 1e-6);
    assert(good.solution.rotation.isApprox(
        Eigen::Matrix3d::Identity(), 1e-6));

    const auto stopped = bounded_teaser::solve(
        source, target, pairs, parameters, .15, argv[0]);
    assert(stopped.state == bounded_teaser::State::timed_out);
    assert(stopped.elapsed_s < 1.);
    int status = 0;
    assert(waitpid(stopped.child, &status, WNOHANG) == -1 && errno == ECHILD);

    const auto failed = bounded_teaser::solve(
        source, target, pairs, parameters, .5, "/bin/true");
    assert(failed.state == bounded_teaser::State::failed);
    assert(!failed.solution.valid);

    if (argc == 2) {
        const auto stressed = bounded_teaser::solve(
            source, target, pairs, parameters, .3, argv[1]);
        assert(stressed.state == bounded_teaser::State::timed_out);
        assert(stressed.elapsed_s < 1.);
        std::cout << "NATIVE_STRESS bounded_s=" << stressed.elapsed_s
                  << std::endl;
    }
    std::cout << "owned solver process: success, timeout/reaping, "
                 "malformed output passed" << std::endl;
}
