#include "localizers/teaser_process.h"

#include <iostream>
#include <sys/resource.h>

int main(int argc, char **argv) {
    if (argc != 2 || std::string(argv[1]) != "--teaser-solve") return 2;
    const rlimit memory{1536ULL * 1024 * 1024, 1536ULL * 1024 * 1024};
    const rlimit core{0, 0};
    setrlimit(RLIMIT_AS, &memory);
    setrlimit(RLIMIT_CORE, &core);

    using namespace bounded_teaser;
    Request request;
    if (!transfer(3, &request, sizeof(request), false) ||
        request.identity != kMagic ||
        request.source_count < 3 || request.source_count > 500000 ||
        request.target_count < 3 || request.target_count > 2000000 ||
        request.pair_count < 3 || request.pair_count > 100000 ||
        !std::isfinite(request.noise_bound) || request.noise_bound <= 0. ||
        !std::isfinite(request.cbar2) || request.cbar2 <= 0. ||
        !std::isfinite(request.clique_seconds) ||
        request.clique_seconds <= 0. ||
        request.clique_threads < 1 || request.clique_threads > 2 ||
        request.rotation_iterations < 1 ||
        request.rotation_iterations > 1000 ||
        !std::isfinite(request.rotation_factor) ||
        request.rotation_factor <= 1. ||
        !std::isfinite(request.rotation_threshold) ||
        request.rotation_threshold <= 0.) return 3;

    try {
        std::vector<teaser::PointXYZ> source_points(request.source_count);
        std::vector<teaser::PointXYZ> target_points(request.target_count);
        std::vector<int32_t> matches(request.pair_count * 2);
        if (!transfer(3, source_points.data(),
                      source_points.size() * sizeof(source_points[0]), false) ||
            !transfer(3, target_points.data(),
                      target_points.size() * sizeof(target_points[0]), false) ||
            !transfer(3, matches.data(),
                      matches.size() * sizeof(matches[0]), false)) return 3;

        teaser::PointCloud source;
        teaser::PointCloud target;
        for (const auto &point : source_points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) return 3;
            source.push_back(point);
        }
        for (const auto &point : target_points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) return 3;
            target.push_back(point);
        }
        std::vector<std::pair<int, int>> pairs;
        for (size_t index = 0; index < request.pair_count; ++index) {
            const auto source_index = matches[2 * index];
            const auto target_index = matches[2 * index + 1];
            if (source_index < 0 || target_index < 0 ||
                static_cast<uint64_t>(source_index) >= request.source_count ||
                static_cast<uint64_t>(target_index) >= request.target_count)
                return 3;
            pairs.emplace_back(source_index, target_index);
        }

        teaser::RobustRegistrationSolver::Params parameters;
        parameters.noise_bound = request.noise_bound;
        parameters.cbar2 = request.cbar2;
        parameters.estimate_scaling = false;
        parameters.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::
                ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        parameters.rotation_gnc_factor = request.rotation_factor;
        parameters.rotation_max_iterations = request.rotation_iterations;
        parameters.rotation_cost_threshold = request.rotation_threshold;
        parameters.max_clique_time_limit = request.clique_seconds;
        parameters.max_clique_num_threads = request.clique_threads;

        teaser::RobustRegistrationSolver solver(parameters);
        solver.solve(source, target, pairs);
        const auto solution = solver.getSolution();
        Response response;
        response.valid = solution.valid ? 1 : 0;
        if (solution.valid) {
            for (int row = 0; row < 3; ++row) {
                response.translation[row] = solution.translation(row);
                for (int column = 0; column < 3; ++column)
                    response.rotation[3 * row + column] =
                        solution.rotation(row, column);
            }
        }
        return transfer(4, &response, sizeof(response), true) ? 0 : 4;
    } catch (const std::exception &error) {
        std::cerr << "TEASER worker failed: " << error.what() << std::endl;
        return 4;
    }
}
