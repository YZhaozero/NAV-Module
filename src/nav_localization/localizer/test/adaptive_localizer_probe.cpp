// Recorded-data harness for the production ICPLocalizer, without ROS or sensors.
#include "../src/localizers/icp_localizer.h"
#include <yaml-cpp/yaml.h>
#include <sys/resource.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
static void number(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value; else out << "null";
}
static void matrix(std::ostream& out, const M4F& pose) {
    out << '[';
    for (int row = 0; row < 4; ++row) {
        if (row) out << ',';
        out << '[';
        for (int col = 0; col < 4; ++col) {
            if (col) out << ',';
            number(out, pose(row, col));
        }
        out << ']';
    }
    out << ']';
}
static ICPConfig config(const YAML::Node& node) {
    ICPConfig result;
#define READ(name) if (node[#name]) result.name = node[#name].as<decltype(result.name)>();
    READ(rough_scan_resolution) READ(rough_map_resolution) READ(rough_max_iteration)
    READ(rough_score_thresh) READ(refine_scan_resolution) READ(refine_map_resolution)
    READ(refine_max_iteration) READ(refine_score_thresh) READ(correspondence_randomness)
    READ(maximum_optimizer_iterations) READ(global_support_distance)
    READ(global_support_min_ratio)
#undef READ
    // Isolate this candidate + the ordinary GICP acceptance path. Do not create
    // Scan Context caches beside the user's PCD or invoke a TEASER subprocess.
    result.use_scan_context = false;
    result.use_global_registration = false;
    result.sc_use_odom_gravity = true;
    result.use_correlative_registration = true;
    return result;
}
int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: adaptive_localizer_probe MAP.pcd CONFIG.yaml QUERIES.yaml OUTPUT.jsonl independent|tracking\n";
        return 2;
    }
    try {
        const std::string mode = argv[5];
        if (mode != "independent" && mode != "tracking") throw std::runtime_error("invalid mode");
        cv::setNumThreads(1);
        std::ofstream output(argv[4]);
        if (!output) throw std::runtime_error("cannot open result file");
        output << std::setprecision(17);
        const auto settings = config(YAML::LoadFile(argv[2]));
        ICPLocalizer localizer(settings);
        auto start = Clock::now();
        if (!localizer.loadMap(argv[1])) throw std::runtime_error("map load failed");
        output << "{\"prepare_s\":" << elapsed(start)
               << ",\"rough_map_points\":" << localizer.roughMap()->size()
               << ",\"refine_map_points\":" << localizer.refineMap()->size() << "}\n";
        output.flush();
        const auto queries = YAML::LoadFile(argv[3]);
        if (!queries.IsSequence() || queries.size() == 0) throw std::runtime_error("no query inputs");
        bool initialized = false;
        M4F map_from_odom = M4F::Identity();
        size_t index = 0;
        for (const auto& query : queries) {
            CloudType::Ptr cloud(new CloudType);
            if (pcl::io::loadPCDFile(query["pcd"].as<std::string>(), *cloud) < 0)
                throw std::runtime_error("query load failed");
            M4F odom = M4F::Identity();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) odom(r, c) = query["odom"][r][c].as<float>();
            if (!odom.allFinite()) throw std::runtime_error("nonfinite odometry");
            const bool global = mode == "independent" || !initialized;
            M4F pose = M4F::Identity();
            if (!global) pose = map_from_odom * odom;
            start = Clock::now();
            localizer.setInput(cloud);
            const double preprocess_s = elapsed(start);
            start = Clock::now();
            std::string last_failure;
            localizer.setDiagnosticsCallback([&](const MatchDiagnostics& update) {
                if (!update.reason_code.empty()) last_failure = update.reason_code;
            });
            const bool accepted = localizer.align(pose, global, !global, odom.block<3, 3>(0, 0));
            localizer.setDiagnosticsCallback({});
            const double align_s = elapsed(start);
            if (accepted) {
                map_from_odom = pose * odom.inverse();
                initialized = true;
            }
            const auto d = localizer.diagnostics();
            rusage usage{};
            getrusage(RUSAGE_SELF, &usage);
            output << "{\"index\":" << index++ << ",\"stamp\":" << query["stamp"].as<double>()
                   << ",\"global\":" << (global ? "true" : "false")
                   << ",\"accepted\":" << (accepted ? "true" : "false")
                   << ",\"reason\":\"" << d.reason_code << "\",\"stage\":\"" << d.stage
                   << "\",\"last_failure\":\"" << last_failure
                   << "\",\"preprocess_s\":" << preprocess_s << ",\"align_s\":" << align_s
                   << ",\"peak_rss_kib\":" << usage.ru_maxrss
                   << ",\"input_points\":" << d.input_points << ",\"rough_points\":" << d.rough_points
                   << ",\"refine_points\":" << d.refine_points << ",\"candidate_count\":" << d.candidate_count
                   << ",\"rough_fitness\":";
            number(output, d.rough_fitness);
            output << ",\"refine_fitness\":"; number(output, d.refine_fitness);
            output << ",\"support\":"; number(output, d.map_support_ratio);
            output << ",\"rmse\":"; number(output, d.rmse);
            output << ",\"inlier_ratio_percent\":"; number(output, d.inlier_ratio);
            output << ",\"map_from_query\":"; matrix(output, pose);
            output << ",\"map_from_odom\":";
            if (accepted) matrix(output, map_from_odom); else output << "null";
            output << "}\n";
            output.flush();
        }
        return 0; // Rejections are measurements, not harness failures.
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
