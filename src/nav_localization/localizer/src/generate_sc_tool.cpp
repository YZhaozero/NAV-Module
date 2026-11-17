/**
 * @file generate_sc_tool.cpp
 * @brief Standalone tool to generate SC database for PCD maps
 */

#include <iostream>
#include <filesystem>
#include <atomic>
#include <csignal>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "localizers/icp_localizer.h"
#include "localizers/scan_context.h"

namespace {
std::atomic<bool> cancelled{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "SC cancellation signal requires a lock-free flag");
void cancel_generation(int) {
    cancelled.store(true, std::memory_order_relaxed);
}
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pcd_file>" << std::endl;
        return 1;
    }

    std::string pcd_path = argv[1];
    std::string sc_path = pcd_path + ".sc.v2";
    std::signal(SIGTERM, cancel_generation);
    std::signal(SIGINT, cancel_generation);

    // Check if PCD exists
    if (!std::filesystem::exists(pcd_path)) {
        std::cerr << "❌ PCD file not found: " << pcd_path << std::endl;
        return 1;
    }

    std::cout << "🔄 Generating SC database for: " << pcd_path << std::endl;

    try {
        // Create ICPLocalizer with default config
        ICPConfig config;
        config.use_scan_context = true;
        config.sc_max_radius = 80.0;
        config.sc_dist_thresh = 0.5;
        config.sc_grid_resolution = 2.0;

        ICPLocalizer localizer(config);
        localizer.setCancellationCheck([] {
            return cancelled.load(std::memory_order_relaxed);
        });

        // Load map (this will trigger SC database generation)
        if (!localizer.loadMap(pcd_path)) {
            std::cerr << "❌ Failed to load map: " << pcd_path << std::endl;
            return 1;
        }
        if (cancelled.load(std::memory_order_relaxed)) return 130;

        // Check if SC file was created
        if (localizer.scanContextCachePersisted()) {
            auto size = std::filesystem::file_size(sc_path);
            std::cout << "✅ SC database generated: " << sc_path
                     << " (" << (size / 1024.0) << " KB)" << std::endl;
            return 0;
        } else {
            std::cerr << "❌ SC database not created" << std::endl;
            return 1;
        }

    } catch (const ICPLocalizer::Cancelled&) {
        std::cout << "SC generation cancelled; committed cache preserved" << std::endl;
        return 130;
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
