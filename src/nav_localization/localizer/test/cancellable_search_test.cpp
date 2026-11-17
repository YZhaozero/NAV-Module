#include "../src/localizers/cancellable_search.h"
#include "../src/localizers/commons.h"
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <atomic>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

using GICP = pcl::GeneralizedIterativeClosestPoint<PointType, PointType>;
using Tree = CancellableSearch<PointType>;
struct Cancelled : std::runtime_error {
    Cancelled() : std::runtime_error("cancelled") {}
};

CloudType::Ptr scene() {
    CloudType::Ptr cloud(new CloudType);
    for (int z = 0; z < 7; ++z)
        for (int y = 0; y < 11; ++y)
            for (int x = 0; x < 13; ++x) {
                PointType point;
                point.x = .23f*x + .009f*std::sin(float(y*z+x));
                point.y = .29f*y + .013f*std::cos(float(x*z+y));
                point.z = .31f*z + .007f*std::sin(float(x*y+z));
                cloud->push_back(point);
            }
    return cloud;
}

void configure(GICP &gicp, const CloudType::Ptr &source, const CloudType::Ptr &target) {
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaximumIterations(5);
    gicp.setMaximumOptimizerIterations(10);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaxCorrespondenceDistance(1.5);
    gicp.setTransformationEpsilon(1e-8);
}

void search_and_registration_match_original_pcl() {
    auto target = scene();
    int checks = 0;
    Tree wrapped([&] { ++checks; });
    pcl::search::KdTree<PointType> original;
    wrapped.setInputCloud(target); original.setInputCloud(target);
    for (std::size_t i = 0; i < target->size(); i += 37) {
        pcl::Indices a, b; std::vector<float> ad, bd;
        assert(original.nearestKSearch((*target)[i], 20, a, ad) ==
               wrapped.nearestKSearch((*target)[i], 20, b, bd));
        assert(a == b && ad == bd);
        assert(original.radiusSearch((*target)[i], .5, a, ad) ==
               wrapped.radiusSearch((*target)[i], .5, b, bd));
        assert(a == b && ad == bd);
    }
    assert(checks > 2);

    M4F shift = M4F::Identity();
    shift.block<3,3>(0,0) = Eigen::AngleAxisf(.015f, V3F::UnitZ()).toRotationMatrix();
    shift(0,3) = .025f; shift(1,3) = -.02f;
    CloudType::Ptr source(new CloudType);
    pcl::transformPointCloud(*target, *source, shift);
    GICP base, hooked;
    configure(base, source, target); configure(hooked, source, target);
    int target_checks = 0, source_checks = 0;
    hooked.setSearchMethodTarget(pcl::make_shared<Tree>([&] { ++target_checks; }));
    hooked.setSearchMethodSource(pcl::make_shared<Tree>([&] { ++source_checks; }));
    CloudType base_output, hooked_output;
    base.align(base_output); hooked.align(hooked_output);
    assert(base.hasConverged() && hooked.hasConverged());
    assert((base.getFinalTransformation()-hooked.getFinalTransformation()).norm() < 1e-7);
    assert(std::abs(base.getFitnessScore()-hooked.getFitnessScore()) < 1e-12);
    assert(base_output.size() == target->size() && hooked_output.size() == target->size());
    assert(target_checks > 2*int(target->size()) && source_checks > 2*int(source->size()));
}

void cancellation_before_and_after_tree_build() {
    auto cloud = scene();
    for (int stop : {1,2}) {
        int checks = 0;
        Tree tree([&] { if (++checks == stop) throw Cancelled(); });
        bool caught = false;
        try { tree.setInputCloud(cloud); } catch (const Cancelled &) { caught = true; }
        assert(caught && checks == stop);
    }
}

void cancellation_inside_original_gicp_loops() {
    auto cloud = scene();
    // Target covariance, source covariance, then the correspondence loop.
    for (int phase = 0; phase < 3; ++phase) {
        GICP gicp; configure(gicp, cloud, cloud);
        int target_checks = 0, source_checks = 0;
        const int target_stop = phase == 0 ? 3 : 2 + 2*int(cloud->size()) + 3;
        gicp.setSearchMethodTarget(pcl::make_shared<Tree>([&] {
            ++target_checks;
            if (phase != 1 && target_checks == target_stop) throw Cancelled();
        }));
        gicp.setSearchMethodSource(pcl::make_shared<Tree>([&] {
            if (++source_checks == 3 && phase == 1) throw Cancelled();
        }));
        CloudType output; bool caught = false;
        try { gicp.align(output); } catch (const Cancelled &) { caught = true; }
        assert(caught && !gicp.hasConverged());
        if (phase == 1) assert(source_checks == 3);
        else assert(target_checks == target_stop);
        if (phase == 2) assert(source_checks >= 2 + 2*int(cloud->size()));
    }
    GICP gicp; configure(gicp, cloud, cloud);
    bool cancelled = false;
    gicp.setSearchMethodTarget(pcl::make_shared<Tree>([&] {
        if (cancelled) throw Cancelled();
    }));
    CloudType output; gicp.align(output); assert(gicp.hasConverged());
    cancelled = true;
    bool caught = false;
    try { gicp.getFitnessScore(); } catch (const Cancelled &) { caught = true; }
    assert(caught);
}

void cancellation_is_caught_inside_each_omp_worker() {
    auto cloud = scene();
    std::atomic<bool> cancelled{false};
    std::atomic<int> checks{0}, started{0}, caught{0}, completed{0};
    #pragma omp parallel for schedule(dynamic,1) num_threads(2)
    for (int candidate = 0; candidate < 10; ++candidate) {
        if (cancelled.load()) continue;
        ++started;
        GICP gicp; configure(gicp, cloud, cloud);
        gicp.setSearchMethodTarget(pcl::make_shared<Tree>([&] {
            if (++checks >= 200) cancelled.store(true);
            if (cancelled.load()) throw Cancelled();
        }));
        CloudType output;
        try { gicp.align(output); ++completed; }
        catch (const Cancelled &) { ++caught; }
    }
    assert(cancelled && started <= 2 && caught == started && completed == 0);
    // A later request uses fresh per-query workers, as the production SC path does.
    GICP retry; configure(retry, cloud, cloud);
    retry.setSearchMethodTarget(pcl::make_shared<Tree>([] {}));
    CloudType output; retry.align(output); assert(retry.hasConverged());
}

struct InspectGICP : GICP {
    bool hasSourceCovariances() const { return bool(input_covariances_); }
    bool hasTargetCovariances() const { return bool(target_covariances_); }
};

void cancelled_persistent_object_clears_partial_caches_and_retries_identically() {
    auto target = scene();
    CloudType::Ptr source(new CloudType);
    M4F shift = M4F::Identity(); shift(0,3) = .025f; shift(1,3) = -.02f;
    pcl::transformPointCloud(*target, *source, shift);
    GICP baseline; configure(baseline, source, target);
    CloudType baseline_output; baseline.align(baseline_output);
    assert(baseline.hasConverged());

    // Target covariance, source covariance, correspondences, and both KD-build
    // boundaries. Every cancellation retries the SAME registration and trees.
    for (int phase = 0; phase < 5; ++phase) {
        InspectGICP gicp; configure(gicp, source, target);
        int target_checks = 0, source_checks = 0;
        bool enabled = true;
        const int target_stop = phase == 0 ? 83 : phase == 2 ?
            2 + 2*int(target->size()) + 83 : phase == 3 ? 1 : 2;
        gicp.setSearchMethodTarget(pcl::make_shared<Tree>([&] {
            ++target_checks;
            if (enabled && phase != 1 && target_checks == target_stop) throw Cancelled();
        }));
        gicp.setSearchMethodSource(pcl::make_shared<Tree>([&] {
            ++source_checks;
            if (enabled && phase == 1 && source_checks == 83) throw Cancelled();
        }));
        CloudType output; bool caught = false;
        const double previous_distance = gicp.getMaxCorrespondenceDistance();
        try {
            ScopedRegistrationState restore(gicp);
            gicp.setMaxCorrespondenceDistance(.7);
            gicp.align(output);
        } catch (const Cancelled &) { caught = true; }
        assert(caught && !gicp.hasSourceCovariances() && !gicp.hasTargetCovariances());
        assert(gicp.getMaxCorrespondenceDistance() == previous_distance);
        enabled = false;
        // Deliberately do not setInputSource/Target: those would independently
        // invalidate the caches and hide a broken cancellation cleanup.
        {
            ScopedRegistrationState restore(gicp);
            gicp.align(output);
        }
        assert(gicp.hasConverged() && gicp.hasSourceCovariances() && gicp.hasTargetCovariances());
        assert((baseline.getFinalTransformation()-gicp.getFinalTransformation()).norm() < 1e-7);
        assert(std::abs(baseline.getFitnessScore()-gicp.getFitnessScore()) < 1e-12);
        assert(output.size() == target->size());
        // A normal scope restores parameters without throwing away valid caches.
        {
            ScopedRegistrationState restore(gicp);
            gicp.setMaxCorrespondenceDistance(.9);
        }
        assert(gicp.getMaxCorrespondenceDistance() == previous_distance);
        assert(gicp.hasSourceCovariances() && gicp.hasTargetCovariances());
    }
}

int main() {
    search_and_registration_match_original_pcl();
    cancellation_before_and_after_tree_build();
    cancellation_inside_original_gicp_loops();
    cancellation_is_caught_inside_each_omp_worker();
    cancelled_persistent_object_clears_partial_caches_and_retries_identically();
    std::cout << "cancellable_search: five real-PCL test groups passed\n";
}
