#pragma once

#include <functional>
#include <exception>
#include <utility>
#include <type_traits>
#include <pcl/search/kdtree.h>

// PCL GICP builds a covariance for every map point inside align(). Checking
// only before/after align leaves that entire loop uninterruptible. All those
// queries use this virtual interface, so retain PCL's exact tree, neighbours
// and covariance formulas while checking cancellation at each query boundary.
// The caller must catch its checkpoint exception inside the same OMP worker.
template <class PointT>
class CancellableSearch : public pcl::search::KdTree<PointT> {
public:
    using Base = pcl::search::KdTree<PointT>;
    using CloudConstPtr = typename Base::PointCloudConstPtr;
    using InputResult = decltype(std::declval<Base &>().setInputCloud(
        std::declval<CloudConstPtr>(), std::declval<pcl::IndicesConstPtr>()));
    using Base::nearestKSearch;
    using Base::radiusSearch;

    explicit CancellableSearch(std::function<void()> checkpoint)
        : checkpoint_(std::move(checkpoint)) {}

    InputResult setInputCloud(const CloudConstPtr &cloud,
                             const pcl::IndicesConstPtr &indices = {}) override {
        check();
        // PCL 1.14 returns bool; Humble's older PCL returns void. Preserve
        // either contract and both cancellation checkpoints.
        return setInputChecked<InputResult>(cloud, indices);
    }

    int nearestKSearch(const PointT &point, int count, pcl::Indices &indices,
                       std::vector<float> &distances) const override {
        check();
        const int found = Base::nearestKSearch(point, count, indices, distances);
        check();
        return found;
    }

    int radiusSearch(const PointT &point, double radius, pcl::Indices &indices,
                     std::vector<float> &distances,
                     unsigned int max_count = 0) const override {
        check();
        const int found = Base::radiusSearch(point, radius, indices, distances, max_count);
        check();
        return found;
    }

private:
    template <class Result>
    typename std::enable_if<std::is_void<Result>::value, Result>::type
    setInputChecked(const CloudConstPtr &cloud, const pcl::IndicesConstPtr &indices) {
        Base::setInputCloud(cloud, indices);
        check();
    }

    template <class Result>
    typename std::enable_if<!std::is_void<Result>::value, Result>::type
    setInputChecked(const CloudConstPtr &cloud, const pcl::IndicesConstPtr &indices) {
        const auto accepted = Base::setInputCloud(cloud, indices);
        check();
        return accepted;
    }

    void check() const { if (checkpoint_) checkpoint_(); }
    std::function<void()> checkpoint_;
};

// GICP allocates its full covariance vector before filling individual entries.
// If a search checkpoint interrupts that loop, PCL otherwise treats the partly
// filled, non-empty vector as a valid cache on the next align() of this object.
// This scope also restores temporary correspondence distances on every exit.
template <class Registration>
class ScopedRegistrationState {
public:
    explicit ScopedRegistrationState(Registration &registration)
        : registration_(registration),
          distance_(registration.getMaxCorrespondenceDistance()),
          exceptions_(std::uncaught_exceptions()) {}
    ScopedRegistrationState(const ScopedRegistrationState &) = delete;
    ScopedRegistrationState &operator=(const ScopedRegistrationState &) = delete;
    ~ScopedRegistrationState() noexcept {
        registration_.setMaxCorrespondenceDistance(distance_);
        if (std::uncaught_exceptions() > exceptions_) {
            registration_.setSourceCovariances({});
            registration_.setTargetCovariances({});
        }
    }
private:
    Registration &registration_;
    double distance_;
    int exceptions_;
};
