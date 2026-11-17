#include "../src/initialization_window.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

using Window = tracking::InitializationWindow;
const M3D identity = M3D::Identity();
const V3D zero = V3D::Zero();

CloudType::Ptr scan(std::size_t points, float seed = 1.f, const std::string &frame = "body") {
    CloudType::Ptr cloud(new CloudType);
    cloud->header.frame_id = frame;
    cloud->header.stamp = 1234567;
    for (std::size_t i = 0; i < points; ++i) {
        PointType point;
        point.x = seed + i; point.y = 2.f + i; point.z = -.25f * i; point.intensity = 17.f + i;
        cloud->push_back(point);
    }
    return cloud;
}

void identity_and_ownership() {
    Window window;
    auto cloud = scan(5);
    assert(!window.push(cloud, identity, zero, 0).accepted);
    assert(window.setContext("odom", "body"));
    assert(window.push(cloud, identity, zero, 0).accepted);
    cloud->points[0].x = 999;
    auto output = window.build(identity, zero, 0);
    assert(output && output->size() == 5 && output->points[0].x == 1);
    assert(output->header.frame_id == "body" && output->header.stamp == cloud->header.stamp);
    assert(output->is_dense && output->width == 5 && output->height == 1);
    assert(output->points[3].intensity == cloud->points[3].intensity);
    window.clear();
    assert(window.size() == 0 && window.pointCount() == 0);
    assert(output->size() == 5);  // An in-flight matcher owns its built input.
    assert(!window.build(identity, zero, 0));
}

void full_3d_motion_compensation() {
    Window window;
    window.setContext("odom", "body", 42);
    std::vector<M3D> rotations;
    std::vector<V3D> translations;
    std::vector<CloudType::Ptr> clouds;
    for (int i = 0; i < 3; ++i) {
        rotations.push_back((Eigen::AngleAxisd(.3 * i, V3D::UnitZ()) *
                             Eigen::AngleAxisd(-.2 * i, V3D::UnitY()) *
                             Eigen::AngleAxisd(.1 * i, V3D::UnitX())).toRotationMatrix());
        translations.emplace_back(2. * i, -.3 * i, .5 * i);
        clouds.push_back(scan(7, 1.f + 10.f * i));
        assert(window.push(clouds.back(), rotations.back(), translations.back(), i * 100000000LL).accepted);
    }
    auto output = window.build(rotations.back(), translations.back(), 200000000);
    assert(output && output->size() == 21 && window.size() == 3);
    for (std::size_t frame = 0; frame < 3; ++frame)
        for (std::size_t point = 0; point < 7; ++point) {
            const auto &source = clouds[frame]->points[point];
            const V3D world = rotations[frame] * V3D(source.x, source.y, source.z) + translations[frame];
            const V3D expected = rotations.back().transpose() * (world - translations.back());
            const auto &actual = output->points[frame * 7 + point];
            assert((V3D(actual.x, actual.y, actual.z) - expected).norm() < 2e-5);
            assert(actual.intensity == source.intensity);
        }
    // A changed stamp/pose cannot relabel cached observations as fresh data.
    assert(!window.build(rotations.back(), translations.back(), 200000001));
    assert(!window.build(rotations.back(), translations.back() + V3D(.1, 0, 0), 200000000));
}

void temporal_bounds() {
    Window window;
    window.setContext("odom", "body");
    const auto cloud = scan(4);
    for (const int64_t stamp : {0LL, 3000000000LL, 6000000000LL})
        assert(window.push(cloud, identity, zero, stamp).accepted);
    assert(window.size() == 3 && window.spanNs() == 6000000000LL);
    auto result = window.push(cloud, identity, zero, 9000000001LL);
    assert(result.accepted && result.reset && window.size() == 1);
    result = window.push(cloud, identity, zero, 9000000001LL);
    assert(result.accepted && result.reset && window.size() == 1); // Duplicate resets.
    result = window.push(cloud, identity, zero, 9000000000LL);
    assert(result.accepted && result.reset && window.size() == 1); // Backwards resets.
    // Large valid timestamps are compared without overflowing subtraction.
    result = window.push(cloud, identity, zero, std::numeric_limits<int64_t>::max());
    assert(result.accepted && result.reset && window.size() == 1);
    result = window.push(cloud, identity, zero, -1);
    assert(!result.accepted && window.size() == 0);
}

void context_boundaries() {
    Window window;
    window.setContext("odom", "body", 1);
    const auto cloud = scan(3);
    window.push(cloud, identity, zero, 1);
    assert(window.setContext("odom", "body", 1) && window.size() == 1);
    window.setContext("odom", "body", 2);
    assert(window.size() == 0);  // Same names/time/pose, explicit odometry restart.
    window.push(cloud, identity, zero, 2);
    window.setContext("new_odom", "body", 2);
    assert(window.size() == 0);
    window.push(cloud, identity, zero, 3);
    window.setContext("new_odom", "new_body", 2);
    assert(window.size() == 0);
    const auto wrong_frame = window.push(cloud, identity, zero, 4);
    assert(!wrong_frame.accepted && wrong_frame.rejection == Window::Rejection::InvalidInput);
    assert(!window.setContext("body", "body") && !window.setContext("", "body"));
    assert(!window.push(cloud, identity, zero, 5).accepted);
}

void frame_and_point_budgets() {
    Window window(12);
    window.setContext("odom", "body");
    for (int i = 1; i <= 4; ++i) {
        const auto result = window.push(scan(3, float(i)), identity, zero, i);
        assert(result.accepted && result.evicted_frames == (i == 4 ? 1u : 0u));
    }
    auto output = window.build(identity, zero, 4);
    assert(window.size() == 3 && output->size() == 9 && output->front().x == 2);
    auto result = window.push(scan(8, 50), identity, zero, 5);
    assert(result.accepted && result.evicted_frames == 2 && window.pointCount() == 11);
    output = window.build(identity, zero, 5);
    assert(output->size() == 11 && output->front().x == 4 && output->back().x == 57);
    result = window.push(scan(12, 60), identity, zero, 6);
    assert(result.accepted && result.evicted_frames == 2 && window.pointCount() == 12);
    auto oversized = scan(13, 70);
    result = window.push(oversized, identity, zero, 7);
    assert(!result.accepted && result.rejection == Window::Rejection::OversizedInput);
    assert(window.size() == 0 && oversized->size() == 13 && oversized->back().x == 82);
    // Caller may keep oversized as its full single-frame matcher input.
    assert(window.push(scan(3), identity, zero, 8).accepted && window.size() == 1);
}

void invalid_input_never_partially_joins() {
    Window window;
    window.setContext("odom", "body");
    auto cloud = scan(5);
    window.push(cloud, identity, zero, 1);
    cloud->points[2].z = std::numeric_limits<float>::quiet_NaN();
    assert(!window.push(cloud, identity, zero, 2).accepted && window.size() == 0);
    assert(!window.push(CloudType::ConstPtr(), identity, zero, 2).accepted);
    assert(!window.push(scan(0), identity, zero, 2).accepted);
    assert(!window.push(scan(3), identity * 2., zero, 2).accepted);
    M3D reflection = identity; reflection(0, 0) = -1;
    assert(!window.push(scan(3), reflection, zero, 2).accepted);
    V3D invalid_translation = zero;
    invalid_translation.x() = std::numeric_limits<double>::infinity();
    assert(!window.push(scan(3), identity, invalid_translation, 2).accepted);
    bool threw = false;
    try { Window invalid(0); } catch (const std::invalid_argument &) { threw = true; }
    assert(threw);
    threw = false;
    try { Window invalid(10, 0); } catch (const std::invalid_argument &) { threw = true; }
    assert(threw);
}

void long_sequence_preserves_each_retained_observation() {
    Window window(51);
    window.setContext("odom", "body", 7);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> random(-1., 1.);
    std::deque<M3D> rotations;
    std::deque<V3D> positions;
    std::deque<CloudType::Ptr> clouds;
    for (int frame = 0; frame < 200; ++frame) {
        const M3D rotation = (Eigen::AngleAxisd(random(rng), V3D::UnitZ()) *
                              Eigen::AngleAxisd(random(rng), V3D::UnitX())).toRotationMatrix();
        const V3D position(random(rng), random(rng), random(rng));
        const auto cloud = scan(17, float(frame));
        rotations.push_back(rotation); positions.push_back(position); clouds.push_back(cloud);
        if (clouds.size() > 3) { rotations.pop_front(); positions.pop_front(); clouds.pop_front(); }
        const auto pushed = window.push(cloud, rotation, position, frame * 100000000LL);
        assert(pushed.accepted && !pushed.reset);
        assert(window.size() <= 3 && window.pointCount() <= 51);
        const auto output = window.build(rotation, position, frame * 100000000LL);
        assert(output && output->size() == 17 * clouds.size());
        for (std::size_t i = 0; i < clouds.size(); ++i)
            for (std::size_t p = 0; p < 17; ++p) {
                const auto &input = clouds[i]->points[p];
                const V3D expected = rotation.transpose() *
                    (rotations[i] * V3D(input.x, input.y, input.z) + positions[i] - position);
                const auto &point = output->points[i * 17 + p];
                assert((expected - V3D(point.x, point.y, point.z)).norm() < 3e-5);
                assert(point.intensity == input.intensity);
            }
    }
}

int main() {
    identity_and_ownership();
    full_3d_motion_compensation();
    temporal_bounds();
    context_boundaries();
    frame_and_point_budgets();
    invalid_input_never_partially_joins();
    long_sequence_preserves_each_retained_observation();
    std::cout << "initialization_window: seven test groups passed\n";
}
