#pragma once

#include "localizers/commons.h"
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace tracking {

// A causal initialization input, not a tracking/map accumulator. Every cloud is
// already deskewed in its observation's BODY frame; R,t map that body into the
// SAME odometry epoch. The caller serializes push/build/clear (e.g. message_mutex)
// and clears on rejected inputs or explicit odometry restart. This class cannot
// discover an unannounced restart whose frame names, times and poses look valid.
class InitializationWindow {
public:
    using CloudPtr = CloudType::Ptr;
    using ConstCloudPtr = CloudType::ConstPtr;
    static constexpr std::size_t kMaxFrames = 3;

    enum class Rejection { None, InvalidContext, InvalidInput, OversizedInput };
    struct PushResult {
        bool accepted = false;
        bool reset = false;
        std::size_t evicted_frames = 0;
        Rejection rejection = Rejection::None;
    };

    // Match prepare_adaptive_core_queries.py's >3 s consecutive-gap reset.
    // Three frames therefore span at most 6 s, not 3 s. The point budget bounds
    // retained data plus one build to <=2*max_points*sizeof(PointType), excluding
    // allocator metadata and the caller's current input. Never voxel/sample here.
    explicit InitializationWindow(std::size_t max_points = 180000,
                                  int64_t max_gap_ns = 3000000000LL)
        : max_points_(max_points), max_gap_ns_(max_gap_ns) {
        if (max_points == 0 || max_points > std::numeric_limits<uint32_t>::max() ||
            max_gap_ns <= 0 || max_gap_ns > std::numeric_limits<int64_t>::max() / 2)
            throw std::invalid_argument("Invalid initialization window bounds");
    }

    // Explicit identity is required: equal coordinates alone cannot establish
    // that two body clouds belong to the same sensor frame/odometry epoch.
    bool setContext(const std::string &odom_frame, const std::string &body_frame,
                    uint64_t odom_epoch = 0) {
        const bool valid = !odom_frame.empty() && !body_frame.empty() && odom_frame != body_frame;
        const bool changed = odom_frame != odom_frame_ || body_frame != body_frame_ || odom_epoch != epoch_;
        if (changed || !valid) clear();
        odom_frame_ = odom_frame;
        body_frame_ = body_frame;
        epoch_ = odom_epoch;
        context_valid_ = valid;
        return valid;
    }

    void clear() { frames_.clear(); points_ = 0; }
    std::size_t size() const { return frames_.size(); }
    std::size_t pointCount() const { return points_; }
    int64_t spanNs() const { return frames_.empty() ? 0 : frames_.back().stamp - frames_.front().stamp; }

    PushResult push(const ConstCloudPtr &cloud, const M3D &rotation,
                    const V3D &translation, int64_t stamp_ns) {
        PushResult result;
        if (!context_valid_) return reject(result, Rejection::InvalidContext);
        if (!cloud || cloud->empty() || stamp_ns < 0 || !validPose(rotation, translation) ||
            (!cloud->header.frame_id.empty() && cloud->header.frame_id != body_frame_))
            return reject(result, Rejection::InvalidInput);
        if (cloud->size() > max_points_)
            // Do not truncate a large valid current scan. Tell the caller to use
            // that scan directly, and remove history so it cannot be mixed later.
            return reject(result, Rejection::OversizedInput);
        for (const auto &point : *cloud)
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                return reject(result, Rejection::InvalidInput);

        if (!frames_.empty() && (stamp_ns <= frames_.back().stamp ||
                                stamp_ns - frames_.back().stamp > max_gap_ns_)) {
            clear();
            result.reset = true;
        }
        while (!frames_.empty() && (frames_.size() >= kMaxFrames ||
                                    cloud->size() > max_points_ - points_)) {
            points_ -= frames_.front().cloud->size();
            frames_.pop_front();
            ++result.evicted_frames;
        }
        // Own an immutable snapshot; a mutable source pointer may be reused by
        // the producer after this call. Evict before copying to honor the bound.
        ConstCloudPtr copy(new CloudType(*cloud));
        frames_.push_back(Frame{copy, rotation, translation, stamp_ns});
        points_ += cloud->size();
        result.accepted = true;
        return result;
    }

    // Output is in the CURRENT body frame, with the current observation header.
    // Require its source stamp, rather than relabelling an old window as fresh.
    // Results own their points; a concurrent matcher can keep a result after the
    // executor subsequently clears/replaces the window.
    CloudPtr build(const M3D &rotation, const V3D &translation, int64_t stamp_ns) const {
        if (!context_valid_ || frames_.empty() || frames_.back().stamp != stamp_ns ||
            !validPose(rotation, translation)) return CloudPtr();
        const auto &current = frames_.back();
        // An unnoticed pose replacement at the same stamp must not move history
        // into another epoch. Caller pushes a new context/observation instead.
        if (!rotation.isApprox(current.rotation, 1e-10) ||
            (translation - current.translation).norm() > 1e-9) return CloudPtr();
        CloudPtr output(new CloudType);
        output->header = current.cloud->header;
        output->header.frame_id = body_frame_;
        output->sensor_origin_ = current.cloud->sensor_origin_;
        output->sensor_orientation_ = current.cloud->sensor_orientation_;
        output->reserve(points_);
        for (const auto &frame : frames_) {
            if (&frame == &current) {
                // Current points need no transform; retain their original bits.
                output->insert(output->end(), frame.cloud->begin(), frame.cloud->end());
                continue;
            }
            const M3D relative_rotation = rotation.transpose() * frame.rotation;
            const V3D relative_translation = rotation.transpose() * (frame.translation - translation);
            for (const auto &point : *frame.cloud) {
                PointType transformed = point;  // Preserve intensity and other fields.
                const V3D xyz = relative_rotation * V3D(point.x, point.y, point.z) + relative_translation;
                transformed.x = static_cast<float>(xyz.x());
                transformed.y = static_cast<float>(xyz.y());
                transformed.z = static_cast<float>(xyz.z());
                if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z))
                    return CloudPtr();  // Never publish a partly transformed cloud.
                output->push_back(transformed);
            }
        }
        output->width = static_cast<uint32_t>(output->size());
        output->height = 1;
        output->is_dense = true;
        return output;
    }

private:
    struct Frame {
        ConstCloudPtr cloud;
        M3D rotation;
        V3D translation;
        int64_t stamp;
    };
    static bool validPose(const M3D &rotation, const V3D &translation) {
        return rotation.allFinite() && translation.allFinite() &&
            (rotation.transpose() * rotation).isApprox(M3D::Identity(), 1e-6) &&
            std::abs(rotation.determinant() - 1.) < 1e-6;
    }
    PushResult reject(PushResult result, Rejection reason) {
        clear();
        result.reset = true;
        result.rejection = reason;
        return result;
    }
    std::size_t max_points_, points_ = 0;
    int64_t max_gap_ns_;
    std::string odom_frame_, body_frame_;
    uint64_t epoch_ = 0;
    bool context_valid_ = false;
    std::deque<Frame> frames_;
};

}  // namespace tracking
