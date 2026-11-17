#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

// All transforms use T_parent_child: p_parent = T_parent_child * p_child.
namespace tracking {
inline Eigen::Matrix4f predict(const Eigen::Matrix3d &map_odom_r,
                              const Eigen::Vector3d &map_odom_t,
                              const Eigen::Matrix3d &odom_body_r,
                              const Eigen::Vector3d &odom_body_t) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = (map_odom_r * odom_body_r).cast<float>();
    pose.block<3,1>(0,3) = (map_odom_r * odom_body_t + map_odom_t).cast<float>();
    return pose;
}
inline bool acceptable(const Eigen::Matrix4f &prior, const Eigen::Matrix4f &result,
                       double fitness, double max_score, double translation,
                       double rotation_deg, bool allow_global) {
    if (!result.allFinite() || !std::isfinite(fitness) || fitness < 0 || fitness > max_score)
        return false;
    const auto r = result.block<3,3>(0,0);
    if (std::abs(r.determinant() - 1.) > .01 ||
        !(r.transpose()*r).isApprox(Eigen::Matrix3f::Identity(), .01f)) return false;
    if (allow_global) return true;
    return (result.block<3,1>(0,3)-prior.block<3,1>(0,3)).norm() <= translation &&
        std::abs(Eigen::AngleAxisf(r * prior.block<3,3>(0,0).transpose()).angle()) *
            180. / M_PI <= rotation_deg;
}
// A match may finish after the 1 s launch-freshness window, but it remains
// usable only while new input is still arriving and its source observation is
// inside the caller's bounded lifetime.
inline bool observationFresh(double source_age, double live_data_age,
                             double source_timeout) {
    return std::isfinite(source_age) && std::isfinite(live_data_age) &&
        std::isfinite(source_timeout) && source_timeout > 0. &&
        live_data_age >= -.1 && live_data_age <= 1. &&
        source_age >= -.1 && source_age <= source_timeout;
}
// A global/manual match establishes map->odom at the captured odometry epoch.
// Keep a bounded candidate while the live stream catches up, but do not make
// it navigation-valid until a new point cloud independently confirms it.
inline bool canAwaitFreshScan(double source_age, double live_data_age,
                             double source_timeout, bool initialization) {
    return initialization && std::isfinite(source_age) && std::isfinite(live_data_age) &&
        std::isfinite(source_timeout) && source_timeout > 0. &&
        source_age >= -.1 && source_age <= source_timeout && live_data_age >= -.1;
}
inline bool needsFreshConfirmation(bool initializing, bool confirming,
                                   double source_age, bool age_valid) {
    // Confirmation is itself a new observation. A valid result taking >1 s
    // must not repeatedly demand another confirmation forever.
    return initializing && (!age_valid || (!confirming && source_age > 1.));
}
inline bool valid(bool initialized, double match_age, double data_age,
                  int failures, double timeout, bool pending) {
    return initialized && !pending && failures < 3 &&
        match_age >= 0 && match_age <= timeout && data_age >= -.1 && data_age <= 1.;
}
}
