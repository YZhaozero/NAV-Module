#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vikit/pinhole_camera.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <vector>

namespace fast_livo_rgb {

struct XYZ { float x, y, z; };
struct XYZRGB { float x, y, z; std::uint8_t r, g, b; };

struct PinholeParameters {
  int width = 0, height = 0;
  double fx = 0., fy = 0., cx = 0., cy = 0.;
  std::array<double, 5> distortion{};
  bool distortion_enabled = false;
  std::shared_ptr<const vk::PinholeCamera> projection;

  static std::optional<PinholeParameters> snapshot(const vk::AbstractCamera &camera) {
    // A derived class could override world2cam(): do not silently approximate
    // a different projection model, even when its base is PinholeCamera.
    if (typeid(camera) != typeid(vk::PinholeCamera)) return std::nullopt;
    const auto &pinhole = static_cast<const vk::PinholeCamera &>(camera);
    PinholeParameters result;
    // Accessors already contain scale; applying camera.scale() again is wrong.
    result.width = pinhole.width(); result.height = pinhole.height();
    result.fx = pinhole.fx(); result.fy = pinhole.fy();
    result.cx = pinhole.cx(); result.cy = pinhole.cy();
    result.distortion = {pinhole.d0(), pinhole.d1(), pinhole.d2(), pinhole.d3(), pinhole.d4()};
    // This is the existing vikit activation rule, including its d0-only test.
    result.distortion_enabled = std::abs(pinhole.d0()) > 0.0000001;
    // Retain an independent immutable camera, including the exact compiled
    // projection implementation. cv::Mat members retain read-only buffers.
    result.projection = std::make_shared<const vk::PinholeCamera>(pinhole);
    result.validate();
    return result;
  }

  void validate() const {
    if (width <= 6 || height <= 6 || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0. || fy <= 0.)
      throw std::invalid_argument("Invalid RGB pinhole snapshot");
    for (double coefficient : distortion)
      if (!std::isfinite(coefficient)) throw std::invalid_argument("Invalid RGB distortion snapshot");
  }

  Eigen::Vector2d project(const Eigen::Vector3d &camera_point) const {
    if (projection) return projection->world2cam(camera_point);
    const Eigen::Vector2d uv = camera_point.head<2>() / camera_point[2];
    if (!distortion_enabled) return {fx * uv[0] + cx, fy * uv[1] + cy};
    const double x = uv[0], y = uv[1];
    const double r2 = x*x + y*y;
    const double r4 = r2*r2;
    const double r6 = r4*r2;
    const double a1 = 2*x*y;
    const double a2 = r2 + 2*x*x;
    const double a3 = r2 + 2*y*y;
    const double cdist = 1 + distortion[0]*r2 + distortion[1]*r4 + distortion[4]*r6;
    const double xd = x*cdist + distortion[2]*a1 + distortion[3]*a2;
    const double yd = y*cdist + distortion[2]*a3 + distortion[3]*a1;
    return {xd*fx + cx, yd*fy + cy};
  }

  bool contains(const Eigen::Vector2d &pixel) const {
    // For the positive, three-pixel boundary this is exactly vikit's
    // isInFrame(pixel.cast<int>(), 3), without casting NaN/huge values to int.
    return pixel.allFinite() && pixel[0] >= 3. && pixel[0] < width - 3. &&
           pixel[1] >= 3. && pixel[1] < height - 3.;
  }
};

struct RgbSnapshot {
  PinholeParameters camera;
  Eigen::Quaterniond q_camera_world = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_camera_world = Eigen::Vector3d::Zero();
  // Retain this independently allocated frame. The producer must not mutate
  // its pixels after submission. VIO already assigns fresh clones per frame.
  cv::Mat rgb;
  std::vector<XYZ> points;
  double blind = 0.;
};

// Shared with synchronous VIO output, with a real function boundary. Otherwise
// ARM FMA contraction can depend on the inlining caller and change uint8 RGB.
Eigen::Vector3f interpolatedBgr(const cv::Mat &image, const Eigen::Vector2d &pixel);

inline std::vector<XYZRGB> colorize(const RgbSnapshot &snapshot) {
  snapshot.camera.validate();
  if (snapshot.rgb.type() != CV_8UC3 || snapshot.rgb.empty() ||
      snapshot.rgb.cols != snapshot.camera.width || snapshot.rgb.rows != snapshot.camera.height)
    throw std::invalid_argument("RGB frame does not match its camera snapshot");
  if (!snapshot.q_camera_world.coeffs().allFinite() || !snapshot.t_camera_world.allFinite() ||
      !std::isfinite(snapshot.blind))
    throw std::invalid_argument("Invalid RGB pose snapshot");
  std::vector<XYZRGB> output;
  output.reserve(snapshot.points.size());
  for (const auto &point : snapshot.points) {
    const Eigen::Vector3d world(point.x, point.y, point.z);
    // Same operation as Sophus::SE3::operator*(Point), once rather than once
    // for Frame::w2f and again inside Frame::w2c. No matrix approximation.
    const Eigen::Vector3d camera = snapshot.q_camera_world * world + snapshot.t_camera_world;
    if (!camera.allFinite() || camera[2] <= 0.) continue;
    const Eigen::Vector2d pixel = snapshot.camera.project(camera);
    if (!snapshot.camera.contains(pixel)) continue;
    const Eigen::Vector3f bgr = interpolatedBgr(snapshot.rgb, pixel);
    if (camera.norm() > snapshot.blind)
      output.push_back({point.x, point.y, point.z,
                        static_cast<std::uint8_t>(bgr[2]),
                        static_cast<std::uint8_t>(bgr[1]),
                        static_cast<std::uint8_t>(bgr[0])});
  }
  return output;
}

}  // namespace fast_livo_rgb
