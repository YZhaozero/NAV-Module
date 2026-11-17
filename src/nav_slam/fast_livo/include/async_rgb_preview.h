#pragma once
#include "rgb_projection_snapshot.h"
#include "latest_output_worker.h"
#include "map_preview.h"
#include <builtin_interfaces/msg/time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fast_livo_rgb {

using GeometryCloud = pcl::PointCloud<pcl::PointXYZINormal>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGB>;

inline bool asynchronousPreviewAllowed(bool pcd_save, bool image_save, bool colmap_save,
                                       bool supported_camera) {
  // Replacement is only a display policy. Research exports retain their
  // existing synchronous order and must never inherit a latest-wins queue.
  return !pcd_save && !image_save && !colmap_save && supported_camera;
}

// Producer-owned recent display geometry. Measured map points are archived by
// the separate NavigationOutput FIFO before this helper is called.
class GeometryWindow {
 public:
  using Blocks = std::vector<GeometryCloud::ConstPtr>;
  explicit GeometryWindow(std::size_t byte_limit = 16 * 1024 * 1024,
                          std::size_t block_limit = 128)
      : byte_limit_(byte_limit), block_limit_(block_limit) {
    if (!byte_limit || !block_limit) throw std::invalid_argument("Invalid RGB window budget");
  }
  void begin(double stamp, std::uint64_t generation, const std::string &world_frame) {
    if (start_ < 0 || stamp < start_ || stamp - start_ > .5 ||
        generation != generation_ || world_frame != world_frame_) {
      clear(); start_ = stamp; generation_ = generation; world_frame_ = world_frame;
    }
  }
  void append(GeometryCloud::ConstPtr cloud) {
    if (!cloud || cloud->empty()) return;
    const auto bytes = cloud->size() * sizeof(GeometryCloud::PointType);
    // A single oversized input already exists in the estimator; retaining its
    // shared ownership adds no point copy. Never retain a second large block.
    while (!blocks_.empty() && (blocks_.size() >= block_limit_ ||
           bytes > byte_limit_ || bytes_ > byte_limit_ - bytes)) {
      bytes_ -= blocks_.front()->size() * sizeof(GeometryCloud::PointType);
      blocks_.erase(blocks_.begin()); ++discarded_preview_blocks_;
    }
    blocks_.push_back(std::move(cloud)); bytes_ += bytes;
  }
  Blocks take() {
    Blocks result; result.swap(blocks_); bytes_ = 0; start_ = -1.; return result;
  }
  void clear() { blocks_.clear(); bytes_ = 0; start_ = -1.; }
  bool empty() const { return blocks_.empty(); }
  std::size_t bytes() const { return bytes_; }
  std::size_t blocks() const { return blocks_.size(); }
  std::uint64_t discardedPreviewBlocks() const { return discarded_preview_blocks_; }
 private:
  const std::size_t byte_limit_, block_limit_;
  Blocks blocks_;
  std::size_t bytes_ = 0;
  double start_ = -1.;
  std::uint64_t generation_ = 0, discarded_preview_blocks_ = 0;
  std::string world_frame_;
};

struct PreviewInput {
  PinholeParameters camera;
  Eigen::Quaterniond q_camera_world = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_camera_world = Eigen::Vector3d::Zero();
  // VIO assigns freshly cloned Mats each frame and does not mutate a previous
  // frame after submission. Their refcounts retain those immutable buffers.
  cv::Mat rgb, annotated;
  GeometryWindow::Blocks geometry;
  builtin_interfaces::msg::Time stamp;
  std::string world_frame;
  std::uint64_t generation = 0;
  double blind = 0.;
  bool colorize_cloud = true;
};

struct PreviewResult {
  ColorCloud::ConstPtr cloud;
  cv::Mat annotated;
  builtin_interfaces::msg::Time stamp;
  std::string world_frame;
  std::uint64_t generation = 0;
};

class AsyncRgbPreview {
 public:
  using Publish = std::function<void(const PreviewResult &)>;
  using Generation = std::function<std::uint64_t()>;
  using Failure = std::function<void(std::exception_ptr)>;
  AsyncRgbPreview(Publish publish, Generation generation, Failure failure = {})
      : publish_(std::move(publish)), generation_(std::move(generation)),
        failure_(std::move(failure)), worker_([this](const auto &input) {
          try { process(*input); }
          catch (...) { if (failure_) failure_(std::current_exception()); else throw; }
        }) {
    if (!publish_ || !generation_) throw std::invalid_argument("Invalid RGB output callbacks");
  }
  ~AsyncRgbPreview() { stop(); }
  AsyncRgbPreview(const AsyncRgbPreview &) = delete;
  AsyncRgbPreview &operator=(const AsyncRgbPreview &) = delete;
  bool submit(std::shared_ptr<const PreviewInput> input) { return worker_.submit(std::move(input)); }
  void stop() noexcept { worker_.stop(false); }
  auto stats() const { return worker_.stats(); }
  void rethrowIfFailed() const { worker_.rethrowIfFailed(); }

 private:
  void process(const PreviewInput &input) {
    if (input.generation != generation_()) return;
    auto colored = std::make_shared<ColorCloud>();
    if (input.colorize_cloud) {
      MapPreview window(.03, 120000);
      for (const auto &block : input.geometry)
        for (const auto &point : *block) window.add({point.x, point.y, point.z});
      RgbSnapshot snapshot;
      snapshot.camera = input.camera;
      snapshot.q_camera_world = input.q_camera_world;
      snapshot.t_camera_world = input.t_camera_world;
      snapshot.rgb = input.rgb;
      snapshot.blind = input.blind;
      snapshot.points.reserve(window.points().size());
      for (const auto &entry : window.points()) {
        const auto &point = entry.second;
        snapshot.points.push_back({point.x, point.y, point.z});
      }
      const auto pixels = colorize(snapshot);
      colored->reserve(pixels.size());
      for (const auto &point : pixels) {
        pcl::PointXYZRGB out{};
        out.x = point.x; out.y = point.y; out.z = point.z;
        out.r = point.r; out.g = point.g; out.b = point.b;
        colored->push_back(out);
      }
    }
    if (input.generation != generation_()) return;
    publish_(PreviewResult{colored, input.annotated, input.stamp,
                           input.world_frame, input.generation});
  }
  Publish publish_;
  Generation generation_;
  Failure failure_;
  fast_livo_output::LatestOutputWorker<PreviewInput> worker_;
};

}  // namespace fast_livo_rgb
