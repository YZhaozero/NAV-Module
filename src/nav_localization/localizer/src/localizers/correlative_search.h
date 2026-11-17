#pragma once
#include "height_profile.h"
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

// Gravity-aligned global proposals. No indoor/outdoor flag and no pose prior.
// Full 3-D registration and the node's new-scan checks remain authoritative.
// Like the owning ICP instance, this cache is used by one matching worker.
class CorrelativeSearch {
 public:
  using Cloud = height_profile::Cloud;
  using Band = height_profile::Band;
  struct Proposal { Eigen::Matrix4f pose; float score; size_t height_family = 0; };
  struct Result {
    std::vector<Proposal> proposals;
    bool unique = false;
    bool refined_grid = false;
    bool height_normalized = false;
    float gap = 0;
    float resolution = .5f;
    size_t layer_count = 0;
    size_t height_hypotheses = 0;
  };

  void clear() { map_.clear(); cache_.clear(); profile_ = {}; }
  void build(const Cloud& map) {
    clear();
    lo_ = Eigen::Vector2f::Constant(INFINITY);
    hi_ = -lo_;
    for (const auto& p : map) {
      if (!p.getVector3fMap().allFinite()) continue;
      map_.push_back(p);
      lo_ = lo_.cwiseMin(Eigen::Vector2f(p.x, p.y));
      hi_ = hi_.cwiseMax(Eigen::Vector2f(p.x, p.y));
    }
    if (!map_.empty()) profile_ = height_profile::describe(map_);
  }

  Result searchWithDiagnostics(const Cloud& query, const Eigen::Matrix3f& level,
      const std::function<void()>& checkpoint = {}) const {
    if (checkpoint) checkpoint();
    Result result;
    if (map_.empty() || query.size() < 20 || !level.allFinite() ||
        !((level.transpose() * level).isApprox(Eigen::Matrix3f::Identity(), .01f)) ||
        std::abs(level.determinant() - 1.f) > .01f) return result;
    Cloud leveled;
    for (const auto& p : query) {
      if (!p.getVector3fMap().allFinite()) continue;
      const Eigen::Vector3f v = level * p.getVector3fMap();
      if (v.head<2>().norm() > 50.f) continue;
      pcl::PointXYZI q;
      q.getVector3fMap() = v;
      leveled.push_back(q);
    }
    if (leveled.size() < 20) return result;
    const auto query_profile = height_profile::describe(leveled);
    if (checkpoint) checkpoint();
    result.height_normalized = profile_.has_floor && query_profile.has_floor;
    auto bands = height_profile::layers(query_profile, result.height_normalized);
    for (auto& b : bands) for (float& z : b) z = std::round(z * 20.f) / 20.f;
    result.layer_count = bands.size();
    const float query_floor = result.height_normalized ? query_profile.floor : 0.f;
    std::vector<float> floors{result.height_normalized ? profile_.floor : 0.f};
    if (result.height_normalized) {
      // Search each distinct vertical structure once. Nearby elevations in
      // disjoint ground patches are already the same candidate family; GICP
      // resolves their local Z. Repeating a whole-map search for each patch
      // only duplicates the same basin. Overlapping storeys remain separate.
      std::set<size_t> families;
      for (const auto& p : profile_.horizontal)
        if (std::abs(p.z - profile_.floor) < 1e-4f) families.insert(p.family);
      for (const auto& p : profile_.horizontal) {
        if (families.insert(p.family).second)
          floors.push_back(p.z);
      }
    }
    result.height_hypotheses = floors.size();
    float radius = 1.f;
    for (auto& p : leveled) {
      radius = std::max(radius, p.getVector3fMap().head<2>().norm());
      p.z -= query_floor;
    }
    for (float requested_resolution : {.5f, .25f}) {
      const float resolution = std::max(requested_resolution, (hi_ - lo_).maxCoeff() / 1020.f);
      if (requested_resolution < .5f && resolution >= result.resolution - 1e-6f) break;
      std::vector<Proposal> all;
      if (requested_resolution == .5f) {
        for (float floor : floors) {
          if (checkpoint) checkpoint();
          const Grid& grid = gridFor(bands, floor, resolution);
          auto proposals = correlate(leveled, level, query_floor, grid, radius, nullptr, checkpoint);
          all.insert(all.end(), proposals.begin(), proposals.end());
        }
      } else {
        // Fine search preserves every coarse competing basin, but does not
        // repeat a fine-resolution FFT across the whole map and all headings.
        for (const auto& seed : result.proposals) {
          if (checkpoint) checkpoint();
          const Grid& grid = gridFor(bands, seed.pose(2,3)+query_floor, resolution);
          auto proposals = correlate(leveled, level, query_floor, grid, radius, &seed, checkpoint);
          all.insert(all.end(), proposals.begin(), proposals.end());
        }
      }
      result.proposals = distinct(all);
      result.resolution = resolution;
      result.refined_grid = requested_resolution < .5f;
      result.gap = result.proposals.size() > 1 ? result.proposals[0].score - result.proposals[1].score : 0.f;
      result.unique = result.proposals.size() > 1 && result.gap >= .05f &&
          (!result.height_normalized || profile_.complete);
      if (result.unique || result.proposals.empty()) break;
    }
    return result;
  }

  std::vector<Proposal> search(const Cloud& query, const Eigen::Matrix3f& level) const {
    return searchWithDiagnostics(query, level).proposals;
  }

 private:
  struct Grid {
    float floor = 0, resolution = .5f;
    size_t height_family = 0;
    Eigen::Vector2f origin;
    std::vector<Band> bands;
    std::vector<cv::Mat> layers;
    bool populated = false;
  };
  const Grid& gridFor(const std::vector<Band>& bands, float floor, float resolution) const {
    for (const auto& g : cache_)
      if (g.floor == floor && g.resolution == resolution && g.bands == bands) return g;
    Grid g;
    g.floor = floor; g.resolution = resolution; g.bands = bands;
    for(const auto& p:profile_.horizontal)if(std::abs(p.z-floor)<1e-4f)g.height_family=p.family;
    g.origin = (lo_ / resolution).array().floor().matrix() * resolution - Eigen::Vector2f::Constant(2 * resolution);
    const Eigen::Vector2i size = ((hi_ - g.origin) / resolution).array().ceil().cast<int>() + 4;
    for (const auto& band : bands) {
      cv::Mat occupied = cv::Mat::ones(size.y(), size.x(), CV_8U);
      bool populated = false;
      for (const auto& p : map_) if (p.z - floor > band[0] && p.z - floor < band[1]) {
        const int x = std::floor((p.x - g.origin.x()) / resolution);
        const int y = std::floor((p.y - g.origin.y()) / resolution);
        occupied.at<unsigned char>(y, x) = 0;
        populated = true;
      }
      cv::Mat distance, score = cv::Mat::zeros(occupied.size(), CV_32F);
      if (populated) {
        g.populated = true;
        cv::distanceTransform(occupied, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        cv::exp(-.5 * distance.mul(distance), score);
      }
      g.layers.push_back(score);
    }
    if (cache_.size() >= 2) cache_.erase(cache_.begin());
    cache_.push_back(std::move(g));
    return cache_.back();
  }

  static std::vector<Proposal> correlate(const Cloud& points, const Eigen::Matrix3f& level,
      float query_floor, const Grid& grid, float radius, const Proposal* seed=nullptr,
      const std::function<void()>& checkpoint = {}) {
    if (!grid.populated) return {};
    const float resolution = grid.resolution;
    const int center = std::ceil(radius / resolution), side = 2 * center + 1;
    cv::Rect window(0,0,grid.layers[0].cols,grid.layers[0].rows);
    float start=-180,end=180,step=3;
    if(seed) {
      const int x=std::lround((seed->pose(0,3)-grid.origin.x())/resolution);
      const int y=std::lround((seed->pose(1,3)-grid.origin.y())/resolution);
      const int range=std::ceil(3.f/resolution);
      window=cv::Rect(x-range,y-range,2*range+1,2*range+1)&window;
      const Eigen::Matrix3f yaw_rotation=seed->pose.block<3,3>(0,0)*level.transpose();
      const float yaw=std::atan2(yaw_rotation(1,0),yaw_rotation(0,0))*180/M_PI;
      start=yaw-6;end=yaw+6.01f;step=1.5f;
    }
    if(window.empty())return {};
    std::vector<cv::Mat> padded;
    for (const auto& layer : grid.layers) {
      cv::Mat p;
      cv::copyMakeBorder(layer, p, center, center, center, center, cv::BORDER_CONSTANT, 0);
      padded.push_back(p(cv::Rect(window.x,window.y,window.width+2*center,window.height+2*center)));
    }
    std::vector<Proposal> all;
    for (float degrees = start; degrees < end; degrees += step) {
      if (checkpoint) checkpoint();
      const float yaw = degrees * M_PI / 180., c = std::cos(yaw), s = std::sin(yaw);
      cv::Mat heat = cv::Mat::zeros(window.size(), CV_32F);
      int used = 0;
      for (size_t b = 0; b < grid.bands.size(); ++b) {
        if (checkpoint) checkpoint();
        cv::Mat source = cv::Mat::zeros(side, side, CV_32F);
        for (const auto& p : points) if (p.z > grid.bands[b][0] && p.z < grid.bands[b][1]) {
          const int x = std::lround((c * p.x - s * p.y) / resolution) + center;
          const int y = std::lround((s * p.x + c * p.y) / resolution) + center;
          if (x >= 0 && x < side && y >= 0 && y < side) source.at<float>(y, x) = 1.f;
        }
        const double count = cv::sum(source)[0];
        if (count < 15) continue;
        cv::Mat correlation;
        cv::matchTemplate(padded[b], source, correlation, cv::TM_CCORR);
        heat += correlation / count;
        ++used;
      }
      if (!used) continue;
      heat /= used;
      for (int peak = 0; peak < 4; ++peak) {
        double score;
        cv::Point at;
        cv::minMaxLoc(heat, nullptr, &score, nullptr, &at);
        if (score < .3) break;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * level;
        pose(0, 3) = grid.origin.x() + (window.x+at.x) * resolution;
        pose(1, 3) = grid.origin.y() + (window.y+at.y) * resolution;
        pose(2, 3) = grid.floor - query_floor;
        all.push_back({pose, static_cast<float>(score), grid.height_family});
        cv::circle(heat, at, std::max(1, int(std::ceil(3. / resolution))), cv::Scalar(0), cv::FILLED);
      }
    }
    return all;
  }

  static std::vector<Proposal> distinct(std::vector<Proposal>& all) {
    std::stable_sort(all.begin(), all.end(), [](const Proposal& a, const Proposal& b) { return a.score > b.score; });
    std::vector<Proposal> selected;
    for (const auto& p : all) {
      bool separate = true;
      for (const auto& old : selected) {
        const float distance = (p.pose.block<2, 1>(0, 3) - old.pose.block<2, 1>(0, 3)).norm();
        const float height = std::abs(p.pose(2, 3) - old.pose(2, 3));
        const float angle = Eigen::AngleAxisf(p.pose.block<3, 3>(0, 0) * old.pose.block<3, 3>(0, 0).transpose()).angle();
        if (distance <= 3.f && (height <= .4f || p.height_family==old.height_family) &&
            std::abs(angle) <= 12 * M_PI / 180.) { separate = false; break; }
      }
      if (separate) selected.push_back(p);
      if (selected.size() == 6) break;
    }
    return selected;
  }
  Cloud map_;
  height_profile::Profile profile_;
  Eigen::Vector2f lo_, hi_;
  mutable std::vector<Grid> cache_;
};
