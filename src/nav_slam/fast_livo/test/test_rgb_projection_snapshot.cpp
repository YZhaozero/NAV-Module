#include "rgb_projection_snapshot.h"
#include <vikit/equidistant_camera.h>
#include <sophus/se3.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <list>
#include <random>
#include <string>
using namespace std;
using namespace Eigen;
using Sophus::SE3;
#include "frame.h"

using namespace fast_livo_rgb;
static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

// Exact existing VIOManager::getInterpolatedPixel implementation, including
// its by-value Mat and pixel arguments. The fixture is contiguous like VIO's
// per-frame clones. The ROI check below uses a contiguous copy as this oracle.
static Vector3f originalPixel(cv::Mat img, Vector2d pc, int width) {
  const float u_ref = pc[0], v_ref = pc[1];
  const int u_ref_i = floorf(pc[0]), v_ref_i = floorf(pc[1]);
  const float subpix_u_ref = u_ref - u_ref_i, subpix_v_ref = v_ref - v_ref_i;
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  const auto *img_ptr = img.data + ((v_ref_i)*width + u_ref_i) * 3;
  const float b = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[3] +
                  w_ref_bl * img_ptr[width * 3] + w_ref_br * img_ptr[width * 3 + 3];
  const float g = w_ref_tl * img_ptr[1] + w_ref_tr * img_ptr[4] +
                  w_ref_bl * img_ptr[1 + width * 3] + w_ref_br * img_ptr[width * 3 + 4];
  const float r = w_ref_tl * img_ptr[2] + w_ref_tr * img_ptr[5] +
                  w_ref_bl * img_ptr[2 + width * 3] + w_ref_br * img_ptr[width * 3 + 5];
  return {b, g, r};
}

static vector<XYZRGB> originalColorize(const RgbSnapshot &job, const Frame &frame) {
  vector<XYZRGB> output;
  output.reserve(job.points.size());
  for (const auto &point : job.points) {
    const Vector3d world(point.x, point.y, point.z);
    const Vector3d camera = frame.w2f(world);
    if (camera[2] < 0) continue;
    const Vector2d pixel = frame.w2c(world);
    if (frame.cam_->isInFrame(pixel.cast<int>(), 3)) {
      const Vector3f bgr = originalPixel(job.rgb, pixel, job.camera.width);
      if (camera.norm() > job.blind)
        output.push_back({point.x, point.y, point.z, static_cast<uint8_t>(bgr[2]),
                          static_cast<uint8_t>(bgr[1]), static_cast<uint8_t>(bgr[0])});
    }
  }
  return output;
}

static void equalCloud(const vector<XYZRGB> &a, const vector<XYZRGB> &b) {
  require(a.size() == b.size(), "Colored point count differs");
  for (size_t i = 0; i < a.size(); ++i) {
    const bool same = a[i].x == b[i].x && a[i].y == b[i].y && a[i].z == b[i].z &&
            a[i].r == b[i].r && a[i].g == b[i].g && a[i].b == b[i].b;
    if (!same) {
      cerr << setprecision(17) << "cloud mismatch index=" << i
           << " xyz old=" << a[i].x << ',' << a[i].y << ',' << a[i].z
           << " new=" << b[i].x << ',' << b[i].y << ',' << b[i].z
           << " rgb old=" << int(a[i].r) << ',' << int(a[i].g) << ',' << int(a[i].b)
           << " new=" << int(b[i].r) << ',' << int(b[i].g) << ',' << int(b[i].b) << '\n';
    }
    require(same, "Colored point/order/channel differs");
  }
}

static cv::Mat image(int width, int height) {
  cv::Mat result(height, width, CV_8UC3);
  std::mt19937 rng(128104);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width * 3; ++x) result.ptr<uint8_t>(y)[x] = rng() % 256;
  return result;
}

static void compareCamera(vk::PinholeCamera &camera, bool benchmark) {
  const auto parameters = PinholeParameters::snapshot(camera);
  require(bool(parameters), "Pinhole camera not recognized");
  RgbSnapshot job;
  job.camera = *parameters;
  job.rgb = image(camera.width(), camera.height());
  job.q_camera_world = Quaterniond(AngleAxisd(.38, Vector3d(1., 2., -.5).normalized()));
  job.t_camera_world = {1.3, -.2, .7};
  job.blind = .4;
  Frame frame(&camera, cv::Mat(camera.height(), camera.width(), CV_8UC1, cv::Scalar(0)));
  frame.T_f_w_ = SE3<double>(job.q_camera_world, job.t_camera_world);
  std::mt19937 rng(77123);
  std::uniform_real_distribution<double> unit(0., 1.);
  double max_pixel_error = 0.;
  size_t exact_projection = 0, exact_interpolation = 0, accepted_interpolation = 0;
  for (int i = 0; i < 120000; ++i) {
    const Vector3d point_camera((unit(rng) - .5) * 12., (unit(rng) - .5) * 8.,
                                .1 + unit(rng) * 20.);
    const Vector3d world = frame.T_f_w_.inverse() * point_camera;
    const XYZ point{float(world[0]), float(world[1]), float(world[2])};
    job.points.push_back(point);
    const Vector3d input(point.x, point.y, point.z);
    const Vector3d original_camera = frame.w2f(input);
    const Vector3d snapshot_camera = job.q_camera_world * input + job.t_camera_world;
    require(std::memcmp(original_camera.data(), snapshot_camera.data(), sizeof(double) * 3) == 0,
            "Quaternion snapshot changed Sophus transform");
    const Vector2d original_pixel = frame.w2c(input), new_pixel = job.camera.project(snapshot_camera);
    max_pixel_error = std::max(max_pixel_error, (original_pixel - new_pixel).cwiseAbs().maxCoeff());
    if (std::memcmp(original_pixel.data(), new_pixel.data(), sizeof(double) * 2) == 0) ++exact_projection;
    require((original_pixel - new_pixel).norm() <= 1e-12 * (1. + original_pixel.norm()),
            "Snapshot changed camera projection");
    require(job.camera.contains(new_pixel) == camera.isInFrame(original_pixel.cast<int>(), 3),
            "Snapshot changed frame boundary");
    if (job.camera.contains(new_pixel)) {
      ++accepted_interpolation;
      const auto a = originalPixel(job.rgb, original_pixel, camera.width());
      const auto b = interpolatedBgr(job.rgb, new_pixel);
      if (std::memcmp(a.data(), b.data(), sizeof(float) * 3) == 0) ++exact_interpolation;
      require((a - b).norm() == 0., "Bilinear RGB sample changed");
    }
  }
  const auto expected = originalColorize(job, frame), actual = colorize(job);
  for (size_t i = 0; i < std::min(expected.size(), actual.size()); ++i) {
    const auto &a = expected[i]; const auto &b = actual[i];
    if (a.r == b.r && a.g == b.g && a.b == b.b) continue;
    const Vector3d input(a.x, a.y, a.z);
    const Vector3d old_camera = frame.w2f(input);
    const Vector3d new_camera = job.q_camera_world * input + job.t_camera_world;
    const Vector2d old_pixel = frame.w2c(input), new_pixel = job.camera.project(new_camera);
    const auto old_bgr = originalPixel(job.rgb, old_pixel, camera.width());
    const auto new_bgr = interpolatedBgr(job.rgb, new_pixel);
    cerr << setprecision(17) << "camera scale=" << camera.scale() << " d0=" << camera.d0()
         << " old_camera=" << old_camera.transpose() << " new_camera=" << new_camera.transpose()
         << " old_pixel=" << old_pixel.transpose() << " new_pixel=" << new_pixel.transpose()
         << " old_bgr=" << old_bgr.transpose() << " new_bgr=" << new_bgr.transpose() << '\n';
    break;
  }
  equalCloud(expected, actual);
  cout << "camera d0=" << camera.d0() << " scale=" << camera.scale()
       << " points=" << job.points.size() << " accepted=" << actual.size()
       << " exact_projection=" << exact_projection << " max_pixel_error=" << max_pixel_error
       << " exact_interpolation=" << exact_interpolation << "/" << accepted_interpolation << '\n';
  if (benchmark) {
    volatile size_t checksum = 0;
    const auto start = chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i) checksum += originalColorize(job, frame).size();
    const auto middle = chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i) checksum += colorize(job).size();
    const auto end = chrono::steady_clock::now();
    const double before = chrono::duration<double, milli>(middle - start).count() / 20;
    const double after = chrono::duration<double, milli>(end - middle).count() / 20;
    cout << "kernel_ms original=" << before << " snapshot=" << after
         << " speedup=" << before / after << " checksum=" << checksum << '\n';
  }
}

static void boundariesAndOwnership() {
  std::optional<PinholeParameters> retained_camera;
  Vector2d expected_projection;
  const Vector3d probe(1.2, -.4, 5.);
  {
    vk::PinholeCamera temporary(640, 480, .5, 440., 442., 320., 240., -.08, .12, .001, -.002, .003);
    retained_camera = PinholeParameters::snapshot(temporary);
    expected_projection = temporary.world2cam(probe);
  }
  const auto retained_projection = retained_camera->project(probe);
  require(std::memcmp(expected_projection.data(), retained_projection.data(), sizeof(double) * 2) == 0,
          "Camera snapshot did not survive destruction of the original camera");
  vk::PinholeCamera camera(64, 48, 1., 1., 1., 0., 0.);
  RgbSnapshot job;
  job.camera = *PinholeParameters::snapshot(camera);
  cv::Mat original = image(64, 48);
  job.rgb = original;
  require(job.rgb.data == original.data, "Snapshot copied pixels rather than retaining the frame");
  original.release();
  job.points = {{3,3,1},{2.9999f,3,1},{3,2.9999f,1},{60.9999f,44.9999f,1},
                {61,3,1},{3,45,1},{3,3,-1},{0,0,0},
                {std::numeric_limits<float>::quiet_NaN(),0,1},
                {std::numeric_limits<float>::infinity(),0,1}};
  const auto actual = colorize(job);
  require(actual.size() == 2 && actual[0].x == 3 && actual[1].x == 60.9999f,
          "Border/behind-camera/invalid point rejection differs");
  job.points = {{3,4,1}};
  job.blind = std::sqrt(26.);
  require(colorize(job).empty(), "Blind-distance boundary must be strict");
  job.blind = std::nextafter(job.blind, 0.);
  require(colorize(job).size() == 1, "Point just outside blind-distance boundary rejected");
  job.blind = 0.;
  cv::Mat padded = image(72, 54);
  job.rgb = padded(cv::Rect(4, 3, 64, 48));
  require(!job.rgb.isContinuous(), "ROI fixture did not exercise row stride");
  const auto roi = colorize(job);
  job.rgb = job.rgb.clone();
  equalCloud(roi, colorize(job));
  job.rgb = cv::Mat(48, 64, CV_8UC1);
  bool rejected = false;
  try { (void)colorize(job); } catch (const invalid_argument &) { rejected = true; }
  require(rejected, "Wrong image format was accepted");
}

int main(int argc, char **) {
  const bool benchmark = argc > 1;
  vk::PinholeCamera d435(1280, 720, 1., 904.478, 903.720, 638.396, 364.942);
  compareCamera(d435, benchmark);
  vk::PinholeCamera radial(640, 480, 1., 440., 442., 320., 240., -.08, .12, .001, -.002, .003);
  compareCamera(radial, false);
  vk::PinholeCamera scaled(1280, 720, .5, 904.478, 903.720, 638.396, 364.942);
  compareCamera(scaled, false);
  vk::PinholeCamera d0disabled(640, 480, 1., 440., 442., 320., 240., 0., .12, .01, -.02);
  compareCamera(d0disabled, false);
  vk::EquidistantCamera unsupported(64, 48, 1., 40., 40., 32., 24., .1);
  require(!PinholeParameters::snapshot(unsupported), "Unknown camera silently approximated by pinhole");
  class Derived : public vk::PinholeCamera { public: using PinholeCamera::PinholeCamera; };
  Derived derived(64, 48, 1., 40., 40., 32., 24.);
  require(!PinholeParameters::snapshot(derived), "Derived projection silently approximated");
  boundariesAndOwnership();
  cout << "RGB immutable-snapshot geometry regression passed\n";
}
