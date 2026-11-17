#include "rgb_projection_snapshot.h"

namespace fast_livo_rgb {

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
Eigen::Vector3f interpolatedBgr(const cv::Mat &image, const Eigen::Vector2d &pixel) {
  const float u_ref = pixel[0], v_ref = pixel[1];
  const int u_ref_i = floorf(pixel[0]), v_ref_i = floorf(pixel[1]);
  const float subpix_u_ref = u_ref - u_ref_i, subpix_v_ref = v_ref - v_ref_i;
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  const auto *top = image.ptr<std::uint8_t>(v_ref_i) + u_ref_i * 3;
  const auto *bottom = image.ptr<std::uint8_t>(v_ref_i + 1) + u_ref_i * 3;
  const float b = w_ref_tl * top[0] + w_ref_tr * top[3] +
                  w_ref_bl * bottom[0] + w_ref_br * bottom[3];
  const float g = w_ref_tl * top[1] + w_ref_tr * top[4] +
                  w_ref_bl * bottom[1] + w_ref_br * bottom[4];
  const float r = w_ref_tl * top[2] + w_ref_tr * top[5] +
                  w_ref_bl * bottom[2] + w_ref_br * bottom[5];
  return {b, g, r};
}

}  // namespace fast_livo_rgb
