#include "gpgpu/demos/ray_lut.hh"

#include "logging/assert.hh"

namespace jcc {

estimation::NonlinearCameraModel make_model() {
  estimation::ProjectionCoefficients proj_coeffs;
  proj_coeffs.fx = 279.76;
  proj_coeffs.fy = 280.034;
  proj_coeffs.cx = 235.992;
  proj_coeffs.cy = 141.951;

  // proj_coeffs.k1 = 0.0669332;
  // proj_coeffs.k2 = -0.224151;
  // proj_coeffs.p1 = 0.00993584;
  // proj_coeffs.p2 = 0.00848696;
  // proj_coeffs.k3 = 0.10179;

  proj_coeffs.k1 = 0.0;
  proj_coeffs.k2 = -0.0;
  proj_coeffs.p1 = 0.0;
  proj_coeffs.p2 = 0.0;
  proj_coeffs.k3 = 0.0;

  proj_coeffs.rows = 270;
  proj_coeffs.cols = 480;

  const estimation::NonlinearCameraModel model(proj_coeffs);
  return model;
}

cv::Mat create_ray_lut(const estimation::NonlinearCameraModel &model,
                       const int cols,
                       const int rows) {
  cv::Mat deprojection_lut(cv::Size(cols, rows), CV_32FC4);
  for (int u = 0; u < cols; ++u) {
    for (int v = 0; v < rows; ++v) {
      const auto optl_ray = model.unproject(jcc::Vec2(u + 0.5, v + 0.5));
      JASSERT(static_cast<bool>(optl_ray), "Need ray");
      const Eigen::Vector3f dir_f = optl_ray->direction.cast<float>();
      deprojection_lut.at<cv::Vec4f>(v, u) =
          cv::Vec4f(dir_f.x(), dir_f.y(), dir_f.z(), 1.0);
    }
  }
  return deprojection_lut;
}
}  // namespace jcc