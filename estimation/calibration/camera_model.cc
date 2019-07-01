#include "estimation/calibration/camera_model.hh"

namespace estimation {

CameraModel::CameraModel(const ProjMat& K) : K_(K) {
  K_inv_ = Eigen::PartialPivLU<ProjMat>(K_);
};

CameraModel::CameraModel(const ProjectionCoefficients& projection_coefficients) {
  K_.setZero();
  K_(0, 0) = projection_coefficients.fx;
  K_(1, 1) = projection_coefficients.fy;
  K_(0, 2) = projection_coefficients.cx;
  K_(1, 2) = projection_coefficients.cy;
  K_(2, 2) = 1.0;
  K_inv_ = Eigen::PartialPivLU<ProjMat>(K_);
}

// @param camera_point: Point in the camera frame (3d)
// @returns The point projected into image space
jcc::Vec2 CameraModel::project(const jcc::Vec3& camera_point) const {
  const jcc::Vec3 projected_h = K_ * camera_point;
  const jcc::Vec2 projected = projected_h.head<2>() / projected_h(2);
  return projected;
}

// Fire that little guy right back through the image plane!
//
// @param image_point: Point in the image frame (2d)
// @returns ray passing through the image point, originating at the center of projection
geometry::Ray CameraModel::unproject(const jcc::Vec2& image_point) const {
  const Eigen::PartialPivLU<ProjMat>& K_inv = get_k_inv();

  const jcc::Vec3 image_point_h = jcc::Vec3(image_point.x(), image_point.y(), 1.0);
  const jcc::Vec3 soln = K_inv.solve(image_point_h);
  const jcc::Vec3 unprojected = soln;
  const geometry::Ray unprojected_ray{.origin = jcc::Vec3::Zero(),
                                      .direction = unprojected.normalized()};
  return unprojected_ray;
}

const CameraModel::ProjMat& CameraModel::get_k() const {
  return K_;
}

const Eigen::PartialPivLU<CameraModel::ProjMat>& CameraModel::get_k_inv() const {
  return K_inv_;
}

}  // namespace estimation
