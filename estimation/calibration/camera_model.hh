#pragma once

#include "eigen.hh"

#include "geometry/shapes/ray.hh"
#include "estimation/calibration/projection_coefficients.hh"

namespace estimation {

class CameraModel {
 public:
  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;
  using ProjMat = Eigen::Matrix<double, 3, 3>;

  CameraModel(const ProjMat& K);

  //
  // @param *: use google pls
  //
  CameraModel(const double f_x, const double f_y, const double u_0, const double v_0);

  // @param camera_point: Point in the camera frame (3d)
  // @returns The point projected into image space
  Vec2 project(const Vec3& camera_point) const;

  // Fire that little guy right back through the image plane!
  //
  // @param image_point: Point in the image frame (2d)
  // @returns ray passing through the image point, originating at the center of projection
  geometry::Ray unproject(const Vec2& image_point) const;

  const ProjMat& get_k() const;

  const Eigen::PartialPivLU<ProjMat>& get_k_inv() const;

 private:
  ProjMat K_;

  // Inversion cache
  Eigen::PartialPivLU<ProjMat> K_inv_;
};
}  // namespace estimation
