#pragma once

#include "eigen.hh"

#include "estimation/calibration/projection_coefficients.hh"
#include "geometry/shapes/ray.hh"

namespace estimation {

class CameraModel {
 public:
  using ProjMat = Eigen::Matrix<double, 3, 3>;

  CameraModel(const ProjMat& K);

  //
  // @param *: use google pls
  //
  CameraModel(const ProjectionCoefficients& projection_coefficients);

  // @param camera_point: Point in the camera frame (3d)
  // @returns The point projected into image space
  jcc::Vec2 project(const jcc::Vec3& camera_point) const;

  // Fire that little guy right back through the image plane!
  //
  // @param image_point: Point in the image frame (2d)
  // @returns ray passing through the image point, originating at the center of projection
  geometry::Ray unproject(const jcc::Vec2& image_point) const;

  const ProjMat& get_k() const;

  const Eigen::PartialPivLU<ProjMat>& get_k_inv() const;

 private:
  ProjMat K_;

  // Inversion cache
  Eigen::PartialPivLU<ProjMat> K_inv_;
};
}  // namespace estimation
