#pragma once

#include "eigen.hh"

#include "estimation/calibration/camera_model.hh"
#include "geometry/shapes/ray.hh"
#include "util/optional.hh"

namespace estimation {

class NonlinearCameraModel {
 public:
  NonlinearCameraModel() = default;
  NonlinearCameraModel(const ProjectionCoefficients& proj);

  // @param camera_point: Point in the camera frame (3d)
  // @returns The point projected into image space
  //          The optional will be null when the point is outside of the camera image
  jcc::Optional<jcc::Vec2> project(const jcc::Vec3& camera_point) const;

  // Project, but don't wrap in an optional
  // One might use this to visualize projections that fall outside of the image frame
  jcc::Vec2 project_unchecked(const jcc::Vec3& camera_point) const;

  // Fire that little guy right back through the image plane!
  //
  // @param image_point: Point in the image frame (2d)
  // @returns ray passing through the image point, originating at the center of projection
  jcc::Optional<geometry::Ray> unproject(const jcc::Vec2& image_point) const;

  int rows() const;
  int cols() const;

 private:
  CameraModel linear_model_;
  ProjectionCoefficients proj_;
};
}  // namespace estimation
