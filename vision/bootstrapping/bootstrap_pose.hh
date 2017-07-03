#pragma once

#include <vector>

#include <Eigen/Dense>

#include "out.hh"
#include "sophus.hh"

#include "vision/camera_model.hh"

namespace slam {

struct BootstrapResult {
  using Vec3 = Eigen::Vector3d;
  bool valid;

  double rms_residual;
  // Pose with unit translation
  SE3                 pose;
  std::vector<double> weights;
  std::vector<Vec3>   estimated_structure;
};

class Bootstrapper {
  using Vec3 = Eigen::Vector3d;
  using Vec2 = Eigen::Vector2d;

  // image_points_a and image_points_b must be of the same size
  BootstrapResult compute_nonmetric_pose(const std::vector<Vec2>& image_points_a,
                                         const std::vector<Vec2>& image_points_b,
                                         const CameraModel&       camera_model,
                                         const SE3&               initial_camera_from_object);
};
}