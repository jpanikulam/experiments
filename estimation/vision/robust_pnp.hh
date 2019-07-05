#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "estimation/calibration/nonlinear_camera_model.hh"

namespace estimation {

struct SolvePnpResult {
  SE3 camera_from_object;
  std::vector<double> weights;
  jcc::Vec2 average_error;
  bool success = false;
};

// TODO use this here
struct ObjectImageAssociations {
  jcc::Vec2 point_image;
  jcc::Vec3 point_object_frame;
};

using SolvePnpVisitor = std::function<void(const SolvePnpResult& result)>;
SolvePnpVisitor make_pnp_visitor(const estimation::NonlinearCameraModel& model,
                                 const std::vector<jcc::Vec3>& points_object_frame,
                                 const std::vector<jcc::Vec2>& image_points);

SolvePnpResult robust_pnp(const NonlinearCameraModel& model,
                          const SE3& object_from_camera_init,
                          const std::vector<jcc::Vec3>& points_object_frame,
                          const std::vector<jcc::Vec2>& image_points,
                          const SolvePnpVisitor& visitor);

}  // namespace estimation
