#pragma once

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "estimation/calibration/nonlinear_camera_model.hh"

namespace estimation {

void visualize_camera_frustum(viewer::SimpleGeometry &geo,
                              viewer::Ui2d &ui2d,
                              const NonlinearCameraModel &model);
}  // namespace estimation