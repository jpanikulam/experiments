#pragma once

// TODO do better than this
#include "estimation/visualization/visualize_calibration.hh"

namespace estimation {

struct CameraCalibrationConfig {
  bool visualize_camera = false;
  bool visualize_camera_distortion = false;
  bool visualize_camera_frustum = false;
};

void visualize_single_camera_frame(
    const NonlinearCameraModel& model,
    const jcc::Optional<SE3> fiducial_from_camera,
    const std::vector<ObjectImageAssociations> associations,
    const ImageMeasurement& image,
    const std::shared_ptr<viewer::Ui2d> ui2d,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const CameraCalibrationConfig& cfg);

}  // namespace estimation
