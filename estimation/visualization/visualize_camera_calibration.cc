#include "estimation/visualization/visualize_camera_calibration.hh"
#include "viewer/window_3d.hh"

namespace estimation {
void visualize_single_camera_frame(
    const NonlinearCameraModel& model,
    const jcc::Optional<SE3> fiducial_from_camera,
    const std::vector<ObjectImageAssociations> associations,
    const ImageMeasurement& image_measurement,
    const std::shared_ptr<viewer::Ui2d> ui2d,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const CameraCalibrationConfig& cfg) {
  const auto view = viewer::get_window3d("Calibration");

  ui2d->clear();
  geo->clear();
  if (fiducial_from_camera) {
    visualize_fiducial_detection(geo, ui2d, model, *fiducial_from_camera, associations);
  } else {
    ui2d->add_image(image_measurement.image, 1.0);
    ui2d->flip();
    view->spin_until_step();
  }

  ui2d->add_image(image_measurement.image, 1.0);
  ui2d->flush();

  if (cfg.visualize_camera_distortion) {
    visualize_camera_distortion(ui2d, model);
  }
  if (cfg.visualize_camera_frustum) {
    visualize_camera_frustum(geo, ui2d, model);
  }

  ui2d->flip();
  geo->flip();

  view->spin_until_step();
}
}  // namespace estimation