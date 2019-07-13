#include "estimation/jet/fiducial_measurement.hh"

namespace estimation {
namespace jet_filter {

FiducialMeasurement observe_fiducial(const State& x, const Parameters& p) {
  const SE3 fiducial_1_from_world = p.T_world_from_fiducial.inverse();
  const SE3 T_camera_from_body = p.T_camera_from_vehicle;
  const SE3 T_world_from_body = SE3(x.R_world_from_body, x.x_world);
  const SE3 T_camera_from_world = T_camera_from_body * T_world_from_body.inverse();
  const SE3 T_fiducial_from_camera = fiducial_1_from_world * T_camera_from_world.inverse();

  const FiducialMeasurement meas{.fiducial_id = -1,
                                 .T_fiducial_from_camera = T_fiducial_from_camera};
  return meas;
}

VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p) {
  const auto expected_fiducial = observe_fiducial(x, p);
  const SE3 error_observed_fiducial_from_expected_fiducial =
      z.T_fiducial_from_camera * expected_fiducial.T_fiducial_from_camera.inverse();

  // This is `v` in  `v = z - h(x)`
  return SE3::log(error_observed_fiducial_from_expected_fiducial);
}
}  // namespace jet_filter
}  // namespace estimation