#include "estimation/jet/bootstrap_jet.hh"

#include "geometry/spatial/form_coordinate_frame.hh"

namespace estimation {
namespace jet_filter {

namespace {

Parameters construct_jet_parameters(const geometry::TransformNetwork& tfn) {
  auto p = JetFilter::reasonable_parameters();
  {
    p.T_world_from_fiducial = tfn.find_source_from_destination("world", "fiducial");
    p.T_imu1_from_vehicle = tfn.find_source_from_destination("imu_78", "vehicle");
    p.T_imu2_from_vehicle = tfn.find_source_from_destination("imu_36", "vehicle");
    p.T_camera_from_vehicle = tfn.find_source_from_destination("camera", "vehicle");
  }

  return p;
}
}  // namespace

//
// Setup parameters
//  - Get fiducial_from_world
//  - Load camera/imu* <- vehicle
//
// Establish world-frame
//  - Establish initial pose
//
BootstrapResult bootstrap_jet(const geometry::Unit3& g_imu_frame,
                              const geometry::TransformNetwork& tfn,
                              const jcc::TimePoint& t0) {
  BootstrapResult result;
  const geometry::Unit3 g_vehicle_frame =
      tfn.transform_a_from_b("vehicle", "imu_78", g_imu_frame);

  const SE3 vehicle_from_fiducial =
      tfn.find_source_from_destination("vehicle", "fiducial");

  const jcc::Vec3 fiducial_vehicle_frame = vehicle_from_fiducial.translation();
  const SO3 world_from_vehicle = geometry::spatial::form_coordinate_frame_from_zhat_and_x(
      g_vehicle_frame, fiducial_vehicle_frame);

  geometry::TransformNetwork tfn_tmp = tfn;
  tfn_tmp.add_edge("world", "vehicle", SE3(world_from_vehicle, jcc::Vec3::Zero()));

  const auto parameters = construct_jet_parameters(tfn_tmp);

  auto xp0 = JetFilter::reasonable_initial_state(t0);
  {
    const SE3 world_from_vehicle =
        tfn_tmp.find_source_from_destination("world", "vehicle");

    xp0.x.R_world_from_body = world_from_vehicle.so3();
    xp0.x.x_world = world_from_vehicle.translation();
  }

  return BootstrapResult{.xp0 = xp0,  //
                         .parameters = parameters};
}

}  // namespace jet_filter

}  // namespace estimation