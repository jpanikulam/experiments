#include "estimation/jet/jet_filter.hh"

#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

const SE3 fiducial_1_from_world;

Parameters get_parameters() {
  const jcc::Vec3 g(0.0, 0.0, -1.0);

  const SE3 vehicle_from_sensor;
  Parameters p;
  p.T_imu_from_vehicle = vehicle_from_sensor;
  p.g_world = g;
  return p;
}

State dynamics(const State& x, const double h) {
  const Parameters p = get_parameters();
  return rk4_integrate(x, p, h);
}

template <typename Meas>
std::function<VecNd<Meas::DIM>(const State&, const Meas&)> bind_parameters(
    const std::function<VecNd<Meas::DIM>(const State&, const Meas&, const Parameters&)>&
        fnc,
    const Parameters& p) {
  const auto bound_fnc = [fnc, p](const State& x, const Meas& z) { return fnc(x, z, p); };
  return bound_fnc;
}

}  // namespace

FiducialMeasurement observe_fiducial(const State& x, const Parameters& p) {
  FiducialMeasurement meas;
  // const SE3 T_camera_from_world = p.T_camera_from_body * x.T_body_from_world;
  const SE3 T_camera_from_body = SE3();
  const SE3 T_camera_from_world = T_camera_from_body * x.T_body_from_world;

  meas.T_fiducial_from_camera = fiducial_1_from_world * T_camera_from_world.inverse();
  return meas;
}

VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p) {
  /*  const SE3 measured_body_from_fiducial =
        (z.T_fiducial_from_camera * p.T_camera_from_body).inverse();
    const SE3 expected_body_from_fiducial =
        x.T_body_from_world * fiducial_1_from_world.inverse();
    const SE3 error = measured_body_from_fiducial * expected_body_from_fiducial;
  */

  const auto expected_fiducial = observe_fiducial(x, p);
  const SE3 error =
      z.T_fiducial_from_camera * expected_fiducial.T_fiducial_from_camera.inverse();

  return SE3::log(error);
}

JetFilter::JetFilter(const JetFilterState& xp0) : xp_(xp0), ekf_(dynamics) {
  parameters_ = get_parameters();
  imu_id_ = ekf_.add_model(
      bind_parameters<AccelMeasurement>(observe_accel_error_model, parameters_));
  gyro_id_ = ekf_.add_model(
      bind_parameters<GyroMeasurement>(observe_gyro_error_model, parameters_));

  fiducial_id_ = ekf_.add_model(
      bind_parameters<FiducialMeasurement>(fiducial_error_model, parameters_));
}

void JetFilter::measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
  ekf_.measure(meas, t, imu_id_);
}

void JetFilter::measure_gyro(const GyroMeasurement& meas, const TimePoint& t) {
  ekf_.measure(meas, t, gyro_id_);
}

void JetFilter::measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t) {
  ekf_.measure(meas, t, fiducial_id_);
}

void JetFilter::free_run() {
  xp_ = ekf_.service_all_measurements(xp_);
}

}  // namespace jet_filter
}  // namespace estimation
