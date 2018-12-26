#include "estimation/jet/jet_filter.hh"

#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

Parameters get_parameters() {
  const jcc::Vec3 g(0.0, 0.0, 0.0);
  const SE3 vehicle_from_sensor;
  Parameters p;
  p.T_sensor_from_body = vehicle_from_sensor;
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

jcc::Vec3 accel_error_model(const State& x,
                            const AccelMeasurement& z,
                            const Parameters& p) {
  const jcc::Vec3 expected_a_mpss = observe_accel(x, p);
  const jcc::Vec3 error = z.observed_acceleration - expected_a_mpss;
  return error;
}

VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p) {
  const SE3 fiducial_1_from_world;

  const SE3 measured_body_from_fiducial =
      (z.T_fiducial_from_camera * p.T_camera_from_body).inverse();
  // const SE3 error = z.vehicle_from_world * x.T_body_from_world.inverse();

  const SE3 expected_body_from_fiducial =
      x.T_body_from_world * fiducial_1_from_world.inverse();

  const SE3 error = measured_body_from_fiducial * expected_body_from_fiducial;

  return SE3::log(error);
}

JetFilter::JetFilter(const JetFilterState& xp0) : xp_(xp0), ekf_(dynamics) {
  const Parameters p = get_parameters();
  imu_id_ = ekf_.add_model(bind_parameters<AccelMeasurement>(accel_error_model, p));
  fiducial_id_ =
      ekf_.add_model(bind_parameters<FiducialMeasurement>(fiducial_error_model, p));
}

void JetFilter::measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
  ekf_.measure(meas, t, imu_id_);
}

void JetFilter::measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t) {
  ekf_.measure(meas, t, fiducial_id_);
}

void JetFilter::free_run() {
  xp_ = ekf_.service_all_measurements(xp_);
}

}  // namespace jet_filter
}  // namespace estimation
