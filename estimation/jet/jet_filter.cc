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

template <typename Meas>
std::function<VecNd<Meas::DIM>(const State&, const Meas&)> bind_parameters(
    const std::function<VecNd<Meas::DIM>(const State&, const Meas&, const Parameters&)>&
        fnc,
    const Parameters& p) {
  const auto bound_fnc = [fnc, p](const State& x, const Meas& z) { return fnc(x, z, p); };
  return bound_fnc;
}

}  // namespace

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

void go() {
  const Parameters p = get_parameters();
  // const SO3 r_vehicle_from_sensor;
  // const jcc::Vec3 t_vehicle_from_sensor = jcc::Vec3(0.0, 0.0, 1.0);
  // const SE3 vehicle_from_sensor = SE3(r_vehicle_from_sensor, t_vehicle_from_sensor);

  // State x;
  // x.eps_ddot[1] = 1.0;
  //

  FilterState<State> xp0;
  xp0.P.setIdentity();
  xp0.x.eps_ddot[0] = 0.0;

  JetFilter jf(xp0);
  AccelMeasurement imu_meas;
  imu_meas.observed_acceleration[0] = 2.0;

  FiducialMeasurement fiducial_meas;

  TimePoint start_time = {};
  for (int k = 0; k < 50; ++k) {
    const TimePoint obs_time = to_duration(k * 0.5) + start_time;
    jf.measure_imu(imu_meas, obs_time);

    jf.measure_fiducial(fiducial_meas, obs_time);

    jf.free_run();

    const auto xp = jf.state();
    std::cout << "eps_dot   : " << xp.x.eps_dot.transpose() << std::endl;
    std::cout << "eps_ddot  : " << xp.x.eps_ddot.transpose() << std::endl;
    std::cout << "accel_bias: " << xp.x.accel_bias.transpose() << std::endl;
    std::cout << "gyro_bias : " << xp.x.gyro_bias.transpose() << std::endl;

    std::cout << "Modelled Error: " << accel_error_model(xp.x, imu_meas, p).transpose()
              << std::endl;
    const auto res = observe_accel(xp.x, get_parameters());
    std::cout << "Expected Accel: " << res.transpose() << std::endl;
  }
}

}  // namespace jet_filter
}  // namespace estimation

// int main() {
//   estimation::jet_filter::go();
// }