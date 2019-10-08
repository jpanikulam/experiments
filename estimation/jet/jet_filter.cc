#include "estimation/jet/jet_filter.hh"

#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"
#include "numerics/set_diag_to_value.hh"

#include "logging/assert.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

auto make_dynamics(const Parameters& p) {
  return [p](const State& x, const double h) {
    //
    return rk4_integrate(x, p, h);
  };
}

template <typename Meas>
std::function<VecNd<Meas::DIM>(const State&, const Meas&)> bind_parameters(
    const std::function<VecNd<Meas::DIM>(const State&, const Meas&, const Parameters&)>&
        fnc,
    const Parameters& p) {
  const auto bound_fnc = [fnc, p](const State& x, const Meas& z) { return fnc(x, z, p); };
  return bound_fnc;
}

MatNd<State::DIM, State::DIM> make_cov() {
  MatNd<State::DIM, State::DIM> state_cov;
  state_cov.setZero();
  numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                              StateDelta::accel_bias_error_ind>(state_cov, 0.0001);
  numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                              StateDelta::gyro_bias_error_ind>(state_cov, 0.0001);

  numerics::set_diag_to_value<3, StateDelta::eps_dot_error_ind>(state_cov, 0.001);

  numerics::set_diag_to_value<3, StateDelta::eps_dot_error_ind + 3>(state_cov, 0.001);

  numerics::set_diag_to_value<3, StateDelta::eps_ddot_error_ind>(state_cov, 0.5);
  numerics::set_diag_to_value<3, StateDelta::eps_ddot_error_ind + 3>(state_cov, 0.5);

  numerics::set_diag_to_value<3, StateDelta::R_world_from_body_error_log_ind>(state_cov,
                                                                              0.01);
  numerics::set_diag_to_value<3, StateDelta::x_world_error_ind>(state_cov, 0.01);

  return state_cov;
}

}  // namespace

FilterState<State> JetFilter::reasonable_initial_state(const jcc::TimePoint& t) {
  FilterState<State> xp0;
  xp0.time_of_validity = t;
  MatNd<State::DIM, State::DIM> state_cov;
  state_cov.setZero();
  numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                              StateDelta::accel_bias_error_ind>(state_cov, 0.0001);
  numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                              StateDelta::gyro_bias_error_ind>(state_cov, 0.0001);
  numerics::set_diag_to_value<StateDelta::eps_dot_error_dim,
                              StateDelta::eps_dot_error_ind>(state_cov, 0.0001);
  numerics::set_diag_to_value<StateDelta::eps_ddot_error_dim,
                              StateDelta::eps_ddot_error_ind>(state_cov, 0.001);

  numerics::set_diag_to_value<3, StateDelta::R_world_from_body_error_log_ind>(state_cov,
                                                                              0.001);
  numerics::set_diag_to_value<3, StateDelta::x_world_error_ind>(state_cov, 0.001);

  xp0.P = state_cov;
  return xp0;
}

Parameters JetFilter::reasonable_parameters() {
  Parameters p;
  // Quickly bleed the acceleration -- This is a strong prior
  p.acceleration_damping = -0.99;

  // TODO: Organize Parameters so they can only be constructed whole
  // These parameters were ballparked by jake
  const SO3 R_vehicle_from_camera = SO3::exp(jcc::Vec3::UnitY() * M_PI * 0.5) *
                                    SO3::exp(jcc::Vec3::UnitZ() * M_PI * 0.5);
  const SE3 vehicle_from_camera(R_vehicle_from_camera, jcc::Vec3(0.11, 0.0, 0.32));
  p.T_camera_from_vehicle = vehicle_from_camera.inverse();
  return p;
}

void JetFilter::setup_models() {
  ekf_ = JetEkf(make_dynamics(parameters_), make_cov());

  MatNd<AccelMeasurement::DIM, AccelMeasurement::DIM> accel_cov;
  accel_cov.setZero();
  {
    numerics::set_diag_to_value<AccelMeasurementDelta::observed_acceleration_error_dim,
                                AccelMeasurementDelta::observed_acceleration_error_ind>(
        accel_cov, 0.5);
  }

  const MatNd<GyroMeasurement::DIM, GyroMeasurement::DIM> gyro_cov = 0.1 * accel_cov;

  MatNd<FiducialMeasurement::DIM, FiducialMeasurement::DIM> fiducial_cov;
  fiducial_cov.setZero();
  {
    fiducial_cov.block<3, 3>(0, 0) = MatNd<3, 3>::Identity() * 0.25;
    fiducial_cov.block<3, 3>(3, 3) = MatNd<3, 3>::Identity() * 0.076;
  }

  imu_id_ = ekf_.add_model(
      bind_parameters<AccelMeasurement>(observe_accel_error_model, parameters_),
      accel_cov);
  gyro_id_ = ekf_.add_model(
      bind_parameters<GyroMeasurement>(observe_gyro_error_model, parameters_), gyro_cov);

  /*
  imu_2_id_ = ekf_.add_model(
      bind_parameters<AccelMeasurement>(observe_accel_2_error_model, parameters_),
      accel_cov);

  gyro_2_id_ = ekf_.add_model(
      bind_parameters<GyroMeasurement>(observe_gyro_2_error_model, parameters_),
      gyro_cov);
  */

  fiducial_id_ = ekf_.add_model(
      bind_parameters<FiducialMeasurement>(fiducial_error_model, parameters_),
      fiducial_cov);
}

JetFilter::JetFilter(const JetFilterState& xp, const Parameters& parameters)
    : xp_(xp), parameters_(parameters) {
  setup_models();
  initialized_ = true;
}

JetFilter::JetFilter(const Parameters& parameters) : parameters_(parameters) {
  setup_models();
  initialized_ = false;
}

void JetFilter::measure_imu(const AccelMeasurement& meas, const jcc::TimePoint& t, bool imu2) {
  JASSERT_FALSE(imu2, "imu2 temporarily unsupported");
  if (imu2) {
    ekf_.measure(meas, t, imu_2_id_);
  } else {
    ekf_.measure(meas, t, imu_id_);
  }
}

void JetFilter::measure_gyro(const GyroMeasurement& meas, const jcc::TimePoint& t, bool imu2) {
  JASSERT_FALSE(imu2, "imu2 temporarily unsupported");
  if (imu2) {
    ekf_.measure(meas, t, gyro_2_id_);
  } else {
    ekf_.measure(meas, t, gyro_id_);
  }
}

void JetFilter::measure_fiducial(const FiducialMeasurement& meas, const jcc::TimePoint& t) {
  ekf_.measure(meas, t, fiducial_id_);
}

State JetFilter::free_run() {
  JASSERT(initialized_, "Filter must be initialized");
  xp_ = ekf_.service_all_measurements(xp_);
  return xp_.x;
}

FilterState<State> JetFilter::run_until(const jcc::TimePoint& t) {
  JASSERT(initialized_, "Filter must be initialized");
  JASSERT(jcc::to_seconds(t - xp_.time_of_validity) < 10.0,
          "You're simulating for more than 10 seconds, something is probably wrong.");
  while (t > xp_.time_of_validity) {
    const auto result = ekf_.service_next_measurement(xp_);
    if (!result) {
      break;
    }
    xp_ = *result;
  }
  return xp_;
}

jcc::Optional<State> JetFilter::next_measurement() {
  JASSERT(initialized_, "Filter must be initialized");
  const auto result = ekf_.service_next_measurement(xp_);
  if (result) {
    xp_ = *result;
    return {result->x};
  } else {
    return {};
  }
}

FilterState<State> JetFilter::view(const jcc::TimePoint& t) const {
  JASSERT(initialized_, "Filter must be initialized");
  // TODO: Make this soft_service_until(xp_, t)
  const auto xp_t = ekf_.soft_service_all_measurements(xp_);
  return ekf_.dynamics_until(xp_t, t);
}

FilterState<State> JetFilter::predict(const jcc::TimePoint& t) const {
  JASSERT(initialized_, "Filter must be initialized");
  // TODO: Make this soft_service_until(xp_, t)
  return ekf_.dynamics_until(xp_, t);
}

}  // namespace jet_filter
}  // namespace estimation
