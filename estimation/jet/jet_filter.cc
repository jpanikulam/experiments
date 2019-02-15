#include "estimation/jet/jet_filter.hh"

#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"
#include "numerics/set_diag_to_value.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

Parameters get_parameters() {
  const jcc::Vec3 g(0.0, 0.0, -9.81);

  // const jcc::Vec3 trans_imu_from_vehicle = jcc::Vec3(0.0, 0.0, 0.0);
  // const SO3 R_imu_from_vehicle = SO3::exp(jcc::Vec3(0.0, 0.0, 0.0));

  // const jcc::Vec3 trans_imu_from_vehicle = jcc::Vec3(0.22094, -0.181402, -0.685096);
  const jcc::Vec3 trans_imu_from_vehicle = jcc::Vec3(0.0, 0.0, 0.0);
  // const SO3 R_imu_from_vehicle = SO3::exp(-M_PI * jcc::Vec3::UnitX());
  const SO3 R_imu_from_vehicle = SO3::exp(jcc::Vec3(-3.10341, -0.0387912, -0.00578994));

  const SE3 imu_from_vehicle(R_imu_from_vehicle, trans_imu_from_vehicle);

  Parameters p;
  p.T_imu_from_vehicle = imu_from_vehicle;
  // p.g_world = g;
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

MatNd<State::DIM, State::DIM> make_cov() {
  MatNd<State::DIM, State::DIM> state_cov;
  state_cov.setZero();
  numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                              StateDelta::accel_bias_error_ind>(state_cov, 0.0001);
  numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                              StateDelta::gyro_bias_error_ind>(state_cov, 0.0001);

  numerics::set_diag_to_value<3, StateDelta::eps_dot_error_ind>(state_cov, 0.001);

  numerics::set_diag_to_value<3, StateDelta::eps_dot_error_ind + 3>(state_cov, 0.001);

  numerics::set_diag_to_value<3, StateDelta::eps_ddot_error_ind>(state_cov, 0.1);
  numerics::set_diag_to_value<3, StateDelta::eps_ddot_error_ind + 3>(state_cov, 0.1);

  numerics::set_diag_to_value<3, StateDelta::T_body_from_world_error_log_ind>(state_cov,
                                                                              0.001);

  numerics::set_diag_to_value<3, StateDelta::T_body_from_world_error_log_ind + 3>(
      state_cov, 0.001);

  return state_cov;
}

}  // namespace

FilterState<State> JetFilter::reasonable_initial_state() {
  FilterState<State> xp0;
  MatNd<State::DIM, State::DIM> state_cov;
  state_cov.setZero();
  numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                              StateDelta::accel_bias_error_ind>(state_cov, 0.001);
  numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                              StateDelta::gyro_bias_error_ind>(state_cov, 0.01);
  numerics::set_diag_to_value<StateDelta::eps_dot_error_dim,
                              StateDelta::eps_dot_error_ind>(state_cov, 0.01);
  numerics::set_diag_to_value<StateDelta::eps_ddot_error_dim,
                              StateDelta::eps_ddot_error_ind>(state_cov, 0.1);

  numerics::set_diag_to_value<StateDelta::T_body_from_world_error_log_dim,
                              StateDelta::T_body_from_world_error_log_ind>(state_cov,
                                                                           1.0);

  xp0.P = state_cov;
  return xp0;
}

void JetFilter::setup_models() {
  MatNd<AccelMeasurement::DIM, AccelMeasurement::DIM> accel_cov;
  accel_cov.setZero();
  {
    numerics::set_diag_to_value<AccelMeasurementDelta::observed_acceleration_error_dim,
                                AccelMeasurementDelta::observed_acceleration_error_ind>(
        accel_cov, 0.01);
  }

  MatNd<FiducialMeasurement::DIM, FiducialMeasurement::DIM> fiducial_cov;
  fiducial_cov.setZero();
  {
    fiducial_cov.block<3, 3>(0, 0) = MatNd<3, 3>::Identity() * 0.00025;
    fiducial_cov.block<3, 3>(3, 3) = MatNd<3, 3>::Identity() * 0.00076;
  }

  parameters_ = get_parameters();
  imu_id_ = ekf_.add_model(
      bind_parameters<AccelMeasurement>(observe_accel_error_model, parameters_),
      accel_cov);
  gyro_id_ = ekf_.add_model(
      bind_parameters<GyroMeasurement>(observe_gyro_error_model, parameters_), accel_cov);

  fiducial_id_ = ekf_.add_model(
      bind_parameters<FiducialMeasurement>(fiducial_error_model, parameters_),
      fiducial_cov);
}

JetFilter::JetFilter(const JetFilterState& xp0) : xp_(xp0), ekf_(dynamics, make_cov()) {
  setup_models();
  initialized_ = true;
}

JetFilter::JetFilter() : ekf_(dynamics, make_cov()) {
  setup_models();
  initialized_ = false;
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

State JetFilter::free_run() {
  assert(initialized_);
  xp_ = ekf_.service_all_measurements(xp_);
  return xp_.x;
}

jcc::Optional<State> JetFilter::next_measurement() {
  assert(initialized_);
  const auto result = ekf_.service_next_measurement(xp_);
  if (result) {
    xp_ = *result;
    return {result->x};
  } else {
    return {};
  }
}

State JetFilter::view(const TimePoint& t) const {
  assert(initialized_);
  // TODO: Make this soft_service_until(xp_, t)
  const auto xp_t = ekf_.soft_service_all_measurements(xp_);
  return ekf_.dynamics_until(xp_t, t).x;
}

State JetFilter::predict(const TimePoint& t) const {
  assert(initialized_);
  // TODO: Make this soft_service_until(xp_, t)
  return ekf_.dynamics_until(xp_, t).x;
}

}  // namespace jet_filter
}  // namespace estimation
