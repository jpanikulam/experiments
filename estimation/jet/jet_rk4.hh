#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
struct StateDelta {
  VecNd<3> accel_bias_error = VecNd<3>::Zero();
  VecNd<6> eps_ddot_error = VecNd<6>::Zero();
  VecNd<6> eps_dot_error = VecNd<6>::Zero();
  VecNd<3> gyro_bias_error = VecNd<3>::Zero();
  VecNd<6> T_body_from_world_error_log = VecNd<6>::Zero();
  static constexpr int accel_bias_error_ind = 0;
  static constexpr int accel_bias_error_dim = 3;
  static constexpr int eps_ddot_error_ind = 3;
  static constexpr int eps_ddot_error_dim = 6;
  static constexpr int eps_dot_error_ind = 9;
  static constexpr int eps_dot_error_dim = 6;
  static constexpr int gyro_bias_error_ind = 15;
  static constexpr int gyro_bias_error_dim = 3;
  static constexpr int T_body_from_world_error_log_ind = 18;
  static constexpr int T_body_from_world_error_log_dim = 6;
  static constexpr int DIM = 24;
};
struct Parameters {
  VecNd<3> g_world = VecNd<3>::Zero();
  SE3 T_sensor_from_body = SE3();
  static constexpr int DIM = 9;
};
struct State {
  VecNd<3> accel_bias = VecNd<3>::Zero();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<6> eps_dot = VecNd<6>::Zero();
  VecNd<3> gyro_bias = VecNd<3>::Zero();
  SE3 T_body_from_world = SE3();
  static constexpr int DIM = 24;
};
struct AccelMeasurement {
  VecNd<3> observed_acceleration = VecNd<3>::Zero();
  static constexpr int observed_acceleration_ind = 0;
  static constexpr int observed_acceleration_dim = 3;
  static constexpr int DIM = 3;
};
struct StateDot {
  VecNd<3> accel_bias_dot = VecNd<3>::Zero();
  VecNd<6> eps_ddot_dot = VecNd<6>::Zero();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<3> gyro_bias_dot = VecNd<3>::Zero();
  VecNd<6> eps_dot = VecNd<6>::Zero();
  static constexpr int accel_bias_dot_ind = 0;
  static constexpr int accel_bias_dot_dim = 3;
  static constexpr int eps_ddot_dot_ind = 3;
  static constexpr int eps_ddot_dot_dim = 6;
  static constexpr int eps_ddot_ind = 9;
  static constexpr int eps_ddot_dim = 6;
  static constexpr int gyro_bias_dot_ind = 15;
  static constexpr int gyro_bias_dot_dim = 3;
  static constexpr int eps_dot_ind = 18;
  static constexpr int eps_dot_dim = 6;
  static constexpr int DIM = 24;
};
struct ParametersDelta {
  VecNd<3> g_world_error = VecNd<3>::Zero();
  VecNd<6> T_sensor_from_body_error_log = VecNd<6>::Zero();
  static constexpr int g_world_error_ind = 0;
  static constexpr int g_world_error_dim = 3;
  static constexpr int T_sensor_from_body_error_log_ind = 3;
  static constexpr int T_sensor_from_body_error_log_dim = 6;
  static constexpr int DIM = 9;
};
struct AccelMeasurementDelta {
  VecNd<3> observed_acceleration_error = VecNd<3>::Zero();
  static constexpr int observed_acceleration_error_ind = 0;
  static constexpr int observed_acceleration_error_dim = 3;
  static constexpr int DIM = 3;
};
VecNd<3> observe_accel(const State &state, const Parameters &parameters);
VecNd<24> compute_delta(const State &a, const State &b);
VecNd<9> compute_delta(const Parameters &a, const Parameters &b);
VecNd<3> compute_delta(const AccelMeasurement &a, const AccelMeasurement &b);
State rk4_integrate(const State &Q, const Parameters &Z, const double h);
State apply_delta(const State &a, const VecNd<24> &delta);
Parameters apply_delta(const Parameters &a, const VecNd<9> &delta);
AccelMeasurement apply_delta(const AccelMeasurement &a, const VecNd<3> &delta);
VecNd<3> observe_gyro(const VecNd<6> &eps_dot, const SE3 &T_world_from_sensor);
} // namespace jet_filter
} // namespace estimation