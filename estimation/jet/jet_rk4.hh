#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
struct StateDelta {
  VecNd<6> eps_dot_error = VecNd<6>::Zero();
  VecNd<6> T_body_from_world_error_log = VecNd<6>::Zero();
  VecNd<6> eps_ddot_error = VecNd<6>::Zero();
  VecNd<3> gyro_bias_error = VecNd<3>::Zero();
  VecNd<3> accel_bias_error = VecNd<3>::Zero();
  static constexpr int DIM = 24;
};
struct Parameters {
  VecNd<3> dgyro_bias = VecNd<3>::Zero();
  VecNd<3> daccel_bias = VecNd<3>::Zero();
  VecNd<6> eps_dddot = VecNd<6>::Zero();
  static constexpr int DIM = 12;
};
struct State {
  VecNd<6> eps_dot = VecNd<6>::Zero();
  SE3 T_body_from_world = SE3();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<3> gyro_bias = VecNd<3>::Zero();
  VecNd<3> accel_bias = VecNd<3>::Zero();
  static constexpr int DIM = 24;
};
struct AccelMeasurement {
  VecNd<3> observed_acceleration = VecNd<3>::Zero();
  static constexpr int DIM = 3;
};
struct StateDot {
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<6> eps_dot = VecNd<6>::Zero();
  VecNd<6> eps_dddot = VecNd<6>::Zero();
  VecNd<3> dgyro_bias = VecNd<3>::Zero();
  VecNd<3> daccel_bias = VecNd<3>::Zero();
  static constexpr int DIM = 24;
};
struct AccelMeasurementDelta {
  VecNd<3> observed_acceleration_error = VecNd<3>::Zero();
  static constexpr int DIM = 3;
};
VecNd<3> observe_accel(const SE3 &T_vehicle_from_world, const VecNd<6> &eps_dot,
                       const VecNd<6> &eps_ddot,
                       const SE3 &T_sensor_from_vehicle,
                       const VecNd<3> &gravity_mpss);
VecNd<24> compute_delta(const State &a, const State &b);
VecNd<3> compute_delta(const AccelMeasurement &a, const AccelMeasurement &b);
State rk4_integrate(const State &Q, const Parameters &Z, const double h);
State apply_delta(const State &a, const VecNd<24> &delta);
AccelMeasurement apply_delta(const AccelMeasurement &a, const VecNd<3> &delta);
VecNd<3> observe_gyro(const VecNd<6> &eps_dot, const SE3 &T_world_from_sensor);
} // namespace jet_filter
} // namespace estimation