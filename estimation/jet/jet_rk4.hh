/* Don't edit this; this code was generated by op_graph */

#pragma once
#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
struct StateDelta {
  VecNd<6> eps_ddot_error = VecNd<6>::Zero();
  VecNd<6> eps_dot_error = VecNd<6>::Zero();
  VecNd<3> gyro_bias_error = VecNd<3>::Zero();
  VecNd<3> accel_bias_error = VecNd<3>::Zero();
  VecNd<3> R_world_from_body_error_log = VecNd<3>::Zero();
  VecNd<3> x_world_error = VecNd<3>::Zero();
  static constexpr int eps_ddot_error_ind = 0;
  static constexpr int eps_ddot_error_dim = 6;
  static constexpr int eps_dot_error_ind = 6;
  static constexpr int eps_dot_error_dim = 6;
  static constexpr int gyro_bias_error_ind = 12;
  static constexpr int gyro_bias_error_dim = 3;
  static constexpr int accel_bias_error_ind = 15;
  static constexpr int accel_bias_error_dim = 3;
  static constexpr int R_world_from_body_error_log_ind = 18;
  static constexpr int R_world_from_body_error_log_dim = 3;
  static constexpr int x_world_error_ind = 21;
  static constexpr int x_world_error_dim = 3;
  static constexpr int DIM = 24;
  static VecNd<24> to_vector(const StateDelta &in_grp);
  static StateDelta from_vector(const VecNd<24> &in_vec);
};
struct Parameters {
  SE3 T_imu2_from_vehicle = SE3();
  double acceleration_damping = 0.0;
  SE3 T_imu1_from_vehicle = SE3();
  SE3 T_camera_from_vehicle = SE3();
  SE3 T_world_from_fiducial = SE3();
  static constexpr int DIM = 25;
  static VecNd<25> compute_delta(const Parameters &a, const Parameters &b);
  static Parameters apply_delta(const Parameters &a, const VecNd<25> &delta);
};
struct GyroMeasurementDelta {
  VecNd<3> observed_w_error = VecNd<3>::Zero();
  static constexpr int observed_w_error_ind = 0;
  static constexpr int observed_w_error_dim = 3;
  static constexpr int DIM = 3;
  static VecNd<3> to_vector(const GyroMeasurementDelta &in_grp);
  static GyroMeasurementDelta from_vector(const VecNd<3> &in_vec);
};
struct GyroMeasurement {
  VecNd<3> observed_w = VecNd<3>::Zero();
  static constexpr int observed_w_ind = 0;
  static constexpr int observed_w_dim = 3;
  static constexpr int DIM = 3;
  static VecNd<3> compute_delta(const GyroMeasurement &a,
                                const GyroMeasurement &b);
  static GyroMeasurement apply_delta(const GyroMeasurement &a,
                                     const VecNd<3> &delta);
};
struct State {
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<6> eps_dot = VecNd<6>::Zero();
  VecNd<3> gyro_bias = VecNd<3>::Zero();
  VecNd<3> accel_bias = VecNd<3>::Zero();
  SO3 R_world_from_body = SO3();
  VecNd<3> x_world = VecNd<3>::Zero();
  static constexpr int DIM = 24;
  static VecNd<24> compute_delta(const State &a, const State &b);
  static State apply_delta(const State &a, const VecNd<24> &delta);
};
struct AccelMeasurement {
  VecNd<3> observed_acceleration = VecNd<3>::Zero();
  static constexpr int observed_acceleration_ind = 0;
  static constexpr int observed_acceleration_dim = 3;
  static constexpr int DIM = 3;
  static VecNd<3> compute_delta(const AccelMeasurement &a,
                                const AccelMeasurement &b);
  static AccelMeasurement apply_delta(const AccelMeasurement &a,
                                      const VecNd<3> &delta);
};
struct StateDot {
  VecNd<6> damped_eps_ddot = VecNd<6>::Zero();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<3> gyro_bias_dot = VecNd<3>::Zero();
  VecNd<3> accel_bias_dot = VecNd<3>::Zero();
  VecNd<3> w = VecNd<3>::Zero();
  VecNd<3> v = VecNd<3>::Zero();
  static constexpr int damped_eps_ddot_ind = 0;
  static constexpr int damped_eps_ddot_dim = 6;
  static constexpr int eps_ddot_ind = 6;
  static constexpr int eps_ddot_dim = 6;
  static constexpr int gyro_bias_dot_ind = 12;
  static constexpr int gyro_bias_dot_dim = 3;
  static constexpr int accel_bias_dot_ind = 15;
  static constexpr int accel_bias_dot_dim = 3;
  static constexpr int w_ind = 18;
  static constexpr int w_dim = 3;
  static constexpr int v_ind = 21;
  static constexpr int v_dim = 3;
  static constexpr int DIM = 24;
};
struct ParametersDelta {
  VecNd<6> T_imu2_from_vehicle_error_log = VecNd<6>::Zero();
  double acceleration_damping_error = 0.0;
  VecNd<6> T_imu1_from_vehicle_error_log = VecNd<6>::Zero();
  VecNd<6> T_camera_from_vehicle_error_log = VecNd<6>::Zero();
  VecNd<6> T_world_from_fiducial_error_log = VecNd<6>::Zero();
  static constexpr int DIM = 25;
  static VecNd<25> to_vector(const ParametersDelta &in_grp);
  static ParametersDelta from_vector(const VecNd<25> &in_vec);
};
struct AccelMeasurementDelta {
  VecNd<3> observed_acceleration_error = VecNd<3>::Zero();
  static constexpr int observed_acceleration_error_ind = 0;
  static constexpr int observed_acceleration_error_dim = 3;
  static constexpr int DIM = 3;
  static VecNd<3> to_vector(const AccelMeasurementDelta &in_grp);
  static AccelMeasurementDelta from_vector(const VecNd<3> &in_vec);
};
State rk4_integrate(const State &Q, const Parameters &Z, const double h);
GyroMeasurement observe_gyro(const State &state, const Parameters &parameters);
AccelMeasurement observe_accel(const State &state,
                               const Parameters &parameters);
VecNd<3> observe_accel_error_model(const State &state,
                                   const AccelMeasurement &meas,
                                   const Parameters &parameters);
VecNd<3> observe_gyro_error_model(const State &state,
                                  const GyroMeasurement &meas,
                                  const Parameters &parameters);
} // namespace jet_filter
} // namespace estimation