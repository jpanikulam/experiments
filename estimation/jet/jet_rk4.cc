#include "estimation/jet/jet_rk4.hh"

#include "eigen.hh"

namespace estimation {
namespace jet_filter {
VecNd<3> observe_accel(const State &state, const Parameters &parameters) {
  const SE3 sensor_from_vehicle = parameters.T_sensor_from_body;
  const MatNd<6, 6> adj = sensor_from_vehicle.Adj();
  const VecNd<6> eps_ddot = state.eps_ddot;
  const VecNd<3> a = (adj * eps_ddot).block<3, 1>(0, 0);
  const VecNd<6> eps_dot = state.eps_dot;
  const VecNd<3> w = (adj * eps_dot).block<3, 1>(3, 0);
  const VecNd<3> v = (adj * eps_dot).block<3, 1>(0, 0);
  const SE3 vehicle_from_world = state.T_body_from_world;
  const SO3 R_sensor_from_world =
      (sensor_from_vehicle * vehicle_from_world).so3();
  const VecNd<3> g_world = parameters.g_world;
  const VecNd<3> g_imu = R_sensor_from_world * g_world;
  const VecNd<3> accel_bias = state.accel_bias;
  const VecNd<3> clean = (w.cross(v)) - a;
  const VecNd<3> observed_acceleration = (clean + g_imu) + accel_bias;
  return observed_acceleration;
}
VecNd<24> to_vector(const StateDelta &in_grp) {
  const VecNd<24> out =
      (VecNd<24>() << ((in_grp.eps_dot_error)[0]), ((in_grp.eps_dot_error)[1]),
       ((in_grp.eps_dot_error)[2]), ((in_grp.eps_dot_error)[3]),
       ((in_grp.eps_dot_error)[4]), ((in_grp.eps_dot_error)[5]),
       ((in_grp.T_body_from_world_error_log)[0]),
       ((in_grp.T_body_from_world_error_log)[1]),
       ((in_grp.T_body_from_world_error_log)[2]),
       ((in_grp.T_body_from_world_error_log)[3]),
       ((in_grp.T_body_from_world_error_log)[4]),
       ((in_grp.T_body_from_world_error_log)[5]), ((in_grp.eps_ddot_error)[0]),
       ((in_grp.eps_ddot_error)[1]), ((in_grp.eps_ddot_error)[2]),
       ((in_grp.eps_ddot_error)[3]), ((in_grp.eps_ddot_error)[4]),
       ((in_grp.eps_ddot_error)[5]), ((in_grp.gyro_bias_error)[0]),
       ((in_grp.gyro_bias_error)[1]), ((in_grp.gyro_bias_error)[2]),
       ((in_grp.accel_bias_error)[0]), ((in_grp.accel_bias_error)[1]),
       ((in_grp.accel_bias_error)[2]))
          .finished();
  return out;
}
State operator-(const State &a, const State &b) {
  const State difference =
      State{((a.eps_dot) - (b.eps_dot)),
            ((a.T_body_from_world) * ((b.T_body_from_world).inverse())),
            ((a.eps_ddot) - (b.eps_ddot)), ((a.gyro_bias) - (b.gyro_bias)),
            ((a.accel_bias) - (b.accel_bias))};
  return difference;
}
VecNd<24> compute_delta(const State &a, const State &b) {
  const State difference = a - b;
  const VecNd<6> eps_dot_error = difference.eps_dot;
  const SE3 T_body_from_world_error = difference.T_body_from_world;
  const VecNd<6> T_body_from_world_error_log =
      SE3::log(T_body_from_world_error);
  const VecNd<6> eps_ddot_error = difference.eps_ddot;
  const VecNd<3> gyro_bias_error = difference.gyro_bias;
  const VecNd<3> accel_bias_error = difference.accel_bias;
  const StateDelta delta =
      StateDelta{eps_dot_error, T_body_from_world_error_log, eps_ddot_error,
                 gyro_bias_error, accel_bias_error};
  const VecNd<24> out_vec = to_vector(delta);
  return out_vec;
}
VecNd<27> to_vector(const ParametersDelta &in_grp) {
  const VecNd<27> out =
      (VecNd<27>() << ((in_grp.eps_dddot_error)[0]),
       ((in_grp.eps_dddot_error)[1]), ((in_grp.eps_dddot_error)[2]),
       ((in_grp.eps_dddot_error)[3]), ((in_grp.eps_dddot_error)[4]),
       ((in_grp.eps_dddot_error)[5]), ((in_grp.g_world_error)[0]),
       ((in_grp.g_world_error)[1]), ((in_grp.g_world_error)[2]),
       ((in_grp.daccel_bias_error)[0]), ((in_grp.daccel_bias_error)[1]),
       ((in_grp.daccel_bias_error)[2]),
       ((in_grp.T_sensor_from_body_error_log)[0]),
       ((in_grp.T_sensor_from_body_error_log)[1]),
       ((in_grp.T_sensor_from_body_error_log)[2]),
       ((in_grp.T_sensor_from_body_error_log)[3]),
       ((in_grp.T_sensor_from_body_error_log)[4]),
       ((in_grp.T_sensor_from_body_error_log)[5]),
       ((in_grp.dgyro_bias_error)[0]), ((in_grp.dgyro_bias_error)[1]),
       ((in_grp.dgyro_bias_error)[2]),
       ((in_grp.T_camera_from_body_error_log)[0]),
       ((in_grp.T_camera_from_body_error_log)[1]),
       ((in_grp.T_camera_from_body_error_log)[2]),
       ((in_grp.T_camera_from_body_error_log)[3]),
       ((in_grp.T_camera_from_body_error_log)[4]),
       ((in_grp.T_camera_from_body_error_log)[5]))
          .finished();
  return out;
}
Parameters operator-(const Parameters &a, const Parameters &b) {
  const Parameters difference =
      Parameters{((a.eps_dddot) - (b.eps_dddot)),
                 ((a.g_world) - (b.g_world)),
                 ((a.daccel_bias) - (b.daccel_bias)),
                 ((a.T_sensor_from_body) * ((b.T_sensor_from_body).inverse())),
                 ((a.dgyro_bias) - (b.dgyro_bias)),
                 ((a.T_camera_from_body) * ((b.T_camera_from_body).inverse()))};
  return difference;
}
VecNd<27> compute_delta(const Parameters &a, const Parameters &b) {
  const Parameters difference = a - b;
  const VecNd<3> g_world_error = difference.g_world;
  const VecNd<6> eps_dddot_error = difference.eps_dddot;
  const VecNd<3> daccel_bias_error = difference.daccel_bias;
  const SE3 T_sensor_from_body_error = difference.T_sensor_from_body;
  const VecNd<6> T_sensor_from_body_error_log =
      SE3::log(T_sensor_from_body_error);
  const VecNd<3> dgyro_bias_error = difference.dgyro_bias;
  const SE3 T_camera_from_body_error = difference.T_camera_from_body;
  const VecNd<6> T_camera_from_body_error_log =
      SE3::log(T_camera_from_body_error);
  const ParametersDelta delta =
      ParametersDelta{eps_dddot_error,   g_world_error,
                      daccel_bias_error, T_sensor_from_body_error_log,
                      dgyro_bias_error,  T_camera_from_body_error_log};
  const VecNd<27> out_vec = to_vector(delta);
  return out_vec;
}
VecNd<3> to_vector(const AccelMeasurementDelta &in_grp) {
  const VecNd<3> out = (VecNd<3>() << ((in_grp.observed_acceleration_error)[0]),
                        ((in_grp.observed_acceleration_error)[1]),
                        ((in_grp.observed_acceleration_error)[2]))
                           .finished();
  return out;
}
AccelMeasurement operator-(const AccelMeasurement &a,
                           const AccelMeasurement &b) {
  const AccelMeasurement difference =
      AccelMeasurement{((a.observed_acceleration) - (b.observed_acceleration))};
  return difference;
}
VecNd<3> compute_delta(const AccelMeasurement &a, const AccelMeasurement &b) {
  const AccelMeasurement difference = a - b;
  const VecNd<3> observed_acceleration_error = difference.observed_acceleration;
  const AccelMeasurementDelta delta =
      AccelMeasurementDelta{observed_acceleration_error};
  const VecNd<3> out_vec = to_vector(delta);
  return out_vec;
}
StateDot compute_qdot(const State &Q, const Parameters &Z) {
  const VecNd<6> eps_dddot = Z.eps_dddot;
  const VecNd<6> eps_dot = Q.eps_dot;
  const VecNd<6> eps_ddot = Q.eps_ddot;
  const VecNd<3> dgyro_bias = Z.dgyro_bias;
  const VecNd<3> daccel_bias = Z.daccel_bias;
  const StateDot Qdot =
      StateDot{eps_ddot, eps_dot, eps_dddot, dgyro_bias, daccel_bias};
  return Qdot;
}
StateDot operator*(const double h, const StateDot &K1) {
  const StateDot anon_271683 =
      StateDot{(h * (K1.eps_ddot)), (h * (K1.eps_dot)), (h * (K1.eps_dddot)),
               (h * (K1.dgyro_bias)), (h * (K1.daccel_bias))};
  return anon_271683;
}
State operator+(const State &Q, const StateDot &anon_6b57cd) {
  const State Q2 =
      State{((Q.eps_dot) + (anon_6b57cd.eps_ddot)),
            ((SE3::exp((anon_6b57cd.eps_dot))) * (Q.T_body_from_world)),
            ((Q.eps_ddot) + (anon_6b57cd.eps_dddot)),
            ((Q.gyro_bias) + (anon_6b57cd.dgyro_bias)),
            ((Q.accel_bias) + (anon_6b57cd.daccel_bias))};
  return Q2;
}
StateDot operator+(const StateDot &anon_271683, const StateDot &anon_35fffa) {
  const StateDot anon_73470e =
      StateDot{((anon_271683.eps_ddot) + (anon_35fffa.eps_ddot)),
               ((anon_271683.eps_dot) + (anon_35fffa.eps_dot)),
               ((anon_271683.eps_dddot) + (anon_35fffa.eps_dddot)),
               ((anon_271683.dgyro_bias) + (anon_35fffa.dgyro_bias)),
               ((anon_271683.daccel_bias) + (anon_35fffa.daccel_bias))};
  return anon_73470e;
}
State rk4_integrate(const State &Q, const Parameters &Z, const double h) {
  const double half = 0.5;
  const double half_h = h * half;
  const StateDot K1 = compute_qdot(Q, Z);
  const State Q2 = Q + (half_h * (h * K1));
  const StateDot K2 = compute_qdot(Q2, Z);
  const State Q3 = Q + (half_h * (h * K2));
  const StateDot K3 = compute_qdot(Q3, Z);
  const State Q4 = Q + (h * (h * K3));
  const StateDot K4 = compute_qdot(Q4, Z);
  const double two = 2.0;
  const double sixth = 0.166666666667;
  const State Qn =
      Q + (sixth * (((h * K1) + (h * K4)) + (two * ((h * K2) + (h * K3)))));
  return Qn;
}
State operator+(const State &a, const StateDelta &grp_b) {
  const State out = State{
      ((a.eps_dot) + (grp_b.eps_dot_error)),
      ((SE3::exp((grp_b.T_body_from_world_error_log))) * (a.T_body_from_world)),
      ((a.eps_ddot) + (grp_b.eps_ddot_error)),
      ((a.gyro_bias) + (grp_b.gyro_bias_error)),
      ((a.accel_bias) + (grp_b.accel_bias_error))};
  return out;
}
StateDelta from_vector(const VecNd<24> &in_vec) {
  const VecNd<3> anon_54bf63 =
      (VecNd<3>() << (in_vec[21]), (in_vec[22]), (in_vec[23])).finished();
  const VecNd<6> anon_20055f =
      (VecNd<6>() << (in_vec[12]), (in_vec[13]), (in_vec[14]), (in_vec[15]),
       (in_vec[16]), (in_vec[17]))
          .finished();
  const VecNd<6> anon_f26658 =
      (VecNd<6>() << (in_vec[0]), (in_vec[1]), (in_vec[2]), (in_vec[3]),
       (in_vec[4]), (in_vec[5]))
          .finished();
  const VecNd<6> anon_3c46ff =
      (VecNd<6>() << (in_vec[6]), (in_vec[7]), (in_vec[8]), (in_vec[9]),
       (in_vec[10]), (in_vec[11]))
          .finished();
  const VecNd<3> anon_5aca41 =
      (VecNd<3>() << (in_vec[18]), (in_vec[19]), (in_vec[20])).finished();
  const StateDelta out = StateDelta{anon_f26658, anon_3c46ff, anon_20055f,
                                    anon_5aca41, anon_54bf63};
  return out;
}
State apply_delta(const State &a, const VecNd<24> &delta) {
  const StateDelta grp_b = from_vector(delta);
  const State out = a + grp_b;
  return out;
}
Parameters operator+(const Parameters &a, const ParametersDelta &grp_b) {
  const Parameters out =
      Parameters{((a.eps_dddot) + (grp_b.eps_dddot_error)),
                 ((a.g_world) + (grp_b.g_world_error)),
                 ((a.daccel_bias) + (grp_b.daccel_bias_error)),
                 ((SE3::exp((grp_b.T_sensor_from_body_error_log))) *
                  (a.T_sensor_from_body)),
                 ((a.dgyro_bias) + (grp_b.dgyro_bias_error)),
                 ((SE3::exp((grp_b.T_camera_from_body_error_log))) *
                  (a.T_camera_from_body))};
  return out;
}
ParametersDelta from_vector(const VecNd<27> &in_vec) {
  const VecNd<6> anon_12da31 =
      (VecNd<6>() << (in_vec[0]), (in_vec[1]), (in_vec[2]), (in_vec[3]),
       (in_vec[4]), (in_vec[5]))
          .finished();
  const VecNd<3> anon_776225 =
      (VecNd<3>() << (in_vec[6]), (in_vec[7]), (in_vec[8])).finished();
  const VecNd<3> anon_2c8a5b =
      (VecNd<3>() << (in_vec[9]), (in_vec[10]), (in_vec[11])).finished();
  const VecNd<6> anon_275c10 =
      (VecNd<6>() << (in_vec[12]), (in_vec[13]), (in_vec[14]), (in_vec[15]),
       (in_vec[16]), (in_vec[17]))
          .finished();
  const VecNd<3> anon_c0a0c5 =
      (VecNd<3>() << (in_vec[18]), (in_vec[19]), (in_vec[20])).finished();
  const VecNd<6> anon_e9f9c9 =
      (VecNd<6>() << (in_vec[21]), (in_vec[22]), (in_vec[23]), (in_vec[24]),
       (in_vec[25]), (in_vec[26]))
          .finished();
  const ParametersDelta out =
      ParametersDelta{anon_12da31, anon_776225, anon_2c8a5b,
                      anon_275c10, anon_c0a0c5, anon_e9f9c9};
  return out;
}
Parameters apply_delta(const Parameters &a, const VecNd<27> &delta) {
  const ParametersDelta grp_b = from_vector(delta);
  const Parameters out = a + grp_b;
  return out;
}
AccelMeasurement operator+(const AccelMeasurement &a,
                           const AccelMeasurementDelta &grp_b) {
  const AccelMeasurement out = AccelMeasurement{
      ((a.observed_acceleration) + (grp_b.observed_acceleration_error))};
  return out;
}
AccelMeasurementDelta from_vector(const VecNd<3> &in_vec) {
  const VecNd<3> anon_9b1757 =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const AccelMeasurementDelta out = AccelMeasurementDelta{anon_9b1757};
  return out;
}
AccelMeasurement apply_delta(const AccelMeasurement &a, const VecNd<3> &delta) {
  const AccelMeasurementDelta grp_b = from_vector(delta);
  const AccelMeasurement out = a + grp_b;
  return out;
}
VecNd<3> observe_gyro(const VecNd<6> &eps_dot, const SE3 &T_world_from_sensor) {
  const SO3 R_world_from_sensor = T_world_from_sensor.so3();
  const VecNd<3> w = eps_dot.block<3, 1>(3, 0);
  return ((R_world_from_sensor.inverse()) * w);
}
} // namespace jet_filter
} // namespace estimation