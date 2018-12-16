#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {
State operator+(const State &a, const StateDelta &grp_b) {
  const State out = State{
      ((SO3::exp((grp_b.R_world_from_body_error_log))) * (a.R_world_from_body)),
      ((a.throttle_pct) + (grp_b.throttle_pct_error)),
      ((a.x) + (grp_b.x_error)), ((a.w) + (grp_b.w_error)),
      ((a.v) + (grp_b.v_error))};
  return out;
}
StateDelta from_vector(const VecNd<13> &in_vec) {
  const VecNd<3> anon_2e3a7d =
      (VecNd<3>() << (in_vec[10]), (in_vec[11]), (in_vec[12])).finished();
  const VecNd<3> anon_6fbdea =
      (VecNd<3>() << (in_vec[7]), (in_vec[8]), (in_vec[9])).finished();
  const VecNd<3> anon_4eaca6 =
      (VecNd<3>() << (in_vec[4]), (in_vec[5]), (in_vec[6])).finished();
  const VecNd<3> anon_f7d0ed =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const StateDelta out = StateDelta{anon_f7d0ed, (in_vec[3]), anon_4eaca6,
                                    anon_6fbdea, anon_2e3a7d};
  return out;
}
State apply_delta(const State &a, const VecNd<13> &delta) {
  const StateDelta grp_b = from_vector(delta);
  const State out = a + grp_b;
  return out;
}
double force_from_throttle(const double throttle) {
  const double out = throttle;
  return out;
}
StateDot compute_qdot(const State &Q, const Controls &U, const Parameters &Z) {
  const double mass = Z.mass;
  const double inv_mass = (1.0 / mass);
  const SO3 R_world_from_body = Q.R_world_from_body;
  const double throttle_pct = Q.throttle_pct;
  const double thrust = force_from_throttle(throttle_pct);
  const VecNd<3> unit_z = Z.unit_z;
  const VecNd<3> body_force = thrust * unit_z;
  const VecNd<3> force_world = R_world_from_body * body_force;
  const VecNd<3> external_force = Z.external_force;
  const VecNd<3> net_force_world = force_world + external_force;
  const VecNd<3> a = inv_mass * net_force_world;
  const VecNd<3> q = U.q;
  const double throttle_dot = U.throttle_dot;
  const VecNd<3> w = Q.w;
  const VecNd<3> v = Q.v;
  const StateDot Qdot = StateDot{w, throttle_dot, v, q, a};
  return Qdot;
}
StateDot operator*(const double half_h, const StateDot &K1) {
  const StateDot anon_7b85dc =
      StateDot{(half_h * (K1.w)), (half_h * (K1.throttle_dot)),
               (half_h * (K1.v)), (half_h * (K1.q)), (half_h * (K1.a))};
  return anon_7b85dc;
}
State operator+(const State &Q, const StateDot &anon_7b85dc) {
  const State Q2 = State{((SO3::exp((anon_7b85dc.w))) * (Q.R_world_from_body)),
                         ((Q.throttle_pct) + (anon_7b85dc.throttle_dot)),
                         ((Q.x) + (anon_7b85dc.v)), ((Q.w) + (anon_7b85dc.q)),
                         ((Q.v) + (anon_7b85dc.a))};
  return Q2;
}
StateDot operator+(const StateDot &K1, const StateDot &K4) {
  const StateDot anon_8683b4 =
      StateDot{((K1.w) + (K4.w)), ((K1.throttle_dot) + (K4.throttle_dot)),
               ((K1.v) + (K4.v)), ((K1.q) + (K4.q)), ((K1.a) + (K4.a))};
  return anon_8683b4;
}
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z,
                    const double h) {
  const double half = 0.5;
  const double half_h = h * half;
  const StateDot K1 = compute_qdot(Q, U, Z);
  const State Q2 = Q + (half_h * K1);
  const StateDot K2 = compute_qdot(Q2, U, Z);
  const State Q3 = Q + (half_h * K2);
  const StateDot K3 = compute_qdot(Q3, U, Z);
  const State Q4 = Q + (h * K3);
  const StateDot K4 = compute_qdot(Q4, U, Z);
  const double two = 2.0;
  const double sixth = 0.166666666667;
  const State Qn = Q + (sixth * ((K1 + K4) + (two * (K2 + K3))));
  return Qn;
}
VecNd<13> to_vector(const StateDelta &in_grp) {
  const VecNd<13> out =
      (VecNd<13>() << ((in_grp.R_world_from_body_error_log)[0]),
       ((in_grp.R_world_from_body_error_log)[1]),
       ((in_grp.R_world_from_body_error_log)[2]), (in_grp.throttle_pct_error),
       ((in_grp.x_error)[0]), ((in_grp.x_error)[1]), ((in_grp.x_error)[2]),
       ((in_grp.w_error)[0]), ((in_grp.w_error)[1]), ((in_grp.w_error)[2]),
       ((in_grp.v_error)[0]), ((in_grp.v_error)[1]), ((in_grp.v_error)[2]))
          .finished();
  return out;
}
State operator-(const State &a, const State &b) {
  const State difference =
      State{((a.R_world_from_body) * ((b.R_world_from_body).inverse())),
            ((a.throttle_pct) - (b.throttle_pct)), ((a.x) - (b.x)),
            ((a.w) - (b.w)), ((a.v) - (b.v))};
  return difference;
}
VecNd<13> delta_vec(const State &a, const State &b) {
  const State difference = a - b;
  const SO3 R_world_from_body_error = difference.R_world_from_body;
  const VecNd<3> R_world_from_body_error_log =
      SO3::log(R_world_from_body_error);
  const double throttle_pct_error = difference.throttle_pct;
  const VecNd<3> x_error = difference.x;
  const VecNd<3> w_error = difference.w;
  const VecNd<3> v_error = difference.v;
  const StateDelta delta =
      StateDelta{R_world_from_body_error_log, throttle_pct_error, x_error,
                 w_error, v_error};
  const VecNd<13> out_vec = to_vector(delta);
  return out_vec;
}
Controls from_vector(const VecNd<4> &in_vec) {
  const VecNd<3> anon_d4f100 =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const Controls out = Controls{anon_d4f100, (in_vec[3])};
  return out;
}
VecNd<4> to_vector(const Controls &in_grp) {
  const VecNd<4> out = (VecNd<4>() << ((in_grp.q)[0]), ((in_grp.q)[1]),
                        ((in_grp.q)[2]), (in_grp.throttle_dot))
                           .finished();
  return out;
}
} // namespace jet
} // namespace planning