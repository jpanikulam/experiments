#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {
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
  const StateDot anon_f05823 =
      StateDot{(half_h * (K1.w)), (half_h * (K1.throttle_dot)),
               (half_h * (K1.v)), (half_h * (K1.q)), (half_h * (K1.a))};
  return anon_f05823;
}
State operator+(const State &Q, const StateDot &anon_f05823) {
  const State Q2 = State{((SO3::exp((anon_f05823.w))) * (Q.R_world_from_body)),
                         ((Q.throttle_pct) + (anon_f05823.throttle_dot)),
                         ((Q.x) + (anon_f05823.v)), ((Q.w) + (anon_f05823.q)),
                         ((Q.v) + (anon_f05823.a))};
  return Q2;
}
StateDot operator+(const StateDot &K1, const StateDot &K4) {
  const StateDot anon_9ffb30 =
      StateDot{((K1.w) + (K4.w)), ((K1.throttle_dot) + (K4.throttle_dot)),
               ((K1.v) + (K4.v)), ((K1.q) + (K4.q)), ((K1.a) + (K4.a))};
  return anon_9ffb30;
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
  const double sixth = 0.166666666667;
  const double two = 2.0;
  const State Qn = Q + (sixth * ((K1 + K4) + (two * (K2 + K3))));
  return Qn;
}
} // namespace jet
} // namespace planning