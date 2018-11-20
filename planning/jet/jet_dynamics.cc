#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {

namespace {
struct StateDot {
  VecNd<3> w;
  double throttle_dot;
  VecNd<3> v;
  VecNd<3> q;
  VecNd<3> a;
};

double force_from_throttle(const double throttle,
                           const double temperature,
                           const jcc::Vec3 &vel) {
  return throttle * 1.0;
}
}  // namespace

StateDot compute_qdot(const State &Q, const Controls &U, const Parameters &Z) {
  const double mass = Z.mass;
  const double inv_mass = (1.0 / mass);
  const SO3 R_world_from_body = Q.R_world_from_body;
  const double throttle_pct = Q.throttle_pct;
  const double temperature = Z.temperature;
  const VecNd<3> v = Q.v;
  const double thrust = force_from_throttle(throttle_pct, temperature, v);
  const VecNd<3> unit_z = Z.unit_z;
  const VecNd<3> body_force = thrust * unit_z;
  const VecNd<3> force_world = R_world_from_body * body_force;
  const VecNd<3> a = inv_mass * force_world;
  const VecNd<3> q = U.q;
  const double throttle_dot = U.throttle_dot;
  const VecNd<3> w = Q.w;
  const StateDot Qdot = StateDot{w, throttle_dot, v, q, a};
  return Qdot;
}
StateDot operator*(double half_h, const StateDot &K1) {
  const StateDot anon_7ebdc5 =
      StateDot{(half_h * (K1.w)), (half_h * (K1.throttle_dot)), (half_h * (K1.v)),
               (half_h * (K1.q)), (half_h * (K1.a))};
  return anon_7ebdc5;
}
State operator+(const State &Q, const StateDot &anon_7ebdc5) {
  const State Q2 =
      State{((SO3::exp((anon_7ebdc5.w))) * (Q.R_world_from_body)),
            ((Q.throttle_pct) + (anon_7ebdc5.throttle_dot)), ((Q.x) + (anon_7ebdc5.v)),
            ((Q.w) + (anon_7ebdc5.q)), ((Q.v) + (anon_7ebdc5.a))};
  return Q2;
}
StateDot operator+(const StateDot &K1, const StateDot &K4) {
  const StateDot anon_607ebe =
      StateDot{((K1.w) + (K4.w)), ((K1.throttle_dot) + (K4.throttle_dot)),
               ((K1.v) + (K4.v)), ((K1.q) + (K4.q)), ((K1.a) + (K4.a))};
  return anon_607ebe;
}
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z, double h) {
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

}  // namespace jet
}  // namespace planning
