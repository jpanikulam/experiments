#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet {
struct StateDelta {
  VecNd<6> eps_dot_error = VecNd<6>::Zero();
  VecNd<6> eps_ddot_error = VecNd<6>::Zero();
  VecNd<6> T_world_from_body_error_log = VecNd<6>::Zero();
  static constexpr int DIM = 18;
};
struct State {
  VecNd<6> eps_dot = VecNd<6>::Zero();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  SE3 T_world_from_body = SE3();
  static constexpr int DIM = 15;
};
struct Parameters {
  VecNd<6> eps_dddot = VecNd<6>::Zero();
  static constexpr int DIM = 6;
};
struct StateDot {
  VecNd<6> eps_dot = VecNd<6>::Zero();
  VecNd<6> eps_ddot = VecNd<6>::Zero();
  VecNd<6> eps_dddot = VecNd<6>::Zero();
  static constexpr int DIM = 18;
};
VecNd<18> compute_delta(const State &a, const State &b);
State rk4_integrate(const State &Q, const Parameters &Z, const double h);
State apply_delta(const State &a, const VecNd<18> &delta);
} // namespace jet
} // namespace estimation