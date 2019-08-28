/* Don't edit this; this code was generated by op_graph */

#pragma once
#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace drifter {
struct StateDelta {
  double x_vel_error = 0.0;
  double phi_error = 0.0;
  VecNd<3> R_world_from_body_error_log = VecNd<3>::Zero();
  VecNd<3> x_world_error = VecNd<3>::Zero();
  static constexpr int DIM = 8;
  static VecNd<8> to_vector(const StateDelta &in_grp);
  static StateDelta from_vector(const VecNd<8> &in_vec);
};
struct Parameters {
  double mass = 0.0;
  VecNd<3> external_force = VecNd<3>::Zero();
  static constexpr int DIM = 4;
};
struct Controls {
  double a = 0.0;
  double phidot = 0.0;
  static constexpr int DIM = 2;
  static Controls from_vector(const VecNd<2> &in_vec);
  static VecNd<2> to_vector(const Controls &in_grp);
};
struct State {
  double x_vel = 0.0;
  double phi = 0.0;
  SO3 R_world_from_body = SO3();
  VecNd<3> x_world = VecNd<3>::Zero();
  static constexpr int DIM = 8;
  static VecNd<8> compute_delta(const State &a, const State &b);
  static State apply_delta(const State &a, const VecNd<8> &delta);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
struct StateDot {
  double a = 0.0;
  double phidot = 0.0;
  VecNd<3> w = VecNd<3>::Zero();
  VecNd<3> vel_world = VecNd<3>::Zero();
  static constexpr int DIM = 8;
};
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z,
                    const double h);
} // namespace drifter
} // namespace planning