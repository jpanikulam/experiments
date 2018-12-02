#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {
struct Parameters {
  VecNd<3> unit_z = VecNd<3>::Zero();
  double mass = 0.0;
  VecNd<3> external_force = VecNd<3>::Zero();
};
struct Controls {
  VecNd<3> q = VecNd<3>::Zero();
  double throttle_dot = 0.0;
};
struct State {
  SO3 R_world_from_body = SO3();
  double throttle_pct = 0.0;
  VecNd<3> x = VecNd<3>::Zero();
  VecNd<3> w = VecNd<3>::Zero();
  VecNd<3> v = VecNd<3>::Zero();
};
struct StateDot {
  VecNd<3> w = VecNd<3>::Zero();
  double throttle_dot = 0.0;
  VecNd<3> v = VecNd<3>::Zero();
  VecNd<3> q = VecNd<3>::Zero();
  VecNd<3> a = VecNd<3>::Zero();
};
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z,
                    const double h);
Controls from_vector(const VecNd<4> &in_vec);
} // namespace jet
} // namespace planning