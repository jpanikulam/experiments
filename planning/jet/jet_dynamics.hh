#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {

struct Parameters {
  double mass = 100.0;
  double temperature = 1.0;
  VecNd<3> unit_z = VecNd<3>::UnitZ();
};
struct Controls {
  VecNd<3> q = VecNd<3>::Zero();
  double throttle_dot = 0.0;
};
struct State {
  SO3 R_world_from_body;
  double throttle_pct = 1.0;
  VecNd<3> x = VecNd<3>::Zero();
  VecNd<3> w = VecNd<3>::Zero();
  VecNd<3> v = VecNd<3>::Zero();
};

State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z, double h);

}  // namespace jet
}  // namespace planning
