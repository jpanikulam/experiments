#pragma once

#include "eigen.hh"

#include "planning/drifter/drifter_dynamics.hh"
#include "planning/drifter/drifter_xlqr.hh"

namespace planning {
namespace drifter {

struct StateControl {
  State state;
  Controls control;
};

struct Avoid {
  jcc::Vec3 pos_world;
  double radius = 1.0;
};

struct BehaviorPrimitives {
  // jcc::Vec3 target = jcc::Vec3::Zero();
  jcc::Vec3 target = jcc::Vec3(1.0, 1.0, 0.0);
  double vmax = 1.0;

  std::vector<Avoid> avoids;
};

std::vector<StateControl> plan(const State& x0,
                               const BehaviorPrimitives& desires,
                               const std::vector<Controls>& initialization = {});

DrifterProblem::Cost make_drifter_cost(const BehaviorPrimitives& desires);

Parameters get_parameters();

}  // namespace drifter
}  // namespace planning