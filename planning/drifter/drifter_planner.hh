#pragma once

#include "eigen.hh"

#include "planning/drifter/drifter_dynamics.hh"
#include "planning/drifter/drifter_xlqr.hh"

#include "planning/drifter/drifter_configuration.hh"

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
  jcc::Vec3 target = jcc::Vec3(1.0, 1.0, 0.0);
  jcc::Vec3 look_target = jcc::Vec3(2.0, -2.0, 0.0);
  bool have_look_target = false;

  double max_vel = 1.0;
  double max_accel = 3.0;

  std::vector<Avoid> avoids;
};

std::vector<StateControl> plan(const State& x0,
                               const BehaviorPrimitives& desires,
                               const PlannerConfiguration& cfg,
                               const std::vector<Controls>& initialization = {});

DrifterProblem::Cost make_drifter_cost(const PlannerConfiguration& cfg,
                                       const BehaviorPrimitives& desires);

Parameters get_parameters();

}  // namespace drifter
}  // namespace planning