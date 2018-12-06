#pragma once

#include "eigen.hh"

#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {

struct StateControl {
  State state;
  Controls control;
};

std::vector<StateControl> plan(const State& x0,
                               const std::vector<Controls>& initialization = {});

}  // namespace jet
}  // namespace planning