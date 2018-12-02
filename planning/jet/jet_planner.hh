#pragma once

#include "eigen.hh"

#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {

std::vector<State> plan(const State& x0);

}  // namespace jet
}  // namespace planning