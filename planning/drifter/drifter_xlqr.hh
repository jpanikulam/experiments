#pragma once

#include "planning/drifter/drifter_dynamics.hh"
#include "planning/problem.hh"
#include "planning/xlqr_problem.hh"
#include "planning/differentiation.hh"

namespace planning {
namespace drifter {
constexpr int X_DIM = State::DIM;
constexpr int U_DIM = Controls::DIM;

constexpr double DT = 0.15;
constexpr int HORIZON = 70;

using DrifterDim = Dimensions<X_DIM, U_DIM>;
using DrifterProblem = Problem<DrifterDim, State>;
using DrifterXlqr = XlqrProblem<DrifterProblem>;
}  // namespace drifter
}  // namespace planning
