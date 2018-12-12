#pragma once

#include "planning/jet/jet_dynamics.hh"
#include "planning/problem.hh"
#include "planning/xlqr_problem.hh"
#include "planning/differentiation.hh"

namespace planning {
namespace jet {
constexpr int X_DIM = 13;
constexpr int U_DIM = 4;

constexpr double DT = 0.1;
constexpr int HORIZON = 15;

using JetDim = Dimensions<X_DIM, U_DIM>;

using JetProblem = Problem<JetDim, State>;
using JetXlqr = XlqrProblem<JetProblem>;
}  // namespace jet
}  // namespace planning
