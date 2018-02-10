#pragma once

#include "eigen.hh"

namespace numerics {

using SimpleCostFunction = std::function<double(const Eigen::VectorXd &, Eigen::VectorXd *)>;

void minimize_gradient_descent(const Eigen::VectorXd &initialization, const SimpleCostFunction &cost_func);
}
