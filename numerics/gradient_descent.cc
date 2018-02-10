#include "gradient_descent.hh"

// TODO
#include <iostream>

namespace numerics {

void minimize_gradient_descent(const Eigen::VectorXd &initialization, const SimpleCostFunction &cost_func) {
  // Later, write bfgs
  double best_cost = cost_func(initialization, nullptr);

  Eigen::VectorXd x = initialization;

  constexpr int MAX_ITERS = 10;
  for (int k = 0; k < MAX_ITERS; ++k) {
    Eigen::VectorXd gradient;
    const double cur_cost = cost_func(x, &gradient);
    std::cout << "x: " << x.transpose() << " : " << cur_cost << std::endl;

    // Do a linesearch
    Eigen::VectorXd best_x = x;
    for (int j = -1; j < 10; ++j) {
      const double alpha = 1.3 * std::pow(0.5, j);
      const Eigen::VectorXd test_x = x - (alpha * gradient);
      const double cost = cost_func(test_x, nullptr);
      if (cost < best_cost) {
        best_cost = cost;
        best_x = test_x;
      }
    }
    x = best_x;
  }
}
}
