#include "gradient_descent.hh"
#include "numdiff.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(GradientDescent, does_it_work) {
  const Eigen::Matrix3d Q_u = Eigen::Matrix3d::Random();
  const Eigen::Matrix3d Q = (Q_u * Q_u.transpose());

  const auto cost_func = [Q](const Eigen::Vector3d &x) -> double {  //
    return x.transpose() * Q * x;
  };

  const auto gd_cost_func = [cost_func](const Eigen::VectorXd &x, Eigen::VectorXd *grad) {  //
    const double cost = cost_func(x);
    if (grad) {
      *grad = numerical_gradient(Eigen::Vector3d(x), cost_func);
    }
    return cost;
  };

  minimize_gradient_descent(Eigen::Vector3d::Random(), gd_cost_func);
}
}
