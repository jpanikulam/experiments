#include "Eigen/Dense"

namespace raytrace {

// todo: move to numerics
template <int cols, typename Callable>
Eigen::Matrix<double, cols, 1> numerical_gradient(const Eigen::Matrix<double, cols, 1> &x,
                                                  const Callable &fcn,
                                                  const double    feps = 1e-6) {
  using OutVec = Eigen::Matrix<double, cols, 1>;

  OutVec jac = OutVec::Zero();

  for (int k = 0; k < cols; ++k) {
    OutVec zero = OutVec::Zero();
    zero(k)     = feps;

    jac(k) = (fcn(x + zero) - fcn(x - zero)) / (2 * feps);
  }
  return jac;
}
}