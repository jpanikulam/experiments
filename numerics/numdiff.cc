#include "numerics/numdiff.hh"

namespace numerics {

Eigen::VectorXd dynamic_numerical_gradient(
    const Eigen::VectorXd &x, const std::function<double(const Eigen::VectorXd &)> &fcn) {
  using OutVec = Eigen::VectorXd;
  constexpr double FEPS = 1e-6;
  constexpr double INV_2FEPS = 1.0 / (2.0 * FEPS);

  OutVec jac = OutVec::Zero(x.rows());
  OutVec twiddle = OutVec::Zero(x.rows());
  for (int k = 0; k < x.rows(); ++k) {
    twiddle(k) = FEPS;
    jac(k) = (fcn(x + twiddle) - fcn(x - twiddle)) * INV_2FEPS;
    twiddle(k) = 0.0;
  }
  return jac;
}

} // namespace numerics
