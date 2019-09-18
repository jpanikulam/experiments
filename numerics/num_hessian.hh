#pragma once

#include "numerics/numdiff.hh"

namespace numerics {

// Compute jacobian of gradient for some stupid function
//
// Don't do any clever central difference approximation or anything, we're just going for
// broke here
template <int rows, typename Callable>
Eigen::Matrix<double, rows, rows> numerical_hessian(
    const Eigen::Matrix<double, rows, 1> &x,
    const Callable &fcn,
    const double feps = 1e-6) {
  MatNd<rows, rows> hessian;
  hessian.setZero();

  const double ifeps = 0.25 / (feps * feps);
  using Vec = VecNd<rows>;
  for (int i = 0; i < rows; ++i) {
    for (int j = i; j < rows; ++j) {
      const Vec u_i = Vec::Unit(i) * feps;
      const Vec u_j = Vec::Unit(j) * feps;

      const double fpp = fcn((x + u_i) + u_j);
      const double fnp = fcn((x - u_i) + u_j);
      const double fpn = fcn((x + u_i) - u_j);
      const double fnn = fcn((x - u_i) - u_j);

      const double diff = ifeps * ((fpp - fnp) - (fpn - fnn));
      hessian(i, j) = diff;
      hessian(j, i) = diff;
    }
  }

  return hessian;
}
}  // namespace numerics
