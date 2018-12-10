#pragma once

#include "Eigen/Dense"

namespace numerics {

template <int rows, typename Callable>
Eigen::Matrix<double, rows, 1> numerical_gradient(
    const Eigen::Matrix<double, rows, 1> &x,
    const Callable &fcn,
    const double feps = 1e-6) {
  static_assert(rows != Eigen::Dynamic, "No dynamic");
  using OutVec = Eigen::Matrix<double, rows, 1>;

  OutVec jac = OutVec::Zero();

  for (int k = 0; k < rows; ++k) {
    OutVec zero = OutVec::Zero();
    zero(k) = feps;

    jac(k) = (fcn(x + zero) - fcn(x - zero)) / (2.0 * feps);
  }
  return jac;
}

Eigen::VectorXd dynamic_numerical_gradient(
    const Eigen::VectorXd &x,
    const std::function<double(const Eigen::VectorXd &)> &fcn) {
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

template <int output_rows, int input_rows, typename Callable>
Eigen::Matrix<double, output_rows, input_rows> numerical_jacobian(
    const Eigen::Matrix<double, input_rows, 1> &x,
    const Callable &fcn,
    const double feps = 1e-6) {
  static_assert(output_rows != Eigen::Dynamic, "No dynamic");
  static_assert(input_rows != Eigen::Dynamic, "No dynamic");

  // df1/dx1    df1/dx2 .. ..    df1/dxm
  // df2/dx1    df2/dx2 .. ..    df2/dxm
  //    :          :     ..
  //    :          :           ..
  // dfn/dx1    dfn/dx2              ..

  using OutVec = Eigen::Matrix<double, input_rows, 1>;
  using Jacobian = Eigen::Matrix<double, output_rows, input_rows>;
  Jacobian jac = Jacobian::Zero();

  for (int k = 0; k < input_rows; ++k) {
    OutVec zero = OutVec::Zero();
    zero(k) = feps;
    jac.col(k) = (fcn(x + zero) - fcn(x - zero)) / (2 * feps);
  }
  return jac;
}
}  // namespace numerics
