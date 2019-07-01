#pragma once

#include "eigen.hh"

#include "numerics/optimization/error_function.hh"

#include <vector>

namespace numerics {

template <int dim_in, int dim_out>
struct MinimizationResult {
  VecNd<dim_in> solution = VecNd<dim_in>::Zero();
  VecNd<dim_in> terminal_jti = VecNd<dim_in>::Zero();
  VecNd<dim_out> terminal_error = VecNd<dim_out>::Zero();

  bool success = false;
};

struct OptimizationConfiguration {
  int max_iterations = 5;
  double levenberg_mu = 1e-3;
};

template <int dim_in, int dim_out>
MinimizationResult<dim_in, dim_out> gauss_newton_minimize(
    const std::vector<ErrorFunctionDifferentials<dim_in, dim_out>> &funcs,
    const VecNd<dim_in> &initialization,
    const OptimizationConfiguration &opt_cfg) {
  using OutVec = VecNd<dim_out>;
  using InVec = VecNd<dim_in>;
  using Information = MatNd<dim_in, dim_in>;
  using ErrorJacobian = MatNd<dim_out, dim_in>;

  MinimizationResult<dim_in, dim_out> result;
  InVec x = initialization;

  for (int k = 0; k < opt_cfg.max_iterations; ++k) {
    Information jtj = Information::Zero();
    InVec jti = InVec::Zero();

    result.terminal_error.setZero();
    ErrorJacobian jac;
    for (const auto &func : funcs) {
      jac.setZero();
      const OutVec innovation = func(x, &jac);
      jtj += jac.transpose() * jac;
      jti += jac.transpose() * innovation;
      result.terminal_error += innovation;
    }

    const Eigen::LLT<Information> llt_jtj(opt_cfg.levenberg_mu * Information::Identity() +
                                          jtj);
    if (llt_jtj.info() != Eigen::Success) {
      result.success = false;
      break;
    }

    const InVec update = llt_jtj.solve(jti);

    x -= update;

    result.solution = x;
    result.terminal_jti = jti;
    result.success = true;
  }

  return result;
}

template <int dim_in, int dim_out>
MinimizationResult<dim_in, dim_out> gauss_newton_minimize(
    const ErrorFunction<dim_in, dim_out> &func,
    const VecNd<dim_in> &initialization,
    const OptimizationConfiguration &opt_cfg = {}) {
  const std::vector<ErrorFunctionDifferentials<dim_in, dim_out>> fncs = {
      wrap_numerical_diff(func)};
  const auto result = gauss_newton_minimize(fncs, initialization, opt_cfg);
  return result;
}

}  // namespace numerics
