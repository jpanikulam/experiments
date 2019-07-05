#pragma once
#include "numerics/numdiff.hh"

namespace numerics {

template <int DIM_IN, int DIM_OUT>
using ErrorFunctionDifferentials =
    std::function<VecNd<DIM_OUT>(const VecNd<DIM_IN> &, MatNd<DIM_OUT, DIM_IN> *)>;

template <int DIM_IN, int DIM_OUT>
using ErrorFunction = std::function<VecNd<DIM_OUT>(const VecNd<DIM_IN> &)>;

template <int DIM_IN, int DIM_OUT>
ErrorFunctionDifferentials<DIM_IN, DIM_OUT> wrap_numerical_diff(
    const ErrorFunction<DIM_IN, DIM_OUT> &fnc) {
  return [fnc](const VecNd<DIM_IN> &x, MatNd<DIM_OUT, DIM_IN> *const jac) {
    if (jac) {
      *jac += numerical_jacobian<DIM_OUT>(x, fnc);
    }
    return fnc(x);
  };
}

}  // namespace numerics