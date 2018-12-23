#pragma once

#include "numerics/numdiff.hh"

#include <functional>

namespace numerics {

template <typename X>
using ApplyDelta = std::function<X(const X&, const VecNd<X::DIM>&)>;

template <typename X>
using ComputeDelta = std::function<VecNd<X::DIM>(const X&, const X&)>;

template <typename X, typename Y>
using GroupFnc = std::function<Y(const X&)>;

template <typename X, typename Y>
MatNd<Y::DIM, X::DIM> group_jacobian(const VecNd<X::DIM>& x,
                                     const GroupFnc<X, Y>& fnc,
                                     const ComputeDelta<Y>& compute_delta,
                                     const ApplyDelta<X>& apply_delta) {
  const auto f_x = [&apply_delta, &compute_delta, &fnc, &x](const VecNd<X::DIM>& dx) {
    const X xplus_dx = apply_delta(x, dx);
    return compute_delta(fnc(xplus_dx), fnc(x));
  };
  return numerical_jacobian<Y::DIM>(X::Zero().eval(), f_x);
}

template <typename X, typename Y>
MatNd<Y::DIM, X::DIM> group_jacobian(const VecNd<X::DIM>& x, const GroupFnc<X, Y>& fnc) {
  const auto f_x = [&fnc, &x](const VecNd<X::DIM>& dx) {
    const X xplus_dx = apply_delta(x, dx);
    return compute_delta(fnc(xplus_dx), fnc(x));
  };
  return numerical_jacobian<Y::DIM>(X::Zero().eval(), f_x);
}

}  // namespace numerics