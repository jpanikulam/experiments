#pragma once

#include "eigen.hh"

#include "Eigen/Dense"

namespace geometry {

template <int ROWS>
VecNd<ROWS> perp(const VecNd<ROWS>& vec) {
  using Vec = VecNd<ROWS>;

  //
  // Gram-schmidt
  //

  int min_coeff_ind;
  vec.minCoeff(&min_coeff_ind);

  const Vec vec_u = vec.normalized();

  const Vec unit = Vec::Unit(min_coeff_ind);
  return (unit - (vec_u * unit.dot(vec_u))).normalized();
}

template <int ROWS>
bool is_perp(const VecNd<ROWS>& a, const VecNd<ROWS>& b) {
  constexpr double EPS = 1e-9;
  if (std::abs(a.dot(b)) > EPS) {
    return false;
  }
  return true;
}
}
