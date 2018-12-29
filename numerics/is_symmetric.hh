#pragma once

#include "eigen.hh"

namespace numerics {
template <int rows>
bool is_symmetric(const MatNd<rows, rows>& mat, const double eps = 1e-5) {
  return (mat.transpose() - mat).template lpNorm<Eigen::Infinity>() < eps;
}
}  // namespace numerics