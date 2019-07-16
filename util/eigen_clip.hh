#pragma once

#include "eigen.hh"

namespace jcc {

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar,
              Derived::RowsAtCompileTime,
              Derived::ColsAtCompileTime>
eigen_clip(const Eigen::MatrixBase<Derived>& v,
           const Eigen::MatrixBase<Derived>& min,
           const Eigen::MatrixBase<Derived>& max) {
  return v.cwiseMax(min).cwiseMin(max).eval();
}

}  // namespace jcc