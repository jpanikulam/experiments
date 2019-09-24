#pragma once

#include "eigen.hh"

namespace jcc {

template <int N>
VecNd<N> lerp(const double alpha, const VecNd<N>& a, const VecNd<N>& b) {
  return (alpha * (b - a)) + a;
}

double lerp(const double alpha, const double a, const double b) {
  return (alpha * (b - a)) + a;
}

}  // namespace jcc