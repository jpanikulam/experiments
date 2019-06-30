#pragma once

#include "eigen.hh"

#include <vector>

namespace numerics {

template <int N>
VecNd<N> compute_mean(const std::vector<VecNd<N>>& vecs) {
  VecNd<N> sum = VecNd<N>::Zero();

  for (const auto& v : vecs) {
    sum += v;
  }

  return (sum / static_cast<double>(vecs.size()));
}
}  // namespace numerics