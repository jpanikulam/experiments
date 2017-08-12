#pragma once

#include "eigen.hh"

// TODO
#include <iostream>

namespace numerics {

namespace {

template <int ROWS>
double compute_line_minimization(const VecNd<ROWS> &gradient, const double dqd, const VecNd<ROWS> &d) {
  const double alpha = d.dot(-gradient) / dqd;
  return alpha;
}
}

//
// Solve the implied quadratic minimization problem via conjugate gradient
//
template <int ROWS>
VecNd<ROWS> conjugate_gradient_solve(const MatNd<ROWS, ROWS> &Q, const VecNd<ROWS> b) {
  // We can solve it in at most N iterations, eh?
  using SolnVec           = VecNd<ROWS>;
  constexpr int NUM_ITERS = ROWS;

  std::array<double, NUM_ITERS>      dQd;
  std::array<VecNd<ROWS>, NUM_ITERS> Qd;
  std::array<VecNd<ROWS>, NUM_ITERS> directions;

  SolnVec x = SolnVec::Random();
  SolnVec gram_schmidt_accum;

  for (int k = 0; k < NUM_ITERS; ++k) {
    const SolnVec gradient = (Q * x) - b;

    gram_schmidt_accum.setZero();
    for (int j = 0; j < k; ++j) {
      gram_schmidt_accum += (gradient.dot(Qd[j]) / dQd[j]) * directions[j];
    }

    const SolnVec direction_k = -gradient + gram_schmidt_accum;
    Qd[k]                     = Q * direction_k;
    dQd[k]                    = direction_k.transpose() * Qd[k];
    directions[k]             = direction_k;
    const double alpha        = compute_line_minimization(gradient, dQd[k], direction_k);

    x += alpha * direction_k;
  }
  return x;
}
}
