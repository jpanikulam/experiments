#include "conjugate_gradient.hh"

#include "testing/gtest.hh"

#include <chrono>

namespace numerics {

TEST(ConjugateGradient, minimizes_error) {
  constexpr int            PROBLEM_DIM = 50;
  const VecNd<PROBLEM_DIM> b           = VecNd<PROBLEM_DIM>::Random();

  // Random square matrices are a.e. invertible
  const MatNd<PROBLEM_DIM, PROBLEM_DIM> A = MatNd<PROBLEM_DIM, PROBLEM_DIM>::Random();
  const MatNd<PROBLEM_DIM, PROBLEM_DIM> Q = A.transpose() * A;
  VecNd<PROBLEM_DIM>       sum_soln;
  const VecNd<PROBLEM_DIM> true_soln = Q.inverse() * b;

  auto t1 = std::chrono::high_resolution_clock::now();
  for (int k = 0; k < 2000; ++k) {
    const VecNd<PROBLEM_DIM> soln = conjugate_gradient_solve(Q, b);
    sum_soln += soln;

    constexpr double EPS = 1e-4;
    EXPECT_LT((true_soln - soln).norm(), EPS);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "f() took " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
            << " milliseconds\n";
  std::cout << "soln: " << sum_soln.norm() << std::endl;
}
}
