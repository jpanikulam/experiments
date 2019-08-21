#include "numerics/project_psd.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(ProjectPsd, psd_unclamped) {
  using TestMat = MatNd<5, 5>;
  const TestMat m_start = TestMat::Random();
  const TestMat m_test = m_start.transpose() * m_start;
  const TestMat result = project_psd(m_test, 0.01);

  EXPECT_LT((result - m_test).lpNorm<Eigen::Infinity>(), 1e-3);
}

TEST(ProjectPsd, clamps_eigenvalues) {
  using TestMat = MatNd<5, 5>;
  const TestMat m_start = TestMat::Random();
  const TestMat m_2 = m_start.transpose() * m_start;
  const TestMat m_test = m_2 - (TestMat::Identity() * 2.0);

  const Eigen::SelfAdjointEigenSolver<TestMat> before_solver(m_test);
  const VecNd<5> eigenvalues_before = before_solver.eigenvalues();

  ASSERT_LT(eigenvalues_before[3], 0.0);

  constexpr double MIN_EIGENVALUE = 0.1;
  const TestMat result = project_psd(m_test, 0.1);
  const Eigen::SelfAdjointEigenSolver<TestMat> after_solver(result);
  const VecNd<5> eigenvalues_after = after_solver.eigenvalues();

  EXPECT_NEAR(eigenvalues_after[0], MIN_EIGENVALUE, 1e-5);
  EXPECT_NEAR(eigenvalues_after[1], MIN_EIGENVALUE, 1e-5);
  EXPECT_NEAR(eigenvalues_after[2], MIN_EIGENVALUE, 1e-5);
  EXPECT_NEAR(eigenvalues_after[3], MIN_EIGENVALUE, 1e-5);
}

}  // namespace numerics