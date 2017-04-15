#include "numdiff.hh"

#include <gtest/gtest.h>

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
namespace numerics {
namespace {
double gfunc(const Vec2 &z) {
  return z(0) * z(0) + 5 * z(1);
}
}

TEST(NumericalGradient, gfunc) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const Vec2 x(1.0, 0.0);
  const Vec2 expected(2.0, 5.0);

  //
  // Action
  //

  const Vec2 J = numerical_gradient<2>(x, gfunc);

  //
  // Verification
  //

  // max error
  EXPECT_LT((J - expected).lpNorm<Eigen::Infinity>(), EPS);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}