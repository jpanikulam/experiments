#include "geometry/rotation_to.hh"
#include "testing/gtest.hh"

namespace geometry {

using Vec3 = Eigen::Vector3d;

TEST(RotationToTest, identical) {
  constexpr double EPS = 1e-5;
  const jcc::Vec3 a = jcc::Vec3::UnitX();
  const jcc::Vec3 b = jcc::Vec3::UnitX();
  const SO3 b_from_a = rotation_to(a, b);

  // Should be identity
  EXPECT_LT(b_from_a.log().norm(), EPS);
}

TEST(RotationToTest, pi_over_2) {
  constexpr double EPS = 1e-5;
  const jcc::Vec3 a = jcc::Vec3::UnitX();
  const jcc::Vec3 b = jcc::Vec3::UnitY();
  const SO3 b_from_a = rotation_to(a, b);

  // Should be identity
  EXPECT_LT(std::abs(b_from_a.log().norm() - (M_PI * 0.5)), EPS);
  EXPECT_LT((b - (b_from_a * a)).norm(), EPS);
}
}  // namespace geometry