#include "geometry/perp.hh"
#include "testing/gtest.hh"

// TODO: Remove
#include <iostream>

using Vec3 = Eigen::Vector3d;
namespace geometry {
TEST(Perp, is_perpendicular) {
  //
  //
  //

  const Vec3 a = Vec3::UnitX();
  EXPECT_TRUE(is_perp(perp(a), a));

  constexpr int NUM_TESTS = 200;

  for (int k = 0; k < NUM_TESTS; ++k) {
    const Vec3 x      = Vec3::Random() * k;
    const Vec3 perp_x = perp(x);
    EXPECT_TRUE(is_perp(perp_x, x));
  }
}
}
