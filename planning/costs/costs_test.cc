#include "planning/costs/costs.hh"

#include "testing/gtest.hh"

namespace planning {

TEST(QuadHingeTest, qhinge) {
  EXPECT_DOUBLE_EQ(quad_hinge(10.0, 0.0), square(10.0));

  EXPECT_DOUBLE_EQ(quad_hinge(0.25, 0.0), square(0.25));

  std::cout << quad_hinge(0.25, 0.0) << std::endl;

  EXPECT_EQ(quad_hinge(10.0, 15.0), 0.0);
  EXPECT_GT(quad_hinge(10.0, 5.0), quad_hinge(10.0, 9.0));
}

}  // namespace planning