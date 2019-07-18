#include "numerics/functions/piecewise_linear.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(TestPiecewiseLinear, inverse) {
  // Setup
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> pts;
  pts.push_back({0.0, 1.0});
  pts.push_back({1.0, 3.0});
  pts.push_back({3.0, 3.1});
  const IncreasingPiecewiseLinearFunction1d pwl(pts);

  // Test
  const auto inv = pwl.inverse();

  // Verification
  for (double x : {0.00, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0}) {
    constexpr double EPS = 1e-6;
    EXPECT_NEAR(inv.at(pwl.at(x)), x, EPS);
  }
}

TEST(TestPiecewiseLinear, evaluation) {
  // Setup
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> pts;
  pts.push_back({0.0, 1.0});
  pts.push_back({1.0, 3.0});
  pts.push_back({3.0, 3.1});

  // Test
  const IncreasingPiecewiseLinearFunction1d pwl(pts);

  // Verification
  for (const auto& pt : pts) {
    EXPECT_DOUBLE_EQ(pwl.at(pt.x), pt.y);
  }
}

}  // namespace numerics
