#include "numerics/functions/piecewise_linear.hh"

#include "logging/assert.hh"

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

TEST(TestPiecewiseLinear, out_of_bounds_eval) {
  // Setup
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> pts;
  pts.push_back({0.0, 1.0});
  pts.push_back({1.0, 2.0});
  pts.push_back({2.0, 3.0});
  const IncreasingPiecewiseLinearFunction1d pwl(pts);

  // Test
  const double below = pwl(-1.0);
  const double above = pwl(3.0);

  // Verification
  constexpr double EPS = 1e-6;
  EXPECT_NEAR(above, 4.0, EPS);
  EXPECT_NEAR(below, 0.0, EPS);
}

TEST(TestPiecewiseLinear, not_sorted) {
  // Setup
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> pts;
  pts.push_back({0.0, 1.0});
  pts.push_back({1.0, 3.0});
  pts.push_back({0.99, 3.1});

  // Test / Verification
  // I guess GTest can't check throws on constructors...?
  const auto f = [pts]() { return IncreasingPiecewiseLinearFunction1d(pts); };
  EXPECT_THROW(f(), jcc::JccException);
}
TEST(TestPiecewiseLinear, not_monotinic) {
  // Setup
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> pts;
  pts.push_back({0.0, 1.0});
  pts.push_back({1.0, 3.0});
  pts.push_back({2.0, 2.99});

  // Test / Verification
  // I guess GTest can't check throws on constructors...?
  const auto f = [pts]() { return IncreasingPiecewiseLinearFunction1d(pts); };
  EXPECT_THROW(f(), jcc::JccException);
}

}  // namespace numerics
