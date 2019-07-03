#include "testing/gtest.hh"

#include "geometry/spatial/time_interpolator.hh"

namespace geometry {
namespace spatial {

TEST(TimeInterpolatorTest, time_interpolator) {
  const estimation::TimePoint t0 = {};
  const estimation::TimeDuration dt = estimation::to_duration(1.0);

  const std::vector<geometry::spatial::TimeControlPoint> points = {
      geometry::spatial::TimeControlPoint{t0, jcc::Vec3(0.0, 0.0, 0.0)},
      geometry::spatial::TimeControlPoint{t0 + dt, jcc::Vec3(1.0, 1.0, 1.0)},
      geometry::spatial::TimeControlPoint{t0 + (2 * dt), jcc::Vec3(5.0, 5.0, 5.0)},
  };

  const TimeInterpolator interp(points);
  constexpr double EPS = 1e-6;

  {
    const auto error = (*interp(t0) - points[0].value).norm();
    EXPECT_LT(error, EPS);
  }

  {
    const auto error = (*interp(t0 + dt) - points[1].value).norm();
    EXPECT_LT(error, EPS);
  }

  {  // Nothing at the end point
    EXPECT_FALSE(interp(t0 + (2 * dt)));

    const estimation::TimePoint t = t0 + estimation::to_duration(1.9999);
    const auto interp_at_t = interp(t);
    ASSERT_TRUE(interp_at_t);
    const auto error = (*interp_at_t - points[2].value).norm();
    EXPECT_LT(error, 1e-3);
  }

  {
    const estimation::TimePoint t = t0 + estimation::to_duration(0.5);
    const auto error = (*interp(t) - jcc::Vec3(0.5, 0.5, 0.5)).norm();
    EXPECT_LT(error, EPS);
  }

  {
    const estimation::TimePoint t = t0 + estimation::to_duration(1.5);
    const auto error = (*interp(t) - jcc::Vec3(3.0, 3.0, 3.0)).norm();
    EXPECT_LT(error, EPS);
  }
}

}  // namespace spatial
}  // namespace geometry
