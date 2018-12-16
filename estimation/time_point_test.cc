#include "time_point.hh"

#include "testing/gtest.hh"

namespace estimation {

TEST(TimePointTest, test_it_all_constexpr) {
  constexpr double TIME = 3.54e-5;

  constexpr auto dur = to_duration(TIME);
  constexpr double secs = to_seconds(dur);

  constexpr double EPS = 0.015;
  EXPECT_LT(std::abs(secs - TIME) / TIME, EPS);
}

TEST(TimePointTest, test_it_all) {
  constexpr double TIME = 3.54e-5;

  const auto dur = to_duration(TIME);
  const double secs = to_seconds(dur);

  constexpr double EPS = 0.015;
  EXPECT_LT(std::abs(secs - TIME) / TIME, EPS);
}
}  // namespace estimation