#include "numerics/functions/piecewise_linear.hh"

#include "logging/assert.hh"

namespace numerics {

IncreasingPiecewiseLinearFunction1d::IncreasingPiecewiseLinearFunction1d(
    const std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint>& points) {
  JASSERT_GE(points.size(), 2u, "Must have at least 2 points");

  for (std::size_t k = 1; k < points.size(); ++k) {
    JASSERT_LT(points[k - 1].x, points[k].x,
               "Points must be monotonically increasing in X coordinate");
    JASSERT_LT(points[k - 1].y, points[k].y,
               "Points must be monotonically increasing in Y coordinate");
    const double slope =
        (points[k].y - points[k - 1].y) / (points[k].x - points[k - 1].x);
    augmented_points_.push_back({points[k - 1].x, points[k - 1].y, slope});
  }
  {
    const double last_slope = augmented_points_.back().slope;
    augmented_points_.push_back({points.back().x, points.back().y, last_slope});
  }
}

IncreasingPiecewiseLinearFunction1d IncreasingPiecewiseLinearFunction1d::inverse() const {
  std::vector<IncreasingPiecewiseLinearFunction1d::ControlPoint> control_points;
  for (const auto& point : augmented_points_) {
    control_points.push_back({point.y, point.x});
  }
  return IncreasingPiecewiseLinearFunction1d(control_points);
}

double IncreasingPiecewiseLinearFunction1d::at(const double x) const {
  JASSERT_GE(x, augmented_points_.front().x,
             "x must be after the first point in the list");
  JASSERT_LE(x, augmented_points_.back().x,
             "x must be before the last point in the list");

  return (*this)(x);
}

// TODO: Use binary search
double IncreasingPiecewiseLinearFunction1d::operator()(const double x) const {
  for (std::size_t k = 0u; k < augmented_points_.size() - 1; ++k) {
    const auto& next_pt = augmented_points_[k + 1];
    if (next_pt.x >= x) {
      const auto& cur_pt = augmented_points_[k];
      const double x_offset = x - cur_pt.x;
      const double value = (cur_pt.slope * x_offset) + cur_pt.y;
      return value;
    }
  }
  std::cout << "Failing over" << std::endl;

  const auto& first_pt = augmented_points_.front();
  return first_pt.slope * (x - first_pt.x) + first_pt.y;
}

}  // namespace numerics