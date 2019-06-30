#include "geometry/spatial/interpolator.hh"

#include <cassert>

namespace geometry {
namespace spatial {

namespace {
jcc::Vec3 lerp(const double alpha, const jcc::Vec3& a, const jcc::Vec3& b) {
  // alpha must be in [0.0, 1.0]
  assert(alpha >= 0.0);
  assert(alpha <= 1.0);
  return (a + (alpha * (b - a)));
}
}  // namespace

std::vector<ControlPoint> sort_control_points(const std::vector<ControlPoint>& points) {
  const auto cmp = [](const ControlPoint& a, const ControlPoint& b) {
    return a.parameter < b.parameter;
  };
  std::vector<ControlPoint> result = points;
  std::sort(result.begin(), result.end(), cmp);
  return result;
}

LinearInterpolator::LinearInterpolator(const std::vector<ControlPoint>& control_points)
    : control_points_(control_points) {
  //
  // Verify that the control points are sorted by spline parameter
  //
  double prev = std::numeric_limits<double>::lowest();
  for (std::size_t k = 0; k < control_points.size(); ++k) {
    const double cur = control_points.at(k).parameter;
    // Strict inequality is required
    assert(cur > prev);
    prev = cur;
  }
}

jcc::Optional<jcc::Vec3> LinearInterpolator::operator()(const double t) const {
  //
  // Compute the interpolation bracket
  //
  const auto cmp = [](const double a, const ControlPoint& b) { return a < b.parameter; };
  const auto upper_bound =
      std::upper_bound(control_points_.begin(), control_points_.end(), t, cmp);

  if (upper_bound == control_points_.end() || upper_bound == control_points_.begin()) {
    return {};
  }

  const auto lower_bound = upper_bound - 1;
  const double upper_bracket = upper_bound->parameter;
  const double lower_bracket = lower_bound->parameter;

  //
  // Compute the interpolation parameter for the bracket
  //

  const double range = upper_bracket - lower_bracket;
  const double t_offset = t - lower_bracket;
  const double interpolation_parameter = t_offset / range;

  //
  // Interpolate
  //

  return lerp(interpolation_parameter, lower_bound->value, upper_bound->value);
}

std::size_t LinearInterpolator::count_between(const double start,
                                              const double end) const {
  std::size_t count = 0u;
  for (const auto& pt : control_points_) {
    if (pt.parameter >= start && pt.parameter <= end) {
      count++;
    }
  }
  return count;
}

}  // namespace spatial
}  // namespace geometry
