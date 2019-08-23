#include "geometry/spatial/time_interpolator.hh"
#include "logging/assert.hh"

namespace geometry {
namespace spatial {

double TimeInterpolator::time_offset(const jcc::TimePoint& time_stamp) const {
  return estimation::to_seconds(time_stamp - first_time_);
}

TimeInterpolator::TimeInterpolator(
    const std::vector<TimeControlPoint>& time_control_points) {
  JASSERT_FALSE(time_control_points.empty(), "Cannot interpolate empty control points");
  first_time_ = time_control_points.front().time_stamp;

  std::vector<ControlPoint> control_points;

  for (const auto& tcp : time_control_points) {
    ControlPoint control_point;
    control_point.parameter = time_offset(tcp.time_stamp);
    control_point.value = tcp.value;
    control_points.push_back(control_point);
  }

  const auto sorted_control_points = sort_control_points(control_points);
  interpolator_ = std::make_shared<LinearInterpolator>(sorted_control_points);
}
jcc::Optional<jcc::Vec3> TimeInterpolator::operator()(
    const jcc::TimePoint& time_stamp) const {
  const double t = time_offset(time_stamp);
  return (*interpolator_)(t);
}

std::size_t TimeInterpolator::count_between(const jcc::TimePoint& start,
                                            const jcc::TimePoint& end) const {
  return interpolator_->count_between(time_offset(start), time_offset(end));
}

}  // namespace spatial
}  // namespace geometry
