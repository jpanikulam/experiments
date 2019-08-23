#pragma once

#include "util/time_point.hh"
#include "geometry/spatial/interpolator.hh"

#include <memory>

namespace geometry {
namespace spatial {

struct TimeControlPoint {
  jcc::TimePoint time_stamp;
  jcc::Vec3 value;
};

class TimeInterpolator {
 public:
  TimeInterpolator(const std::vector<TimeControlPoint>& control_points);
  jcc::Optional<jcc::Vec3> operator()(const jcc::TimePoint& time_stamp) const;

  std::size_t count_between(const jcc::TimePoint& start,
                            const jcc::TimePoint& end) const;

 private:
  double time_offset(const jcc::TimePoint& time_stamp) const;


  // This is only used to key the interpolator
  jcc::TimePoint first_time_;

  std::shared_ptr<Interpolator> interpolator_;
};

}  // namespace spatial
}  // namespace geometry
