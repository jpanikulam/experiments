#pragma once

#include "estimation/time_point.hh"
#include "geometry/spatial/interpolator.hh"

#include <memory>

namespace geometry {
namespace spatial {

struct TimeControlPoint {
  estimation::TimePoint time_stamp;
  jcc::Vec3 value;
};

class TimeInterpolator {
 public:
  TimeInterpolator(const std::vector<TimeControlPoint>& control_points);
  jcc::Optional<jcc::Vec3> operator()(const estimation::TimePoint& time_stamp) const;

  std::size_t count_between(const estimation::TimePoint& start,
                            const estimation::TimePoint& end) const;

 private:
  double time_offset(const estimation::TimePoint& time_stamp) const;


  // This is only used to key the interpolator
  estimation::TimePoint first_time_;

  std::shared_ptr<Interpolator> interpolator_;
};

}  // namespace spatial
}  // namespace geometry
