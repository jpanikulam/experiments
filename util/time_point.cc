#include "estimation/time_point.hh"

std::ostream& operator<<(std::ostream& os, const jcc::TimePoint& t) {
  os << std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch())
            .count();
  return os;
}

std::ostream& operator<<(std::ostream& os, const estimation::TimeDuration& dt) {
  os << estimation::to_seconds(dt) << "s";
  return os;
}
