#include "util/time_point.hh"

std::ostream& operator<<(std::ostream& os, const jcc::TimePoint& t) {
  os << std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch())
            .count();
  return os;
}

std::ostream& operator<<(std::ostream& os, const jcc::TimeDuration& dt) {
  os << jcc::to_seconds(dt) << "s";
  return os;
}
