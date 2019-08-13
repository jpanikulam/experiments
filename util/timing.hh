#pragma once

#include "estimation/time_point.hh"

namespace jcc {
class ScopedTimer {
 public:
  ScopedTimer();
  ~ScopedTimer();

 private:
  estimation::TimePoint start_time_;
};
}  // namespace jcc