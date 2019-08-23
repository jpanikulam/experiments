#pragma once

#include "estimation/time_point.hh"

#include <string>

namespace jcc {
class PrintingScopedTimer {
 public:
  PrintingScopedTimer(const std::string& title = "Timer");
  ~PrintingScopedTimer();

 private:
  jcc::TimePoint start_time_;
  std::string title_;
};
}  // namespace jcc