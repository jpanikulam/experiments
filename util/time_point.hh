#pragma once

#include <chrono>
#include <iostream>

namespace jcc {

using TimePoint = std::chrono::system_clock::time_point;
using TimeDuration = TimePoint::duration;

constexpr TimeDuration to_duration(const double sec) {
  const auto dbl_dur = std::chrono::duration<double, std::ratio<1>>(sec);
  const auto real_dur = std::chrono::duration_cast<TimeDuration>(dbl_dur);
  return real_dur;
}

constexpr double to_seconds(const TimeDuration& t) {
  constexpr double SECONDS_PER_MICROSECOND = 1e-6;
  const double microseconds = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(t).count());
  return microseconds * SECONDS_PER_MICROSECOND;
}

constexpr TimePoint average(const TimePoint& t0, const TimePoint& t1) {
  const double diff = to_seconds(t1 - t0);
  const auto t_avg = t0 + to_duration(diff * 0.5);
  return t_avg;
}

constexpr TimePoint from_nanoseconds(const long int time_nanoseconds) {
  // const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(t);
  const auto nanos = std::chrono::nanoseconds(time_nanoseconds);
  const TimePoint tp(nanos);
  return tp;
}

inline jcc::TimePoint now() {
  return std::chrono::system_clock::now();
}  // namespace jcc

}  // namespace jcc

std::ostream& operator<<(std::ostream& os, const jcc::TimePoint& t);
std::ostream& operator<<(std::ostream& os, const jcc::TimeDuration& dt);
