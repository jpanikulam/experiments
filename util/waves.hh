#pragma once

namespace jcc {

inline double triangle_wave(double t) {
  return std::abs(t - std::floor(t) - 0.5);
}

inline double half_triangle_wave(double t) {
  return t - std::floor(t);
}

inline double square_wave(double t) {
  return 2.0 * (2.0 * std::floor(t) - std::floor(2.0 * t)) + 1.0;
}

}  // namespace jcc