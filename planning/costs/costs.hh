#include <cmath>

namespace planning {

inline double huber(double x, double k) {
  constexpr double half = 0.5;
  const double abs_x = std::abs(x);
  if (abs_x < k) {
    return (x * x) * half;
  } else {
    return k * (abs_x - (k * half));
  }
}

inline double square(double x) {
  return x * x;
}

inline double quad_hinge(double x, double k) {
  return square(std::max(x - k, 0.0));
}

inline double huber_hinge(double x, double hinge_k, double huber_k) {
  return huber(std::max(x - hinge_k, 0.0), huber_k);
}

}  // namespace planning
