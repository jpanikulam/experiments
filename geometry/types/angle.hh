#pragma once

#include <cstddef>

namespace geometry {

class Angle {
 public:
  Angle() = default;

  static Angle from_degrees(double x) {
    constexpr double RAD_PER_DEG = (M_PI / 180.0);
    const double angle_rad = x * RAD_PER_DEG;
    return Angle::from_radians(angle_rad);
  }

  static Angle from_radians(double x) {
    Angle angle;
    angle.angle_rad_ = x;
    return angle;
  }

  template <typename Scalar>
  Scalar as() const {
    return static_cast<Scalar>(angle_rad_);
  }

  double rads() const {
    return angle_rad_;
  }

 private:
  double angle_rad_ = 0.0;
  ;
};
}  // namespace geometry