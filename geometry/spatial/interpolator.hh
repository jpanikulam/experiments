#pragma once

#include "eigen.hh"
#include "util/optional.hh"

namespace geometry {
namespace spatial {

struct ControlPoint {
  double parameter;
  jcc::Vec3 value;
};

// Return a sorted vector of control points
std::vector<ControlPoint> sort_control_points(const std::vector<ControlPoint>& points);

class Interpolator {
 public:
  jcc::Optional<jcc::Vec3> interpolate(const double t) const {
    return (*this)(t);
  }
  virtual jcc::Optional<jcc::Vec3> operator()(const double t) const = 0;

  virtual std::size_t count_between(const double start, const double end) const = 0;
};

class LinearInterpolator final : public Interpolator {
 public:
  LinearInterpolator(const std::vector<ControlPoint>& control_points);

  jcc::Optional<jcc::Vec3> operator()(const double t) const override;

  std::size_t count_between(const double start, const double end) const override;

 private:
  const std::vector<ControlPoint> control_points_;
};

}  // namespace spatial
}  // namespace geometry
