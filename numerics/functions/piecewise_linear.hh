#pragma once

#include <vector>

namespace numerics {

class IncreasingPiecewiseLinearFunction1d {
 public:
  struct ControlPoint {
    double x;
    double y;
  };

  IncreasingPiecewiseLinearFunction1d() = default;
  IncreasingPiecewiseLinearFunction1d(const std::vector<ControlPoint>& points);

  IncreasingPiecewiseLinearFunction1d inverse() const;

  double operator()(const double x) const;
  double at(const double x) const;

 private:
  struct SuperControlPoint {
    double x;
    double y;
    double slope;
  };
  // Include a slope cache!
  std::vector<SuperControlPoint> augmented_points_;
};
}  // namespace numerics