#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include <vector>
#include <functional>

namespace estimation {

struct RotationBetweenVectorSeries {
  SO3 b_from_a;
  jcc::Vec3 average_error;
  std::vector<double> weights;
  bool success;
};

using OptimizationVisitor = std::function<void(const RotationBetweenVectorSeries&)>;

RotationBetweenVectorSeries rotation_between_vector_series(
    const std::vector<jcc::Vec3>& a,
    const std::vector<jcc::Vec3>& b,
    const OptimizationVisitor& visitor);

}  // namespace estimation