#pragma once

#include <vector>

namespace numerics {
struct ConvenientStatistics {
  double mean;
  double max;
  double min;
  double variance;
};

ConvenientStatistics compute_convenient_statistics(const std::vector<double>& values);

}  // namespace numerics