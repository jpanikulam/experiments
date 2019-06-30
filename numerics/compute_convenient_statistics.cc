#include "numerics/compute_convenient_statistics.hh"

#include <limits>

namespace numerics {

ConvenientStatistics compute_convenient_statistics(const std::vector<double>& values) {
  double max = std::numeric_limits<double>::max();
  double min = std::numeric_limits<double>::min();

  const double n_values = static_cast<double>(values.size());
  double sum = 0.0;
  for (const double x : values) {
    max = std::max(x, max);
    min = std::min(x, min);

    sum += x;
  }

  const double mean = sum / n_values;
  double sum_of_sq_err = 0.0;
  for (const double x : values) {
    sum_of_sq_err += (x - mean) * (x - mean);
  }

  const double variance = sum_of_sq_err / n_values;

  const ConvenientStatistics stats{
      .max = max, .min = min, .mean = mean, .variance = variance};
  return stats;
}

}  // namespace numerics
