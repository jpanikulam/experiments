#pragma once

#include <string>
#include <vector>

namespace jcc {

std::string ascii_histogram(const std::vector<double> &v, int n_bins = 10);

// End is inclusive
std::string ascii_histogram(const std::vector<double> &v,
                            double start,
                            double end,
                            int n_bins);
}  // namespace jcc