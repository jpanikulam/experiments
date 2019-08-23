#include "util/ascii/ascii_histogram.hh"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace jcc {
namespace {
std::vector<int> form_histogram(const std::vector<double> &v,
                                const double start,
                                const double end,
                                const int n_bins) {
  const double bin_spacing = (end - start) / n_bins;
  const double inv_bin_spacing = start == end ? 1.0 : 1.0 / bin_spacing;

  std::vector<int> bins(n_bins, 0);

  for (const auto &val : v) {
    const double bin_d = (val - start) * inv_bin_spacing;
    const int bin_index = std::round(bin_d);
    if (bin_index >= 0 && bin_index < n_bins) {
      ++bins[bin_index];
    }
  }

  return bins;
}
}  // namespace

std::string ascii_histogram(const std::vector<double> &v, int n_bins) {
  if (v.empty()) {
    return "";
  }
  const double start = *std::min_element(v.begin(), v.end());
  const double end = *std::max_element(v.begin(), v.end());

  return ascii_histogram(v, start, end, n_bins);
}
std::string ascii_histogram(const std::vector<double> &v,
                            const double start,
                            const double end,
                            const int n_bins) {
  if (v.empty()) {
    return "";
  }
  const std::vector<int> histogram = form_histogram(v, start, end, n_bins);

  constexpr int MAX_WIDTH_CHARS = 120;

  constexpr int N_LEADING_CHARS = 1;

  constexpr int MAX_BAR_WIDTH_CHARS = MAX_WIDTH_CHARS - N_LEADING_CHARS;

  const int max_count = *std::max_element(histogram.begin(), histogram.end());
  const double scaling = static_cast<double>(MAX_BAR_WIDTH_CHARS) / max_count;

  std::stringstream ss;
  ss << "\n| " << std::to_string(start) << "\n";
  ss << "|" << std::string(MAX_WIDTH_CHARS - 1, '=') << "\n";

  for (std::size_t k = 0; k < histogram.size(); ++k) {
    const int n_chars = scaling * histogram[k];
    ss << "|";
    ss << std::string(n_chars, '#');
    ss << "\n";
  }
  ss << "|" << std::string(MAX_WIDTH_CHARS - 1, '=') << "\n";

  ss << "| " << std::to_string(end) << "\n";
  return ss.str();
}
}  // namespace jcc