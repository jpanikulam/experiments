#include "util/ascii/ascii_histogram.hh"

#include "util/random.hh"

#include <iostream>

int main() {
  const double mean = 0.0;
  const double variance = 2.0;
  jcc::RandomNormal rn(mean, variance);

  std::vector<double> histo_elements;

  for (int k = 0; k < 10000; ++k) {
    const double sample = rn();
    histo_elements.push_back(sample);
  }

  const std::string result = jcc::ascii_histogram(histo_elements, -3.0, 3.0, 25);
  std::cout << result << std::endl;
}