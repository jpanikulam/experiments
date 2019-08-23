#include <random>

namespace jcc {
class RandomNormal {
 public:
  RandomNormal(const double mean, const double variance, int seed = 0)
      : gen_(seed), dist_(mean, variance) {
  }

  double operator()() {
    return dist_(gen_);
  }

 private:
  std::mt19937 gen_;
  std::normal_distribution<> dist_;
};
}  // namespace jcc