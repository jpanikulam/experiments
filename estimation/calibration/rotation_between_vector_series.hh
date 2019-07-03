#include "eigen.hh"
#include "sophus.hh"

#include <vector>

namespace estimation {

struct RotationBetweenVectorSeries {
  SO3 b_from_a;
  jcc::Vec3 average_error;
  bool success;
};

RotationBetweenVectorSeries rotation_between_vector_series(
    const std::vector<jcc::Vec3>& a, const std::vector<jcc::Vec3>& b);

}  // namespace estimation