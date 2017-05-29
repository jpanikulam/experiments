#include "spatial_hash.hh"

#include <limits>
#include <unordered_set>

namespace clustering {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec2Int = Eigen::Matrix<HashInt, 2, 1>;

template <int DIM, typename Scalar = double>
class BoundingBox {
public:
  using Vec = Eigen::Matrix<Scalar, DIM, 1>;

  void expand(const Vec &point) {
    lower_ = lower_.cwiseMin(point);
    upper_ = upper_.cwiseMax(point);
  }

  const Vec &lower() {
    return lower_;
  }

  const Vec &upper() {
    return upper_;
  }

private:
  Vec lower_;
  Vec upper_;
};
}

//
std::vector<HashInt> spatial_hash(const std::vector<Vec2> &points, double scale) {

  BoundingBox<2> bbox;
  for (const auto &pt : points) {
    bbox.expand(pt);
  }

  //
  // Force into the nonnegative orthant
  //

  std::vector<HashInt> identities(points.size());

  const Vec2 range = bbox.upper() - bbox.lower();
  const Vec2 lower = bbox.lower();

  // const double int_max = std::numeric_limits<HashInt>::max() / (std::pow(2, 16));

  for (size_t k = 0; k < points.size(); ++k) {
    const Vec2 normalized = (points[k] - lower).cwiseQuotient(range) * scale;
    const Vec2Int normalized_int = normalized.cast<HashInt>();
    const HashInt hash_value = (0xFFFF0000 & (normalized_int(0) << 16)) | (0x0000FFFF & normalized_int(1));
    identities[k] = hash_value;
  }
  return identities;
}
}
