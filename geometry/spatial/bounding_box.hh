#pragma once

#include "eigen.hh"
#include <limits>

namespace geometry {
namespace spatial {

template <int DIM, typename Scalar = double>
class BoundingBox {
public:
  using Vec = Eigen::Matrix<Scalar, DIM, 1>;

  void expand(const Vec &point) {
    lower_ = lower_.cwiseMin(point);
    upper_ = upper_.cwiseMax(point);
  }

  void expand(const BoundingBox<DIM> &box) {
    // lower_ = lower_.cwiseMin(box.lower());
    // upper_ = upper_.cwiseMax(box.upper());
    expand(box.lower());
    expand(box.upper());
  }

  Vec center() const {
    return (upper_ + lower_) * 0.5;
  }

  double surface_area() const {
    static_assert(DIM == 3, "Dimension must be 3 because how do I higher dimensions?");
    const Vec delta = upper_ - lower_;
    double sa = 0.0;
    for (int i = 0; i < DIM; ++i) {
      sa += 2.0 * delta(i) * delta((i + 1) % DIM);
    }

    return sa;
  }

  const Vec &lower() const {
    return lower_;
  }

  const Vec &upper() const {
    return upper_;
  }

private:
  Vec lower_ = Vec::Ones() * std::numeric_limits<double>::max();
  Vec upper_ = Vec::Ones() * std::numeric_limits<double>::lowest();
};
}
}
