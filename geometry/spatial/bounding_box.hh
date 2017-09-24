#pragma once

#include "geometry/ray.hh"
#include "intersection.hh"

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

  // Note: It looks like it would be easy check that an intersection took place with no divisions
  // (Just don't compute the distance, only verify that the ray passes through at all)
  Intersection intersect(const Ray &ray) {
    static_assert(DIM == 3, "Dimension must be 3 because rays are of dim 3 so");
    Intersection intersection;

    const double distance = std::numeric_limits<double>::max();
    int faces = 0;
    for (int i = 0; i < DIM; ++i) {
      const double inv_ray_dir_i = 1.0 / ray.direction(i);
      const double u_t = (upper_(i) - ray.origin(i)) * inv_ray_dir_i;
      const double l_t = (lower_(i) - ray.origin(i)) * inv_ray_dir_i;

      if (u_t > 0.0 && l_t > 0.0) {
        faces += 2;
        distance = std::min(std::min(u_t, l_t), distance);
      } else if (u_t > 0.0 || l_t > 0.0) {
        ++faces;
        distance = std::min(std::max(u_t, l_t), distance);
      }
    }
    intersection.intersected = set;
    intersection.distance = distance;
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
