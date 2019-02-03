#pragma once

#include <functional>

#include "eigen.hh"

namespace geometry {
namespace shapes {

using AffineMat3 = MatNd<3, 3>;
struct Ellipse {
  // Cholesky factor for the ellipse
  // Eigen::LLT<AffineMat3> cholesky_factor;
  AffineMat3 cholesky_factor;

  // Ellipse centroid
  jcc::Vec3 p0;

  // The ellipse is an implicit function, described as:
  // (x - p0).T * L * L.T * (x - p0) = 1.0
  //
  // Where L is the cholesky factor
};

struct EllipseFit {
  double residual;
  Ellipse ellipse;
};

using Visitor = std::function<void(const EllipseFit& fit)>;

EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts, const Visitor& visitor = {});

}  // namespace shapes
}  // namespace geometry