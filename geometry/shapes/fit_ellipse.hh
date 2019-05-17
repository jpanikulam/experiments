#pragma once

#include "eigen.hh"

#include "geometry/shapes/ellipse.hh"

#include <functional>

namespace geometry {
namespace shapes {

struct EllipseFit {
  // Reserved but unused
  double total_error;

  // The ellipse that has been fit to the data
  Ellipse ellipse;
};

using Visitor = std::function<void(const EllipseFit& fit)>;

// Fit an ellipsoid to data
// pts: 3D points that belong (noisly) to the surface of an ellipsoid
// [visitor]: A function that will be called on the current state of
//            the optimization at each iteration
EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts, const Visitor& visitor = {});

}  // namespace shapes
}  // namespace geometry