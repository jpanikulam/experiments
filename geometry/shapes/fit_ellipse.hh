#pragma once

#include "eigen.hh"

#include "geometry/shapes/ellipse.hh"

#include <functional>

namespace geometry {
namespace shapes {

struct EllipseFit {
  // Average residual, in ~nearly whatever units `pts` is provided in
  double average_error;

  // The ellipse that has been fit to the data
  Ellipse ellipse;
};

using Visitor = std::function<void(const EllipseFit& fit)>;

// Fit an ellipsoid to data
// pts: 3D points that belong (noisly) to the surface of an ellipsoid
//      NOTE: Must contain at *least* 6 points.
// [visitor]: A function that will be called on the current state of
//            the optimization at each iteration
EllipseFit fit_ellipse(const std::vector<jcc::Vec3>& pts,
                       bool ignore_bias = false,
                       const Visitor& visitor = {});

}  // namespace shapes
}  // namespace geometry