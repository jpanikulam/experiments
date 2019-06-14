#pragma once

#include "planning/body.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace planning {
void put_body(viewer::SimpleGeometry& geo,
              const Body& body,
              const Eigen::Vector4d& color = jcc::Vec4(1.0, 1.0, 1.0, 1.0));
}  // namespace planning