#pragma once

#include "geometry/kinematics/transform_network.hh"
#include "viewer/primitives/simple_geometry.hh"

#include <memory>
#include <string>

namespace geometry {

void put_transform_network(viewer::SimpleGeometry& geo,
                           const TransformNetwork& tfn,
                           const std::string& root_node);

}  // namespace geometry
