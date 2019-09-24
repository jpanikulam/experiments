#pragma once

#include "viewer/primitives/geometry_buffer.hh"

#include "geometry/spatial/bounding_volume_hierarchy.hh"

namespace geometry {

spatial::BoundingVolumeHierarchy::IntersectVisitorFunction make_visit(
    viewer::GeometryBuffer &buf);

}  // namespace geometry