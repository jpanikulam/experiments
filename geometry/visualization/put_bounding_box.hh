#pragma once

#include "geometry/spatial/bounding_box.hh"
#include "viewer/primitives/geometry_buffer.hh"

namespace geometry {
void put_bounding_box(viewer::GeometryBuffer &buf,
                      const geometry::spatial::BoundingBox<3> &bbox,
                      const jcc::Vec4 &color,
                      double line_width = 1.0);
}  // namespace geometry