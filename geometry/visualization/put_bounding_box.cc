#include "geometry/visualization/put_bounding_box.hh"

namespace geometry {

void put_bounding_box(viewer::GeometryBuffer &buf,
                      const geometry::spatial::BoundingBox<3> &bbox,
                      const jcc::Vec4 &color,
                      double line_width) {
  using Vec3 = jcc::Vec3;
  const Vec3 lower = bbox.lower();
  const Vec3 upper = bbox.upper();
  buf.lines.push_back({Vec3(lower.x(), lower.y(), upper.z()),
                       Vec3(lower.x(), upper.y(), upper.z()), color, line_width});
  buf.lines.push_back({Vec3(lower.x(), lower.y(), upper.z()),
                       Vec3(lower.x(), lower.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(lower.x(), lower.y(), upper.z()),
                       Vec3(upper.x(), lower.y(), upper.z()), color, line_width});
  buf.lines.push_back({Vec3(lower.x(), upper.y(), upper.z()),
                       Vec3(upper.x(), upper.y(), upper.z()), color, line_width});
  buf.lines.push_back({Vec3(lower.x(), upper.y(), upper.z()),
                       Vec3(lower.x(), upper.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), upper.y(), upper.z()),
                       Vec3(upper.x(), upper.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), upper.y(), upper.z()),
                       Vec3(upper.x(), lower.y(), upper.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), upper.y(), lower.z()),
                       Vec3(upper.x(), lower.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), upper.y(), lower.z()),
                       Vec3(lower.x(), upper.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), lower.y(), lower.z()),
                       Vec3(lower.x(), lower.y(), lower.z()), color, line_width});
  buf.lines.push_back({Vec3(upper.x(), lower.y(), lower.z()),
                       Vec3(upper.x(), lower.y(), upper.z()), color, line_width});
  buf.lines.push_back({Vec3(lower.x(), lower.y(), lower.z()),
                       Vec3(lower.x(), upper.y(), lower.z()), color, line_width});
}

}  // namespace geometry