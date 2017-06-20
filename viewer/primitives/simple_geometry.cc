#include "simple_geometry.hh"

namespace gl_viewer {

void SimpleGeometry::draw() const {
  for (const auto& axes : axes_) {
    draw_axes(axes);
  }

  for (const auto& points : points_) {
    draw_points(points);
  }

  for (const auto& points2d : points2d_) {
    draw_points2d(points2d);
  }

  draw_lines(lines_);
}
}
