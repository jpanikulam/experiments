#include "simple_geometry.hh"

namespace gl_viewer {

void SimpleGeometry::draw() const {
  std::lock_guard<std::mutex> lk(draw_mutex_);

  for (const auto &axes : axes_) {
    draw_axes(axes);
  }

  for (const auto &points : points_) {
    draw_points(points);
  }

  for (const auto &points2d : points2d_) {
    draw_points2d(points2d);
  }

  for (const auto &circle : billboard_circles_) {
    draw_billboard_circle(circle);
  }

  draw_lines(lines_);
}
}  // namespace gl_viewer
