#include "plot.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

// TODO
#include <iostream>

namespace gl_viewer {

namespace {

void draw_surface(const Surface& surface) {
  glBegin(GL_QUADS);

  const std::vector<std::array<int, 2>> offsets = {{0, -1}, {-1, -1}, {-1, 0}};
  const double max   = surface.surface.maxCoeff();
  const double min   = surface.surface.minCoeff();
  const double range = max - min;

  constexpr double EPS        = 1e-6;
  const double     normalizer = range < EPS ? 1.0 : 1.0 / range;

  for (int row = 1; row < surface.surface.rows(); ++row) {
    for (int col = 1; col < surface.surface.cols(); ++col) {
      const double value = surface.surface(row, col);
      glVertex3d(row * surface.scale, col * surface.scale, value);

      glColor4f(normalizer * value, (normalizer * value * 0.5) + 0.2, (normalizer * value * 0.25) + 0.2, 0.8);

      for (const auto& r : offsets) {
        const int& x = r[0];
        const int& y = r[1];

        const int row_plus_x = row + x;
        const int col_plus_y = col + y;

        const double value = surface.surface(row_plus_x, col_plus_y);

        glVertex3d(row_plus_x * surface.scale, col_plus_y * surface.scale, value);
      }
    }
  }

  glEnd();
}
}  // namespace

void Plot::draw() const {
  for (const auto& surface : surfaces_) {
    draw_surface(surface);
  }
}
}  // namespace gl_viewer
