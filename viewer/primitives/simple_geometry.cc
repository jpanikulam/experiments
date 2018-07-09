#include "eigen_helpers.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/colors/viridis.hh"

namespace viewer {

void SimpleGeometry::add_colored_points(const Points &points,
                                        const std::vector<double> &intensities) {
  ColoredPoints colored_points;
  colored_points.colors.reserve(intensities.size());
  assert(points.points.size() == intensities.size());

  colored_points.points = points.points;
  colored_points.size = points.size;

  double max = 0.01;
  for (std::size_t k = 0; k < intensities.size(); ++k) {
    max = std::max(intensities[k], max);
  }

  const double inv_max = 1.0 / max;
  for (std::size_t k = 0; k < intensities.size(); ++k) {
    // new_intensities[k] = points.intensities[k] * inv_max;
    const Vec4 color = jcc::augment(colors::viridis(intensities[k] * inv_max), 0.8);
    colored_points.colors.push_back(color);
  }

  std::lock_guard<std::mutex> lk(draw_mutex_);
  colored_points_.push_back(colored_points);
}

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

  for (const auto &polygon : polygons_) {
    draw_polygon(polygon);
  }

  for (const auto &colored_points : colored_points_) {
    draw_colored_points(colored_points);
  }

  draw_lines(lines_);
}
}  // namespace viewer
