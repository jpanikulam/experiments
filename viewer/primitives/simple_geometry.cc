#include "eigen_helpers.hh"

#include "viewer/colors/viridis.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace viewer {

void SimpleGeometry::add_ray(const Ray &ray) {
  std::lock_guard<std::mutex> lk(draw_mutex_);
  const Eigen::Vector3d first_endpoint = ray.origin + (ray.direction * 0.9 * ray.length);
  back_buffer_.lines.push_back({ray.origin, first_endpoint, ray.color, ray.width});
  const Eigen::Vector4d new_color(ray.color.y(), ray.color.x(), ray.color.z(),
                                  ray.color.w());
  back_buffer_.lines.push_back({first_endpoint,
                                first_endpoint + (ray.direction * 0.1 * ray.length),
                                new_color, 1.1 * ray.width});
}

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
    const Vec4 color = jcc::augment(colors::viridis(intensities[k] * inv_max), 0.8);
    colored_points.colors.push_back(color);
  }

  std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.colored_points.push_back(colored_points);
}

void SimpleGeometry::add_box(const AxisAlignedBox &box) {
  std::lock_guard<std::mutex> lk(draw_mutex_);
  using Vec3 = Eigen::Vector3d;

  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                                Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                                Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                                Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
                                Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
                                Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
                                Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
                                Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
                                Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
                                Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
                                Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
                                Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
                                box.color});
  back_buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
                                Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
                                box.color});
}

void SimpleGeometry::draw() const {
  std::lock_guard<std::mutex> lk(draw_mutex_);

  for (const auto &axes : front_buffer_.axes) {
    draw_axes(axes);
  }

  for (const auto &points : front_buffer_.points) {
    draw_points(points);
  }

  for (const auto &point : front_buffer_.raw_points) {
    draw_point(point);
  }

  for (const auto &points2d : front_buffer_.points2d) {
    draw_points2d(points2d);
  }

  for (const auto &circle : front_buffer_.spheres) {
    draw_sphere(circle);
  }

  for (const auto &polygon : front_buffer_.polygons) {
    draw_polygon(polygon);
  }

  for (const auto &colored_points : front_buffer_.colored_points) {
    draw_colored_points(colored_points);
  }

  for (const auto &plane : front_buffer_.planes) {
    draw_plane_grid(plane);
  }

  draw_lines(front_buffer_.lines);
}
}  // namespace viewer
