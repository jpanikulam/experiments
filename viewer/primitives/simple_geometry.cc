#include "eigen_helpers.hh"

#include "viewer/colors/viridis.hh"
#include "viewer/primitives/simple_geometry.hh"

#include <GL/glew.h>

namespace viewer {

void SimpleGeometry::add_axes(const Axes &axes) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.axes.push_back(axes);
}

void SimpleGeometry::add_line(const Line &line) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.lines.push_back(line);
}

void SimpleGeometry::add_polygon(const Polygon &polygon) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.polygons.push_back(polygon);
}

void SimpleGeometry::add_points(const Points &points) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.points.push_back(points);
}

void SimpleGeometry::add_point(const Point &point) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.raw_points.push_back(point);
}

void SimpleGeometry::add_points2d(const Points2d &points) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.points2d.push_back(points);
}

void SimpleGeometry::add_sphere(const Sphere &sphere) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.spheres.push_back(sphere);
}

void SimpleGeometry::add_ellipsoid(const Ellipsoid &ellipsoid) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.ellipsoids.push_back(ellipsoid);
}

void SimpleGeometry::add_plane(const Plane &plane) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.planes.push_back(plane);
}

void SimpleGeometry::add_triangle_mesh(const TriMesh &mesh) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.tri_meshes.push_back(mesh);
}

void SimpleGeometry::clear() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.clear();
  front_buffer_.clear();
}

void SimpleGeometry::flip() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  front_buffer_ = std::move(back_buffer_);
  front_buffer_.mesh_displaylists.clear();
  back_buffer_.mesh_displaylists.clear();
}

void SimpleGeometry::flush() {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  const auto insert = [](auto &into, const auto &from) {
    into.insert(into.begin(), from.begin(), from.end());
  };

  insert(front_buffer_.axes, back_buffer_.axes);
  insert(front_buffer_.lines, back_buffer_.lines);
  insert(front_buffer_.points, back_buffer_.points);
  insert(front_buffer_.raw_points, back_buffer_.raw_points);
  insert(front_buffer_.points2d, back_buffer_.points2d);
  insert(front_buffer_.spheres, back_buffer_.spheres);
  insert(front_buffer_.ellipsoids, back_buffer_.ellipsoids);
  insert(front_buffer_.planes, back_buffer_.planes);
  insert(front_buffer_.polygons, back_buffer_.polygons);
  insert(front_buffer_.colored_points, back_buffer_.colored_points);
  insert(front_buffer_.tri_meshes, back_buffer_.tri_meshes);

  back_buffer_.clear();

  // TODO: Why do we have to invalidate the whole cache?
  back_buffer_.mesh_displaylists.clear();
}

void SimpleGeometry::add_ray(const geometry::Ray &ray,
                             const double length,
                             const Eigen::Vector4d &color) {
  add_ray({ray.origin, ray.direction, length, color});
}

void SimpleGeometry::add_ray(const Ray &ray) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  const Eigen::Vector3d first_endpoint = ray.origin + (ray.direction * 0.9 * ray.length);
  back_buffer_.lines.push_back({ray.origin, first_endpoint, ray.color, ray.width});
  const Eigen::Vector4d new_color(ray.color.y(), ray.color.x(), ray.color.z(),
                                  ray.color.w());
  back_buffer_.lines.push_back({first_endpoint,
                                first_endpoint + (ray.direction * 0.1 * ray.length),
                                new_color, 1.1 * ray.width});
}

void SimpleGeometry::add_colored_points(const Points &points,
                                        const std::vector<double> &intensities,
                                        const std::vector<double> &sizes) {
  ColoredPoints colored_points;
  colored_points.colors.reserve(intensities.size());
  assert(points.points.size() == intensities.size());
  if (sizes.empty()) {
    colored_points.sizes = std::vector<double>(intensities.size(), points.size);
  } else {
    colored_points.sizes = sizes;
    assert(sizes.size() == intensities.size());
  }

  colored_points.points = points.points;

  double max = 0.01;
  for (std::size_t k = 0; k < intensities.size(); ++k) {
    max = std::max(intensities[k], max);
  }

  const double inv_max = 1.0 / max;
  for (std::size_t k = 0; k < intensities.size(); ++k) {
    const Vec4 color = jcc::augment(colors::viridis(intensities[k] * inv_max),
                                    0.7 + (0.3 * intensities[k] * inv_max));
    colored_points.colors.push_back(color);
  }

  const std::lock_guard<std::mutex> lk(draw_mutex_);
  back_buffer_.colored_points.push_back(colored_points);
}

void SimpleGeometry::add_box(const AxisAlignedBox &box) {
  const std::lock_guard<std::mutex> lk(draw_mutex_);
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
  const std::lock_guard<std::mutex> lk(draw_mutex_);
  draw_geometry_buffer(front_buffer_);
}
}  // namespace viewer
