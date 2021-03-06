#pragma once

//%deps(opengl)

#include "viewer/primitives/geometry_buffer.hh"
#include "viewer/primitives/simple_geometry_primitives.hh"

#include "geometry/shapes/ray.hh"
#include "geometry/tri_mesh.hh"

#include "viewer/primitives/primitive.hh"

#include "sophus.hh"

#include <map>
#include <mutex>
#include <vector>

namespace viewer {

class SimpleGeometry final : public Primitive {
 public:
  SimpleGeometry() = default;

  void draw() const override;

  void add_axes(const Axes &axes);

  void add_line(const Line &line);

  void add_ray(const Ray &ray);

  void add_ray(const geometry::Ray &ray,
               const double length,
               const Eigen::Vector4d &color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));

  void add_polygon(const Polygon &polygon);

  void add_points(const Points &points);

  void add_point(const Point &point);

  void add_colored_points(const Points &points,
                          const std::vector<double> &intensities,
                          const std::vector<double> &sizes = {});

  void add_points2d(const Points2d &points);

  void add_sphere(const Sphere &sphere);

  void add_ellipsoid(const Ellipsoid &ellipsoid);

  void add_box(const AxisAlignedBox &box);

  void add_plane(const Plane &plane);

  void add_triangle_mesh(const TriMesh &mesh);

  void clear();

  void flip() override;

  void flush();

 private:
  GeometryBuffer back_buffer_;
  GeometryBuffer front_buffer_;

  mutable std::mutex draw_mutex_;
};
}  // namespace viewer
