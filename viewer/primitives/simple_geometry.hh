#pragma once

#include "simple_geometry_primitives.hh"

#include "geometry/ray.hh"
#include "primitive.hh"
#include "sophus.hh"

#include <vector>

namespace gl_viewer {

class SimpleGeometry final : public Primitive {
public:
  void draw() const override;

  void add_axes(const Axes &axes) {
    axes_.push_back(axes);
  }

  void add_line(const Line &line) {
    lines_.push_back(line);
  }

  void add_ray(const Ray &ray) {
    const Eigen::Vector3d first_endpoint = ray.origin + (ray.direction * 0.9 * ray.length);
    lines_.push_back({ray.origin, ray.origin + (ray.direction * 0.9 * ray.length), ray.color, ray.width});
    const Eigen::Vector4d new_color(ray.color.y(), ray.color.x(), ray.color.z(), ray.color.w());
    lines_.push_back({first_endpoint, first_endpoint + (ray.direction * 0.1 * ray.length), new_color, 1.1 * ray.width});
  }

  void add_ray(const geometry::Ray &ray, const double length, const Eigen::Vector4d &color) {
    add_ray({ray.origin, ray.direction, length, color});
  }

  void add_points(const Points &points) {
    points_.push_back(points);
  }

  void add_points2d(const Points2d &points) {
    points2d_.push_back(points);
  }

  void add_billboard_circle(const Sphere &sphere) {
    billboard_circles_.push_back(sphere);
  }

  void add_box(const AxisAlignedBox &box) {
    using Vec3 = Eigen::Vector3d;

    add_line({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
              Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
              box.color});
    add_line({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
              Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
              Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
              box.color});
    add_line({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
              Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
              box.color});
    add_line({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
              Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
              Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
              Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
              Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
              Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
              Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
              box.color});
    add_line({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
              Vec3(box.upper.x(), box.lower.y(), box.upper.z()),
              box.color});
    add_line({Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
              Vec3(box.lower.x(), box.upper.y(), box.lower.z()),
              box.color});
  }

  void clear() {
    axes_.clear();
    lines_.clear();
    points_.clear();
    points2d_.clear();
    billboard_circles_.clear();
  }

private:
  std::vector<Axes> axes_;
  std::vector<Line> lines_;
  std::vector<Points> points_;
  std::vector<Points2d> points2d_;
  std::vector<Sphere> billboard_circles_;
};
}
