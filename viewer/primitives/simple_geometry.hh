#pragma once

#include "viewer/primitives/simple_geometry_primitives.hh"

#include "geometry/ray.hh"
#include "sophus.hh"
#include "viewer/primitives/primitive.hh"

#include <mutex>
#include <vector>

namespace viewer {

class SimpleGeometry final : public Primitive {
 public:
  void draw() const override;

  void add_axes(const Axes &axes) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.axes.push_back(axes);
  }

  void add_line(const Line &line) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.lines.push_back(line);
  }

  void add_ray(const Ray &ray);

  void add_ray(const geometry::Ray &ray,
               const double length,
               const Eigen::Vector4d &color) {
    add_ray({ray.origin, ray.direction, length, color});
  }

  void add_polygon(const Polygon &polygon) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.polygons.push_back(polygon);
  }

  void add_points(const Points &points) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.points.push_back(points);
  }

  void add_point(const Point &point) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.raw_points.push_back(point);
  }

  void add_colored_points(const Points &points, const std::vector<double> &intensities);

  void add_points2d(const Points2d &points) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.points2d.push_back(points);
  }

  void add_sphere(const Sphere &sphere) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.spheres.push_back(sphere);
  }

  void add_box(const AxisAlignedBox &box);

  void add_plane(const Plane &plane) {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.planes.push_back(plane);
  }

  struct Primitives {
    std::vector<Axes> axes;
    std::vector<Line> lines;
    std::vector<Points> points;
    std::vector<Point> raw_points;
    std::vector<Points2d> points2d;
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;
    std::vector<Polygon> polygons;
    std::vector<ColoredPoints> colored_points;

    void clear() {
      axes.clear();
      lines.clear();
      points.clear();
      raw_points.clear();
      points2d.clear();
      spheres.clear();
      planes.clear();
      polygons.clear();
      colored_points.clear();
    }
  };

  void clear() {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    front_buffer_.clear();
  }

  void flip() {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    front_buffer_ = std::move(back_buffer_);
  }

  void flush() {
    std::lock_guard<std::mutex> lk(draw_mutex_);
    const auto insert = [](auto &into, const auto &from) {
      into.insert(into.begin(), from.begin(), from.end());
    };

    insert(front_buffer_.axes, back_buffer_.axes);
    insert(front_buffer_.lines, back_buffer_.lines);
    insert(front_buffer_.points, back_buffer_.points);
    insert(front_buffer_.raw_points, back_buffer_.raw_points);
    insert(front_buffer_.points2d, back_buffer_.points2d);
    insert(front_buffer_.spheres, back_buffer_.spheres);
    insert(front_buffer_.planes, back_buffer_.planes);
    insert(front_buffer_.polygons, back_buffer_.polygons);
    insert(front_buffer_.colored_points, back_buffer_.colored_points);
  }

 private:
  Primitives back_buffer_;
  Primitives front_buffer_;

  mutable std::mutex draw_mutex_;
};
}  // namespace viewer
