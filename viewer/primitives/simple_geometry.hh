#pragma once

#include "simple_geometry_primitives.hh"

#include "primitive.hh"
#include "sophus.hh"

#include <vector>

namespace gl_viewer {

class SimpleGeometry final : public Primitive {
 public:
  void draw() const override;

  void add_axes(const Axes& axes) {
    axes_.push_back(axes);
  }

  void add_line(const Line& line) {
    lines_.push_back(line);
  }

  void add_ray(const Ray& ray) {
    lines_.push_back({ray.origin, ray.origin + (ray.direction * ray.length), ray.color, ray.width});
  }

  void add_points(const Points& points) {
    points_.push_back(points);
  }

  void add_points2d(const Points2d& points) {
    points2d_.push_back(points);
  }

 private:
  std::vector<Axes>     axes_;
  std::vector<Line>     lines_;
  std::vector<Points>   points_;
  std::vector<Points2d> points2d_;
};
}
