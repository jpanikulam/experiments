#pragma once

#include "out.hh"

#include <Eigen/Dense>

namespace raytrace {

namespace {
using Vec2 = Eigen::Vector2d;
}

double cross2d(const Vec2 &a, const Vec2 &b);

// A proper geometric ray, that points endlessly in `direction`
//
//
struct Ray {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ray(const Vec2 &origin_, const Vec2 &direction_) : origin(origin_), direction(direction_.normalized()) {
  }
  const Vec2 origin;
  const Vec2 direction;
};

// A proper geometric line, that points endlessly in both `direction` and `-direction`
//
//
struct Line {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Line(const Vec2 &point_, const Vec2 &direction_) : point(point_), direction(direction_.normalized()) {
  }
  const Vec2 point;
  const Vec2 direction;
};

// A 2D line segment (parameterized by `start` and `end`)
//
//
struct LineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LineSegment(const Vec2 &start_, const Vec2 &end_) : start(start_), end(end_) {
  }
  const Vec2 start;
  const Vec2 end;
};

// A 2D plane section (parameterized by `normal` and `width`)
// Equivalent to a line segment, but with a "sidedness", determined by `normal`
//
struct DirectedLineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DirectedLineSegment(const Vec2 &normal_, const Vec2 &center_, const double width_)
      : normal(normal_.normalized()), center(center_), width(width_) {
  }
  const Vec2   normal;
  const Vec2   center;
  const double width;

  // Signed
  // `point`s in the halfspace indicated by `normal` will have positive distance
  double to_plane_distance(const Vec2 &point) const {
    // Project into the normal subspace
    return normal.dot(point - center);
  }

  Vec2 dto_plane_distance(const Vec2 &point) const {
    return normal;
  }

  // Signed
  // `point`s in the halfspace indicated by `normal` will have positive distance
  double along_plane_distance(const Vec2 &point) const {
    // Norm of the vector projected into the direction subspace
    return cross2d(normal, point - center);
  }

  Vec2 dalong_plane_distance(const Vec2 &point) const {
    return Vec2(-normal(1), normal(0));
  }
};

bool ray_line_intersection(const Ray &ray, const Line &line, Out<Vec2> intersection);

bool ray_line_segment_intersection(const Ray &ray, const LineSegment &segment, Out<Vec2> intersection);
}
