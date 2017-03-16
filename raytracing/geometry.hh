#pragma once

#include "out.hh"

#include <Eigen/Dense>

namespace raytrace {

namespace {
using Vec2 = Eigen::Vector2d;
}

struct Ray {
  Ray(const Vec2 &origin_, const Vec2 &direction_) : origin(origin_), direction(direction_.normalized()) {
  }
  const Vec2 origin;
  const Vec2 direction;
};

struct Line {
  Line(const Vec2 &point_, const Vec2 &direction_) : point(point_), direction(direction_.normalized()) {
  }
  const Vec2 point;
  const Vec2 direction;
};

struct LineSegment {
  LineSegment(const Vec2 &start_, const Vec2 &end_) : start(start_), end(end_) {
  }
  const Vec2 start;
  const Vec2 end;
};

bool ray_line_intersection(const Ray &ray, const Line &line, Out<Vec2> intersection);

bool ray_line_segment_intersection(const Ray &ray, const LineSegment &segment, Out<Vec2> intersection);
}