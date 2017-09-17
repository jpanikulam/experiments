#pragma once

#include "primitive.hh"
#include "sophus.hh"

#include <vector>

namespace gl_viewer {

struct Axes {
  SE3 axes_from_world;
  double scale = 1.0;
};

struct Line {
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;
  Vec3 start;
  Vec3 end;
  Vec4 color = Vec4::Ones();
  double width = 1.0;
};

struct Ray {
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;

  Vec3 origin;
  Vec3 direction;
  double length = 1.0;
  Vec4 color = Vec4::Ones();
  double width = 1.0;
};

struct Points {
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;

  std::vector<Vec3> points;
  Vec4 color = Vec4::Ones();
  double size = 1.0;
};

struct Points2d {
  using Vec2 = Eigen::Vector2d;
  using Vec4 = Eigen::Vector4d;

  std::vector<Vec2> points;
  Vec4 color = Vec4::Ones();
  double size = 1.0;
  double z_offset = 0.0;
};

struct Sphere {
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;
  Vec3 center;
  double radius;
  Vec4 color = Vec4(0.0, 1.0, 0.0, 1.0);
};

struct AxisAlignedBox {
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;
  Vec3 lower;
  Vec3 upper;
  Vec4 color = Vec4(1.0, 0.0, 1.0, 0.6);
};

void draw_axes(const Axes &axes);

void draw_lines(const std::vector<Line> &lines);

void draw_points(const Points &points);

void draw_points2d(const Points2d &points);

void draw_billboard_circle(const Sphere &billboard_circle);
}
