#include "simple_geometry_primitives.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

#include "geometry/perp.hh"

namespace gl_viewer {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

void draw_axes(const Axes &axes) {
  glPushAttrib(GL_CURRENT_BIT);
  glLineWidth(1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTransform(axes.axes_from_world);
  glScaled(axes.scale, axes.scale, axes.scale);
  glBegin(GL_LINES);

  glColor4d(1.0, 0.0, 0.0, 0.9);
  glVertex(Vec3::UnitX().eval());
  glVertex(Vec3::Zero().eval());

  glColor4d(0.0, 1.0, 0.0, 0.9);
  glVertex(Vec3::UnitY().eval());
  glVertex(Vec3::Zero().eval());

  glColor4d(0.0, 0.0, 1.0, 0.9);
  glVertex(Vec3::UnitZ().eval());
  glVertex(Vec3::Zero().eval());
  glEnd();

  glPopMatrix();
  glPopAttrib();
}

void draw_lines(const std::vector<Line> &lines) {
  glPushAttrib(GL_CURRENT_BIT);

  for (const auto &line : lines) {
    glLineWidth(line.width);
    glBegin(GL_LINES);
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
    glEnd();
  }

  glPopAttrib();
}

void draw_polygon(const Polygon &polygon) {
  glPushAttrib(GL_CURRENT_BIT);
  glLineWidth(polygon.width);

  glBegin(GL_LINE_STRIP);
  glColor(polygon.color);
  for (const auto &point : polygon.points) {
    glVertex(point);
  }
  glEnd();

  glPopAttrib();
}

void draw_points(const Points &points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto &pt : points.points) {
    glVertex(pt);
  }
  glEnd();
  glPopAttrib();
}

void draw_points2d(const Points2d &points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto &pt : points.points) {
    glVertex3d(pt.x(), pt.y(), points.z_offset);
  }
  glEnd();
  glPopAttrib();
}

void draw_circle(const Vec3 &center, const Vec3 &normal, const double radius, const Vec4 &color) {
  constexpr double CIRCLE_RES = 0.05;

  const Vec3 x_u = geometry::perp(normal);
  const Vec3 y_u = -x_u.cross(normal);

  Eigen::Matrix3d world_from_circle_frame;
  world_from_circle_frame.col(0) = x_u;
  world_from_circle_frame.col(1) = y_u;
  world_from_circle_frame.col(2) = normal;

  glColor(color);
  glBegin(GL_LINE_LOOP);
  for (double t = 0.0; t < (2.0 * M_PI); t += CIRCLE_RES) {
    const Vec3 pt_circle_frm(radius * std::sin(t), radius * std::cos(t), 0.0);
    const Vec3 pt_world_frm = (world_from_circle_frame * pt_circle_frm) + center;
    glVertex(pt_world_frm);
  }
  glEnd();
}

void draw_billboard_circle(const Sphere &billboard_circle) {
  draw_circle(billboard_circle.center, Vec3::UnitX(), billboard_circle.radius, billboard_circle.color);
  draw_circle(billboard_circle.center, Vec3::UnitY(), billboard_circle.radius, billboard_circle.color);
  draw_circle(billboard_circle.center, Vec3::UnitZ(), billboard_circle.radius, billboard_circle.color);
}
}  // namespace gl_viewer
