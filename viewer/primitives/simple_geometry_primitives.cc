#include "simple_geometry_primitives.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

namespace gl_viewer {

using Vec3 = Eigen::Vector3d;

void draw_axes(const Axes& axes) {
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

void draw_lines(const std::vector<Line>& lines) {
  glPushAttrib(GL_CURRENT_BIT);

  for (const auto& line : lines) {
    glLineWidth(line.width);
    glBegin(GL_LINES);
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
    glEnd();
  }

  glPopAttrib();
}

void draw_points(const Points& points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto& pt : points.points) {
    glVertex(pt);
  }
  glEnd();
  glPopAttrib();
}

void draw_points2d(const Points2d& points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto& pt : points.points) {
    glVertex3d(pt.x(), pt.y(), points.z_offset);
  }
  glEnd();
  glPopAttrib();
}
}