#pragma once

//
// You must have already included glfw
//

#include <Eigen/Dense>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

namespace gl_viewer {

// for now: Only support double
namespace {

using so2 = Sophus::SO2<double>;
using se2 = Sophus::SE2<double>;

using so3 = Sophus::SO3<double>;
using se3 = Sophus::SE3<double>;
}

//////////////////////////////////////////////////////////////////////////////
// Vertices and Vertex Manipulation
//////////////////////////////////////////////////////////////////////////////

//
// Vertex (4)
//

void glVertex(const Eigen::Vector4d &vec) {
  glVertex4d(vec.x(), vec.y(), vec.z(), vec.w());
}

void glVertex(const Eigen::Vector4f &vec) {
  glVertex4f(vec.x(), vec.y(), vec.z(), vec.w());
}

//
// Vertex (3)
//

void glVertex(const Eigen::Vector3d &vec) {
  glVertex3d(vec.x(), vec.y(), vec.z());
}

void glVertex(const Eigen::Vector3f &vec) {
  glVertex3f(vec.x(), vec.y(), vec.z());
}

//
// Vertex (2)
//

void glVertex(const Eigen::Vector2d &vec) {
  // temp for testing
  glVertex3d(vec.x(), vec.y(), 0.0);
}

void glVertex(const Eigen::Vector2f &vec) {
  glVertex2f(vec.x(), vec.y());
}

//
// Color (4)
//

void glColor(const Eigen::Vector4d &vec) {
  glColor4d(vec.x(), vec.y(), vec.z(), vec.w());
}

void glColor(const Eigen::Vector4f &vec) {
  glColor4f(vec.x(), vec.y(), vec.z(), vec.w());
}

//
// Color (3)
//

void glColor(const Eigen::Vector3d &vec) {
  glColor3d(vec.x(), vec.y(), vec.z());
}

void glColor(const Eigen::Vector3f &vec) {
  glColor3f(vec.x(), vec.y(), vec.z());
}

//
// Scale (3)
//

void glScale(const Eigen::Vector3d &vec) {
  glScaled(vec.x(), vec.y(), vec.z());
}

void glScale(const Eigen::Vector3f &vec) {
  glScalef(vec.x(), vec.y(), vec.z());
}

//////////////////////////////////////////////////////////////////////////////
// View transformation
//////////////////////////////////////////////////////////////////////////////

//
// Rotate/Translate (2D)
//

void glRotate(const so2 &T) {
  glRotated(T.log(), 0.0, 0.0, 1.0);
}

void glTranslate(const Eigen::Vector2d &vec) {
  glTranslated(vec.x(), vec.y(), 0.0);
}

void glTransform(const se2 &T) {
  glRotate(T.so2());
  glTranslate(T.translation());
}

//
// Rotate/Translate (3D)
//
void glRotate(const so3 &T) {
  const Eigen::Vector3d log = T.log();
  const double          norm = log.norm();
  const Eigen::Vector3d axis = log / norm;

  glRotated(norm, axis.x(), axis.y(), axis.z());
}

void glTranslate(const Eigen::Vector3d &vec) {
  glTranslated(vec.x(), vec.y(), vec.z());
}

void glTransform(const se3 &T) {
  // glLoadMatrixd(T.matrix().transpose().data());
  glMultMatrixd(T.matrix().transpose().data());
}
}
