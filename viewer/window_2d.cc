#include "window_2d.hh"

#include <GL/glew.h>

#include <GLFW/glfw3.h>
#include "gl_aliases.hh"

#include <iostream>

namespace gl_viewer {

void Window2D::View2D::apply() {
  //
  // Projection
  //

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, 1.0, 1.0, 1000.0);

  glMatrixMode(GL_MODELVIEW);

  // glRotated(M_PI, 0.0, 1.0, 0.0);
  // Eigen::Matrix4d mat;
  // mat.row(0) << -1.0, 0.0, 0.0, 0.0;
  // mat.row(1) << 0.0, 1.0, 0.0, 0.0;
  // mat.row(2) << 0.0, 0.0, -1.0, 0.0;
  // mat.row(3) << 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d mat;
  mat.row(0) << 1.0, 0.0, 0.0, 0.0;
  mat.row(1) << 0.0, 1.0, 0.0, 0.0;
  mat.row(2) << 0.0, 0.0, 1.0, 0.0;
  mat.row(3) << 0.0, 0.0, 0.0, 1.0;

  glLoadMatrixd(mat.transpose().data());

  glTransform(camera_pose);
  glTranslated(0.0, 0.0, -camera_height);

  // double mvm[16];
  // glGetDoublev(GL_MODELVIEW_MATRIX, mvm);
  // Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> mvm_eigen(mvm);
  // std::cout << "--" << std::endl;
  // std::cout << mvm_eigen << std::endl;
  // std::cout << "--" << std::endl;
  // std::cout << camera_pose.matrix() << std::endl;

  simulate();
}

void Window2D::View2D::simulate() {
  const double t_now = glfwGetTime();
  const double dt    = t_now - last_update_time;

  const Vec3 delta    = dt * velocity;
  const se2  expdelta = se2::exp(delta);
  camera_pose         = expdelta * camera_pose;

  camera_height += dcamera_height * dt;
}

void Window2D::on_key(int key, int scancode, int action, int mods) {
  // todo
}

void Window2D::apply_keys_to_view() {
  const auto keys = pressed_keys();

  const double acceleration        = 0.001;
  const double translation_damping = 0.9;
  const double rotation_damping    = 0.98;

  Vec3 delta_vel = Vec3::Zero();
  for (const auto& key_element : keys) {
    const bool held = key_element.second;
    const int  key  = key_element.first;

    if (!held) {
      continue;
    }

    switch (key) {
      case (static_cast<int>('W')):
        delta_vel(1) -= acceleration;
        break;
      case (static_cast<int>('A')):
        delta_vel(0) += acceleration;
        break;
      case (static_cast<int>('S')):
        delta_vel(1) += acceleration;
        break;
      case (static_cast<int>('D')):
        delta_vel(0) -= acceleration;
        break;
    }
  }

  view_.velocity += delta_vel;
  view_.velocity.head<2>() *= translation_damping;
  view_.velocity(2) *= rotation_damping;
}

void Window2D::draw_renderables(const Renderables& renderables) const {
  //
  // Draw lines
  //
  glBegin(GL_LINES);
  for (const auto line : renderables.lines) {
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
  }
  glEnd();

  //
  // Draw rays
  //

  glBegin(GL_LINES);
  for (const auto ray : renderables.rays) {
    glColor(ray.color);
    glVertex(ray.origin);

    const Vec4 v4 = (Vec4() << ray.direction, 0.0, 0.0).finished();
    glVertex(v4);
  }
  glEnd();
}

void pre_render() {
  //
  // Flag soup
  //

  glShadeModel(GL_SMOOTH);

  // Check depth when rendering
  glEnable(GL_DEPTH_TEST);

  // Turn on lighting
  // glEnable(GL_LIGHTING);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void Window2D::render() {
  pre_render();

  //
  // Draw all renderables
  //

  apply_keys_to_view();
  view_.apply();

  draw_renderables(renderables_);
}
}
