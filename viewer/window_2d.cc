#include "window_2d.hh"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "gl_aliases.hh"
#include "spatial_geometry/plane.hh"

#include <iostream>

namespace gl_viewer {

void Window2D::View2D::apply() {
  //
  // Projection
  //

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTranslated(0.0, 0.0, -camera_height);
  glScaled(zoom, zoom, zoom);
  glRotate(camera_pose.so2().inverse());
  glTranslate((-camera_pose.translation()).eval());

  simulate();
}

void Window2D::View2D::simulate() {
  //
  // Deal with time
  //

  const double t_now = glfwGetTime();
  const double dt    = t_now - last_update_time;

  //
  // Apply the current transform and derivatives
  //

  const Vec3 delta    = dt * velocity;
  const se2  expdelta = se2::exp(delta);
  camera_pose         = expdelta * camera_pose;

  camera_height += dcamera_height * dt;
  zoom *= std::exp(0.2 * dzoom * dt);
  // std::cout << zoom << std::endl;

  //
  // Apply damping
  //

  constexpr double translation_damping = 0.95;
  constexpr double rotation_damping    = 0.98;
  constexpr double scroll_damping      = 0.90;
  constexpr double zoom_damping        = 0.8;

  velocity.head<2>() *= translation_damping;
  velocity(2) *= rotation_damping;

  dcamera_height *= scroll_damping;
  dzoom *= zoom_damping;
}

void Window2D::on_key(int key, int scancode, int action, int mods) {
  if (action == GLFW_RELEASE) {
    Projection::get_from_current();
  }
}

void Window2D::on_mouse_button(int button, int action, int mods) {
  if (left_mouse_held()) {
  }
}

void Window2D::on_mouse_move(const WindowPoint& position) {
  if (left_mouse_held()) {
    const geometry::Ray   mouse_ray = projection_.unproject(position);
    const geometry::Plane plane({Vec3::Zero(), Vec3::UnitZ()});

    Vec3 intersection;
    if (plane.intersect(mouse_ray, out(intersection))) {
      const double t_now = glfwGetTime();
      const Vec4   color((std::sin(t_now / 5.0) + 1.0) * 0.5, (std::sin(t_now / 1.0) + 1.0) * 0.5, 0.2, 0.9);

      add_circle({Vec2(intersection.x(), intersection.y()), 0.3, color});
    }
  }
}

void Window2D::on_scroll(const double amount) {
  const double scroll_acceleration = 0.01;
  view_.dzoom += amount * scroll_acceleration;
}

void Window2D::resize(const GlSize& gl_size) {
  glViewport(0, 0, gl_size.height, gl_size.width);
  gl_size_ = gl_size;
}

void Window2D::apply_keys_to_view() {
  const auto keys = held_keys();

  const double acceleration = 0.001 / view_.zoom;

  Vec3 delta_vel = Vec3::Zero();
  for (const auto& key_element : keys) {
    const bool held = key_element.second;
    const int  key  = key_element.first;

    if (!held) {
      continue;
    }

    switch (key) {
      case (static_cast<int>('W')):
        delta_vel(1) += acceleration;
        break;

      case (static_cast<int>('A')):
        delta_vel(0) -= acceleration;
        break;

      case (static_cast<int>('S')):
        delta_vel(1) -= acceleration;

        break;
      case (static_cast<int>('D')):
        delta_vel(0) += acceleration;
        break;
    }
  }

  view_.velocity += delta_vel;
}

void Window2D::draw_renderables(const Renderables& renderables) const {
  //
  // Draw lines
  //

  glBegin(GL_LINES);
  for (const auto& line : renderables.lines) {
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
  }
  glEnd();

  //
  // Draw rays
  //

  glBegin(GL_LINES);
  for (const auto& ray : renderables.rays) {
    glColor(ray.color);
    glVertex(ray.origin);

    const Vec4 v4 = (Vec4() << ray.direction, 0.0, 0.0).finished();
    glVertex(v4);
  }
  glEnd();

  //
  // Draw circles
  //

  for (const auto& circle : renderables.circles) {
    const double circumference            = circle.radius * 2.0 * M_PI;
    const int    num_segments             = 30 * static_cast<int>(circumference);
    const double segment_angular_fraction = 2.0 * M_PI / num_segments;

    glColor(circle.color);
    glBegin(GL_LINE_LOOP);
    for (double angular_fraction = 0.0; angular_fraction < (2.0 * M_PI); angular_fraction += segment_angular_fraction) {
      const double x = (circle.radius * std::cos(angular_fraction)) + circle.center.x();
      const double y = (circle.radius * std::sin(angular_fraction)) + circle.center.y();

      glVertex2d(x, y);
    }
    glEnd();
  }
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

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void Window2D::render() {
  pre_render();

  //
  // Apply zoom
  //

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, static_cast<double>(gl_size_.height) / static_cast<double>(gl_size_.width), 0.1, 100.0);

  //
  // Compute ground plane intersection
  //

  projection_ = Projection::get_from_current();

  //
  // Draw all renderables
  //

  apply_keys_to_view();
  view_.apply();

  // glScaled(view_.zoom, view_.zoom, view_.zoom);
  draw_renderables(renderables_);
}
}
