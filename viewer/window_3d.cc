// inc order weird for these guys
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "viewer/window_manager.hh"
#include "window_3d.hh"

#include "eigen_helpers.hh"
#include "gl_aliases.hh"

#include "primitives/simple_geometry_primitives.hh"

namespace gl_viewer {

void View3D::apply() {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Eigen::AngleAxisd  az_rot(azimuth, Vec3::UnitY());
  const Eigen::AngleAxisd  elev_rot(elevation, Vec3::UnitX());
  const Eigen::Quaterniond q(elev_rot * az_rot);
  const SE3                instantaneous_rotation(SO3(q), Vec3::Zero());
  const SE3                offset(SE3(SO3(), Vec3(0.0, 0.0, -1.0)));
  glTransform(camera_from_target.inverse() * instantaneous_rotation * target_from_world);
  draw_axes({target_from_world.inverse(), 0.5});

  glScaled(zoom, zoom, zoom);
  simulate();
}

void View3D::simulate() {
  const double t_now = glfwGetTime();
  const double dt    = t_now - last_update_time;
  last_update_time   = t_now;

  const VecNd<6> delta = jcc::vstack(velocity, angular_velocity) * dt;
  camera_from_target   = SE3::exp(delta) * camera_from_target;

  constexpr double translation_damping = 0.95;
  constexpr double rotation_damping    = 0.98;

  velocity *= translation_damping;
  angular_velocity *= rotation_damping;
}

void Window3D::spin_until_step() {
  while (!should_continue_ && WindowManager::any_windows()) {
    WindowManager::draw();
  }
  should_continue_ = false;
}

void Window3D::on_key(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    if (key == static_cast<int>('N')) {
      should_continue_ = true;
    }
  }
}
void Window3D::on_mouse_button(int button, int action, int mods) {
  mouse_pos_last_click_ = mouse_pos();
}

void Window3D::on_mouse_move(const WindowPoint& mouse_pos) {
  if (left_mouse_held()) {
    const Vec2 motion     = mouse_pos.point - mouse_pos_last_click_.point;
    mouse_pos_last_click_ = mouse_pos;

    view_.azimuth += motion(0) * 0.005;
    view_.elevation += motion(1) * 0.005;

    if (view_.azimuth > M_PI) {
      view_.azimuth = -M_PI;
    } else if (view_.azimuth < -M_PI) {
      view_.azimuth = M_PI;
    }

    if (view_.elevation > M_PI_2) {
      view_.elevation = M_PI_2;
    } else if (view_.elevation < -M_PI_2) {
      view_.elevation = -M_PI_2;
    }
  }

  if (right_mouse_held()) {
    const Vec2 motion     = mouse_pos.point - mouse_pos_last_click_.point;
    mouse_pos_last_click_ = mouse_pos;

    const Vec3 motion_camera_frame(motion.x(), -motion.y(), 0.0);

    const Eigen::AngleAxisd  az_rot(view_.azimuth, Vec3::UnitY());
    const Eigen::AngleAxisd  elev_rot(view_.elevation, Vec3::UnitX());
    const Eigen::Quaterniond q(elev_rot * az_rot);
    const SE3                instantaneous_rotation(SO3(q), Vec3::Zero());

    const Vec3 motion_target_frame = instantaneous_rotation.inverse() * (motion_camera_frame * 0.005);
    view_.target_from_world.translation() += motion_target_frame;
  }
}

void Window3D::on_scroll(const double amount) {
  const double scroll_acceleration = 0.8;
  view_.zoom *= std::exp(amount);
  if (view_.zoom <= 0.001) {
    view_.zoom = 0.001;
  }
}

void Window3D::resize(const GlSize& gl_size) {
  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
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

void Window3D::render() {
  pre_render();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, (static_cast<double>(gl_size_.width) / static_cast<double>(gl_size_.height)), 0.001, 1000.0);

  // Update projection
  projection_ = Projection::get_from_current();

  for (const auto& primitive : primitives_) {
    primitive->draw();
  }

  apply_keys_to_view();
  view_.apply();

  glFlush();
  glFinish();
}

void Window3D::apply_keys_to_view() {
  const auto   keys         = held_keys();
  const double acceleration = 0.1;

  Vec3 delta_vel = Vec3::Zero();
  for (const auto& key_element : keys) {
    const bool held = key_element.second;
    const int  key  = key_element.first;

    if (!held) {
      continue;
    }

    switch (key) {
      case (static_cast<int>('W')):
        delta_vel(2) += acceleration;
        break;

      case (static_cast<int>('A')):
        delta_vel(0) += acceleration;
        break;

      case (static_cast<int>('S')):
        delta_vel(2) -= acceleration;

        break;
      case (static_cast<int>('D')):
        delta_vel(0) -= acceleration;
        break;

      // ctrl
      case 341:
        delta_vel(1) += acceleration;
        break;

      case 32:
        delta_vel(1) -= acceleration;
        break;
    }
  }

  view_.velocity -= delta_vel;
}

struct Window3DGlobalState {
  std::map<std::string, std::shared_ptr<Window3D>> windows;
};

Window3DGlobalState window_3d_state;

std::shared_ptr<Window3D> get_window3d(const std::string& title) {
  const auto it = window_3d_state.windows.find(title);
  if (it != window_3d_state.windows.end()) {
    return it->second;
  } else {
    auto window                    = std::make_shared<Window3D>();
    window_3d_state.windows[title] = window;
    WindowManager::register_window(GlSize(640, 640), window, title);
    return window;
  }
}
}