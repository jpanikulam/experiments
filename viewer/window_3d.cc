// inc order weird for these guys
#include <GL/glew.h>
#include <GLFW/glfw3.h>

//%deps(opengl, glfw)

#include "viewer/gl_aliases.hh"
#include "viewer/primitives/simple_geometry_primitives.hh"
#include "viewer/rendering/pre_render.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include "eigen_helpers.hh"
#include "util/time_point.hh"

#include "third_party/imgui/imgui.h"

#include <map>
#include <thread>

namespace viewer {
namespace {

void apply_view_with_axes(const OrbitCamera &view, bool show_axes) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTransform(view.camera_from_anchor());
  if (show_axes) {
    draw_axes({SE3(), 0.1 / view.zoom(), 3.0, 5.0});
  }

  glTransform(view.anchor_from_world());
  if (show_axes) {
    draw_axes({SE3(), 0.2 / view.zoom(), 3.0});
  }
}

}  // namespace

Window3D::Window3D() {
  set_view_preset(ViewSetting::STANDARD);
}

void Window3D::init(const GlSize &gl_size) {
  // Initialize imgui
  gl_size_ = gl_size;
  imgui_mgr_.init(get_window());
}

void Window3D::set_view_preset(const ViewSetting &setting) {
  switch (setting) {
    case ViewSetting::STANDARD: {
      set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
                                Eigen::Vector3d::Zero()));
      break;
    }
    case ViewSetting::CAMERA: {
      set_target_from_world(
          SE3(SO3::exp(Eigen::Vector3d::Zero()), Eigen::Vector3d::Zero()));
      break;
    }
  }
}

void Window3D::spin_until_step() {
  const std::string title_at_start = title();
  if (!should_step_ && !should_continue_) {
    set_title(title_at_start + " (Waiting For Step)");
  }

  while (!should_step_ && !should_continue_ && !should_close()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (should_continue_ && !should_close()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(continue_ms_));
  }

  if (should_step_) {
    should_step_ = false;
  }
  set_title(title_at_start);
}

void Window3D::on_key(int key, int scancode, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  if (imgui_mgr_.want_capture()) {
    return;
  }

  // TODO: There is precision loss here, so the character is actually ambiguous.
  const char key_char = static_cast<char>(key);

  if (action == GLFW_PRESS) {
    //
    // Toggling
    //
    if (key_mappings_.count(key_char) != 0) {
      const std::string menu_name = key_mappings_.at(key_char);
      menus_[menu_name].value =
          (menus_[menu_name].value + 1) % (menus_[menu_name].n_states);
      const int new_toggle_state = menus_[menu_name].value;
      for (const auto &menu_callback : menu_callbacks_[menu_name]) {
        menu_callback(new_toggle_state);
      }
    }

    //
    // Navigation UI management
    //
    if (key == GLFW_KEY_N) {
      should_step_ = true;
    }

    if (key == GLFW_KEY_C) {
      should_continue_ = !should_continue_;
    }

    if (key == GLFW_KEY_O) {
      orthogonal_projection_ = !orthogonal_projection_;
    }

    if (key == GLFW_KEY_T) {
      view_.toggle_orbit();
    }

    if (key == GLFW_KEY_H) {
      hide_axes_ = !hide_axes_;
    }
  }
}
void Window3D::on_mouse_button(int button, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  if (imgui_mgr_.want_capture()) {
    return;
  }

  const auto current_mouse_pos = mouse_pos();
  // const auto ray = projection_.unproject(current_mouse_pos);

  mouse_pos_last_click_ = current_mouse_pos;
}

void Window3D::on_mouse_move(const WindowPoint &mouse_pos) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  if (imgui_mgr_.want_capture()) {
    return;
  }

  // TODO(jpanikulam): Fix this
  // const bool shift = held_keys().count(GLFW_KEY_LEFT_SHIFT) == 1
  //                        ? held_keys().at(GLFW_KEY_LEFT_SHIFT)
  //                        : false;
  const bool shift = false;

  const bool left = left_mouse_held() && !shift;
  const bool right = right_mouse_held() || (shift && left_mouse_held());

  // if (left || right) {
  // glfwSetInputMode(get_window(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  // } else {
  // glfwSetInputMode(get_window(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  // }

  view_.apply_mouse(mouse_pos, mouse_pos_last_click_, left, right);

  if (left || right) {
    mouse_pos_last_click_ = mouse_pos;
  }
}

void Window3D::on_scroll(const double amount) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  if (imgui_mgr_.want_capture()) {
    return;
  }
  view_.apply_scroll(amount);
}

void Window3D::resize(const GlSize &gl_size) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
}

void Window3D::draw_all_primitives() const {
  for (const auto &primitive : primitives_) {
    primitive->state_update();
    primitive->draw();
  }
}

void Window3D::draw() {
}

void Window3D::render() {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  if (need_flip_) {
    for (const auto &primitive : primitives_) {
      primitive->flip();
    }
    need_flip_ = false;
  }

  //
  // Render cameras
  //
  for (const auto &camera : cameras_) {
    prepare_to_render();
    camera->prepare_view();
    draw_all_primitives();
    camera->draw();
  }

  {  // Render main scene

    set_perspective(gl_size_, orthogonal_projection_);
    prepare_to_render();
    apply_view_with_axes(view_, !hide_axes_);
    projection_ = Projection::get_from_current();

    const auto ray = projection_.unproject(mouse_pos());
    callback_manager_.handle_callbacks(ray, projection_.to_viewport(mouse_pos()));

    draw_all_primitives();
  }

  imgui_mgr_.new_frame();
  draw();
  imgui_mgr_.render();

  const double t_now = glfwGetTime();

  const double dt = std::max(t_now - last_update_time_, 0.02);
  view_.apply_keys(held_keys(), dt);
  view_ = view_.simulate(dt);
  last_update_time_ = t_now;

  glFinish();
}  // namespace viewer

namespace {
struct Window3DGlobalState {
  std::map<std::string, std::shared_ptr<Window3D>> windows;
};

Window3DGlobalState window_3d_state;

Window3DGlobalState &get_global_state() {
  return window_3d_state;
}
}  // namespace

std::shared_ptr<Window3D> get_window3d(const std::string &title) {
  const auto it = get_global_state().windows.find(title);
  if (it != get_global_state().windows.end()) {
    return it->second;
  } else {
    const GlSize gl_size(640, 640);
    auto window = std::make_shared<Window3D>();
    get_global_state().windows[title] = window;
    WindowManager::register_window(gl_size, window, title);
    return window;
  }
}
}  // namespace viewer
