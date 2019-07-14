// inc order weird for these guys
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//%deps(opengl, glfw)

#include "viewer/metaviewer.hh"

#include "viewer/gl_aliases.hh"
#include "viewer/primitives/simple_geometry_primitives.hh"
#include "viewer/window_manager.hh"

#include "eigen_helpers.hh"

#include <map>

namespace viewer {

void MetaViewer::on_key(int key, int scancode, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  for (const auto &partition : partitions_) {
    partition->key_pressed(key, scancode, action, mods);
  }
}
void MetaViewer::on_mouse_button(int button, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  for (const auto &partition : partitions_) {
    partition->mouse_button(button, action, mods);
  }
}

void MetaViewer::on_mouse_move(const WindowPoint &mouse_pos) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  for (const auto &partition : partitions_) {
    partition->mouse_moved(mouse_pos.point.x(), mouse_pos.point.y());
  }
}

void MetaViewer::on_scroll(const double amount) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  for (const auto &partition : partitions_) {
    partition->on_scroll(amount);
  }
}

void MetaViewer::resize(const GlSize &gl_size) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
}

void MetaViewer::render() {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  {  // Render main scene
    // TODO: Set viewport

    for (const auto &partition : partitions_) {
      partition->render();
    }
  }

  const double t_now = glfwGetTime();

  const double dt = std::max(t_now - last_update_time_, 0.02);
  (void)dt;  // I expect this to be necessary -- todo(jpanikulam)
  last_update_time_ = t_now;

}  // namespace viewer

namespace {

struct MetaViewerGlobalState {
  std::map<std::string, std::shared_ptr<MetaViewer>> windows;
};

MetaViewerGlobalState window_3d_state;

MetaViewerGlobalState &get_global_state() {
  return window_3d_state;
}
}  // namespace

std::shared_ptr<MetaViewer> get_metaviewer(const std::string &title) {
  const auto it = get_global_state().windows.find(title);
  if (it != get_global_state().windows.end()) {
    return it->second;
  } else {
    const GlSize gl_size(640, 640);
    auto window = std::make_shared<MetaViewer>(gl_size);
    get_global_state().windows[title] = window;
    WindowManager::register_window(gl_size, window, title);
    return window;
  }
}
}  // namespace viewer
