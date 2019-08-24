#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "planning/simulation/sim_viewer.hh"

#include "third_party/imgui/imgui.h"

#include "viewer/window_manager.hh"

namespace jcc {
namespace simulation {

SimViewer::SimViewer() {
  const auto background = this->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void SimViewer::on_key(int key, int scancode, int action, int mods) {
  Window3D::on_key(key, scancode, action, mods);
}
void SimViewer::on_mouse_button(int button, int action, int mods) {
  Window3D::on_mouse_button(button, action, mods);
}
void SimViewer::on_mouse_move(const viewer::WindowPoint &mouse_pos) {
  Window3D::on_mouse_move(mouse_pos);
}
void SimViewer::on_scroll(const double amount) {
  Window3D::on_scroll(amount);
}

void SimViewer::draw() {
  create_main_menu(main_menu_state_);
}

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title) {
  const viewer::GlSize gl_size(640, 640);
  auto window = std::make_shared<SimViewer>();
  viewer::WindowManager::register_window(gl_size, window, title);
  return window;
}

}  // namespace simulation
}  // namespace jcc