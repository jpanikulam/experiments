#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "planning/simulation/sim_viewer.hh"

#include "geometry/plane.hh"
#include "viewer/window_manager.hh"

#include "third_party/imgui/imgui.h"

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

void SimViewer::update_editor_state() {
  //
  // Plane intersection
  //
  const geometry::Plane plane{Vec3::Zero(), geometry::Unit3::UnitZ()};

  const auto &proj = projection();
  const auto ray = proj.unproject(mouse_pos());

  jcc::Vec3 intersection;
  const bool intersected = plane.intersect(ray, out(intersection));
  if (intersected) {
    editor_state_.world_clicked = true;
    editor_state_.world_click_pos = intersection;
  } else {
    editor_state_.world_clicked = false;
  }
}

void SimViewer::draw() {
  create_main_menu(out(main_menu_state_));

  //
  // Draw things
  //

  // for (const auto &element : editor_state_.elements) {
  //   geo_primary_->add_axes({element.second.world_from_object});
  // }
  // geo_primary_->flip();

  TaskPopupState popup_state{};
  const auto maybe_command = create_task_popup(editor_state_, out(popup_state));
  if (maybe_command) {
    cmd_queue_.commit(*maybe_command, out(editor_state_));
  }
}

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title) {
  const viewer::GlSize gl_size(640, 640);
  auto window = std::make_shared<SimViewer>();
  viewer::WindowManager::register_window(gl_size, window, title);
  return window;
}

}  // namespace simulation
}  // namespace jcc