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

  if (action == GLFW_PRESS) {
    if ((key == GLFW_KEY_Z) && (mods == GLFW_MOD_CONTROL)) {
      cmd_queue_.undo(out(editor_state_));

    } else if ((key == GLFW_KEY_Y) && (mods == GLFW_MOD_CONTROL)) {
      cmd_queue_.redo(out(editor_state_));
    }
  }
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

  //
  // Main Menu UI Availability
  //
  editor_state_.can_undo = cmd_queue_.can_undo();
  editor_state_.can_redo = cmd_queue_.can_redo();
}

namespace {

void draw_element(const Element &element) {
  const jcc::Vec3 size = element.properties["size"];

  buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                           Vec3(box.lower.x(), box.upper.y(), box.upper.z()), box.color});
  buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                           Vec3(box.lower.x(), box.lower.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.upper.z()),
                           Vec3(box.upper.x(), box.lower.y(), box.upper.z()), box.color});
  buffer_.lines.push_back({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
                           Vec3(box.upper.x(), box.upper.y(), box.upper.z()), box.color});
  buffer_.lines.push_back({Vec3(box.lower.x(), box.upper.y(), box.upper.z()),
                           Vec3(box.lower.x(), box.upper.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
                           Vec3(box.upper.x(), box.upper.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.upper.z()),
                           Vec3(box.upper.x(), box.lower.y(), box.upper.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
                           Vec3(box.upper.x(), box.lower.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.upper.y(), box.lower.z()),
                           Vec3(box.lower.x(), box.upper.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
                           Vec3(box.lower.x(), box.lower.y(), box.lower.z()), box.color});
  buffer_.lines.push_back({Vec3(box.upper.x(), box.lower.y(), box.lower.z()),
                           Vec3(box.upper.x(), box.lower.y(), box.upper.z()), box.color});
  buffer_.lines.push_back({Vec3(box.lower.x(), box.lower.y(), box.lower.z()),
                           Vec3(box.lower.x(), box.upper.y(), box.lower.z()), box.color});
}
}  // namespace

void SimViewer::draw() {
  update_editor_state();

  create_main_menu(editor_state_, out(main_menu_state_));

  if (main_menu_state_.cmd_queue_update == CommandQueueAction::Undo) {
    cmd_queue_.undo(out(editor_state_));
  } else if (main_menu_state_.cmd_queue_update == CommandQueueAction::Redo) {
    cmd_queue_.redo(out(editor_state_));
  }

  //
  // Draw things
  //

  for (const auto &element : editor_state_.elements) {
    // viewer::draw_axes({element.second.world_from_object});
    draw_element(element);
  }

  const auto maybe_command = create_task_popup(editor_state_, out(task_popup_state_));
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