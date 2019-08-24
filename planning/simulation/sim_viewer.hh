#pragma once

#include "viewer/window_3d.hh"

#include "planning/simulation/sim_viewer_ui.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "planning/simulation/sim_viewer_command_queue.hh"
#include "planning/simulation/sim_viewer_types.hh"

namespace jcc {
namespace simulation {

class SimViewer : public viewer::Window3D {
 public:
  SimViewer();
  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const viewer::WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void draw() override;
  void update_editor_state();

 private:
  std::shared_ptr<viewer::SimpleGeometry> geo_primary_;
  MainMenuState main_menu_state_;
  EditorState editor_state_;

  //
  // Between frames
  //
  TaskPopupState task_popup_state_;

  CommandQueue cmd_queue_;
};

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title);

}  // namespace simulation
}  // namespace jcc