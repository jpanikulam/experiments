#pragma once

#include "viewer/window_3d.hh"

#include "planning/simulation/sim_viewer_ui.hh"
#include "util/time_point.hh"
#include "viewer/primitives/geometry_buffer.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "planning/simulation/interactable_geometry.hh"
#include "planning/simulation/sim_viewer_asset.hh"
#include "planning/simulation/sim_viewer_command_queue.hh"
#include "planning/simulation/sim_viewer_types.hh"
// TODO: Add pausing
#include "planning/simulation/sim_viewer_integrator.hh"

// todo
#include "geometry/spatial/raycaster.hh"

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
  void draw_element(int id, const Element &element);
  void draw_robot(const planning::drifter::State &state);

  // @returns true if it's time for a view update
  bool update_editor_state();

 private:
  MainMenuState main_menu_state_;
  EditorState editor_state_;
  InteractableGeometry interactable_geo_;
  SimulationIntegrator integrator_;

  geometry::spatial::RayCaster ray_caster_;
  bool show_bvh_ = false;

  viewer::GeometryBuffer geo_;
  viewer::GeometryBuffer tmp_geo_;

  viewer::GeometryBuffer plan_geo_;
  viewer::GeometryBuffer robot_geo_;

  viewer::GeometryBuffer bgnd_geo_;


  viewer::GeometryBuffer lidar_geo_;

  // SceneTree robot_tree_;

  std::map<std::string, Asset> assets_;

  std::stack<EditorCommand> editor_commands_;

  bool had_robots_ = false;
  bool view_state_change_ = false;

  jcc::TimePoint last_step_time_ = jcc::TimePoint::min();

  //
  // Between frames
  //

  TaskPopupState task_popup_state_;

  CommandQueue cmd_queue_;
};

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title);

}  // namespace simulation
}  // namespace jcc