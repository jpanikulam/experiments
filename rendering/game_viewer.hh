#pragma once

#include "viewer/simple_window.hh"

#include "viewer/interaction/imgui_manager.hh"
#include "viewer/interaction/view3d.hh"
#include "viewer/primitives/geometry_buffer.hh"
#include "viewer/projection.hh"

#include "eigen.hh"
#include "sophus.hh"
#include "util/time_point.hh"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

// TODO
#include "rendering/imgui_elements/game_debug_configuration.hh"
#include "rendering/assets/load_voxel_asset.hh"
#include "rendering/shaders/load_shader.hh"

namespace jcc {

struct GameViewerState {
  struct InteractionState {
    viewer::WindowPoint mouse_pos_last_click;
    bool paused = false;
  };

  struct ViewState {
    viewer::Projection projection;
    viewer::OrbitCamera camera;
  };

  struct SimulationState {
    double last_update_time = 0.0;
  };

  ViewState view;
  InteractionState interaction;
  SimulationState sim;
};

class GameViewer : public viewer::SimpleWindow {
 public:
  GameViewer();
  void init(const viewer::GlSize &gl_size) override;
  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const viewer::WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;
  void resize(const viewer::GlSize &gl_size) override;

  void render() override;

  void draw_scene();

  void go() const;

 private:
  GameViewerState gv_state_;
  viewer::ImGuiManager imgui_mgr_;

  Shader test_shader_;
  VoxelAsset test_asset_;

  viewer::GeometryBuffer demo_buffer_;

  GameDebugConfiguration ui_cfg_;
};

struct GameViewerConfig {
  std::string title = "Demo Game";
};

std::shared_ptr<GameViewer> create_gameviewer(const GameViewerConfig &cfg = {});

}  // namespace jcc