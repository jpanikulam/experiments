#pragma once

#include "viewer/simple_window.hh"

#include "viewer/gl_size.hh"
#include "viewer/interaction/callback_manager.hh"
#include "viewer/interaction/imgui_manager.hh"
#include "viewer/interaction/view3d.hh"
#include "viewer/primitives/camera.hh"
#include "viewer/primitives/primitive.hh"
#include "viewer/projection.hh"

#include "sophus.hh"

#include "eigen.hh"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace viewer {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec2Map = Eigen::Map<const Eigen::Vector2d>;
using Vec3Map = Eigen::Map<const Eigen::Vector3d>;
}  // namespace

class Window3D : public SimpleWindow {
 public:
  Window3D();
  virtual ~Window3D() = default;
  void init(const GlSize &gl_size) override;

  ///////////////////////////////////
  // Window3D Configuration
  //////////////////////////////////

  virtual void on_key(int key, int scancode, int action, int mods) override;
  virtual void on_mouse_button(int button, int action, int mods) override;
  virtual void on_mouse_move(const WindowPoint &mouse_pos) override;
  virtual void on_scroll(const double amount) override;
  virtual void resize(const GlSize &gl_size) override;

  void render() override;

  virtual void draw();

  ///////////////////////////////////
  // Drawing
  //////////////////////////////////

  void add_primitive(const std::shared_ptr<Primitive> primitive) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.push_back(std::move(primitive));
  }

  template <typename PrimitiveType, typename... Args>
  std::shared_ptr<PrimitiveType> add_primitive(const Args &... args) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);

    auto ptr = std::make_shared<PrimitiveType>(args...);
    primitives_.push_back(ptr);
    return ptr;
  }

  void add_camera(const std::shared_ptr<Camera> &camera) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    cameras_.push_back(camera);
  }

  void clear() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.clear();
  }

  void flip() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    need_flip_ = true;
  }

  ///////////////////////////////////
  // GUI
  ///////////////////////////////////

  void spin_until_step();
  void trigger_continue() {
    should_continue_ = true;
  }

  //
  // Toggle UI
  //

  void add_toggle_hotkey(const std::string &toggle_name, bool default_state, char key) {
    add_menu_hotkey(toggle_name, default_state, key, 2);
  }

  void add_menu_hotkey(const std::string &toggle_name,
                       int default_state,
                       char key,
                       int n_states) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    menus_[toggle_name] = {n_states, default_state};
    key_mappings_[key] = toggle_name;
  }

  bool get_toggle(const std::string &state) const {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    return menus_.at(state).value;
  }

  int get_menu(const std::string &state) const {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    return menus_.at(state).value;
  }

  void clear_toggle_callbacks(const std::string &toggle_name) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    menu_callbacks_[toggle_name].clear();
  }

  using MenuCallback = std::function<void(int)>;
  void add_toggle_callback(const std::string &toggle_name, const MenuCallback &callback) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    menu_callbacks_[toggle_name].push_back(callback);
  }

  void run_menu_callbacks() {
    for (const auto &cb : menu_callbacks_) {
      for (const auto &fnc : cb.second) {
        fnc(menus_[cb.first].value);
      }
    }
  }

  //
  // Click Callbacks UI
  //

  void add_click_callback(
      const CallbackManager::ClickCallback &callback,
      const std::vector<geometry::shapes::LineSegment> &line_segments) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    callback_manager_.register_click_callback(callback, line_segments);
  }

  void add_click_callback(const CallbackManager::ClickCallback &callback,
                          const geometry::Plane &plane) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    callback_manager_.register_click_callback(callback, plane);
  }

  void clear_click_callbacks() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    callback_manager_.clear_callbacks();
  }

  ///////////////////////////////////
  // View
  ///////////////////////////////////

  //
  // Gross View Configuration
  //

  void set_continue_time_ms(const int dt_ms) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    continue_ms_ = dt_ms;
  }

  //
  // View Camera Configuration
  //

  void set_target_from_world(const SE3 &se3) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    view_.set_anchor_from_world(se3);
  }

  // Get the Z-Forward, Y-Down camera_from_world
  // This is the OpenCV / General CV convention
  SE3 standard_camera_from_world() const {
    return view_.standard_camera_from_world();
  }

  const Projection &projection() const {
    return projection_;
  }

  // Z-Backward, Y-Up
  SE3 camera_from_world() const {
    return view_.camera_from_world();
  }

  void set_azimuth(const double azimuth) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    view_.set_azimuth(azimuth);
  }
  void set_elevation(const double elevation) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    view_.set_elevation(elevation);
  }
  void set_zoom(const double zoom) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    view_.set_zoom(zoom);
  }

  // How should the Z-Axis be set?
  enum class ViewSetting {
    STANDARD = 0,
    CAMERA = 1,
  };

  void set_view_preset(const ViewSetting &);

  // Switch to orthographic projection
  void set_orthogonal(const bool ortho = true) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    orthogonal_projection_ = ortho;
  }

  //
  // View Minutia Configuration
  //

  void set_show_axes(const bool show_axes = true) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    hide_axes_ = !show_axes;
  }

  const GlSize &size() const {
    return gl_size_;
  }

 private:
  void draw_all_primitives() const;

  //
  // Rendering properties
  //

  Projection projection_;
  bool orthogonal_projection_ = false;
  std::vector<std::shared_ptr<Primitive>> primitives_;
  std::vector<std::shared_ptr<Camera>> cameras_;

  GlSize gl_size_;

  //
  // UI Control State
  //

  WindowPoint mouse_pos_last_click_;
  OrbitCamera view_;
  int continue_ms_ = 20;
  bool hide_axes_ = false;

  struct MenuSettings {
    int n_states;
    int value;
  };
  std::map<std::string, MenuSettings> menus_;
  std::map<std::string, std::vector<MenuCallback>> menu_callbacks_;
  std::map<char, std::string> key_mappings_;

  // Click Callbacks
  CallbackManager callback_manager_;

  ImGuiManager imgui_mgr_;

  // Queued UI flip
  bool need_flip_ = false;

  //
  // Interaction History
  //

  double last_update_time_ = 0.0;

  //
  // Threading
  //
  std::atomic<bool> should_step_{false};
  std::atomic<bool> should_continue_{false};
  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<Window3D> get_window3d(const std::string &title = "main");
}  // namespace viewer
