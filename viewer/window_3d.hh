#pragma once

#include "viewer/gl_size.hh"
#include "viewer/projection.hh"
#include "viewer/simple_window.hh"

#include "sophus.hh"
#include "viewer/primitives/primitive.hh"

#include <Eigen/Dense>

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace gl_viewer {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec2Map = Eigen::Map<const Eigen::Vector2d>;
using Vec3Map = Eigen::Map<const Eigen::Vector3d>;
}  // namespace

class View3D {
 public:
  SE3 target_from_world;

  // Default identity
  SE3 camera_from_target = SE3(SO3(), Vec3(0.0, 0.0, 15.0));

  // Time
  double last_update_time = 0.0;

  double zoom = 0.001;

  // Tangent vector; Left tangent space or gtfo
  Vec3 velocity;
  Vec3 angular_velocity;

  double azimuth = 0.0;
  double elevation = 0.0;
  double dist_to_target = 1.0;

  // Apply the transformation
  void apply();

  void simulate();
};

class Window3D final : public SimpleWindow {
 public:
  Window3D() = default;

  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void render() override;

  void resize(const GlSize &gl_size) override;

  void spin_until_step();

  void add_primitive(const std::shared_ptr<Primitive> primitive) {
    primitives_.push_back(std::move(primitive));
  }

  void set_target_from_world(const SE3 &se3) {
    view_.target_from_world = se3;
  }

  template <typename PrimitiveType, typename... Args>
  std::shared_ptr<PrimitiveType> add_primitive(const Args &... args) {
    auto ptr = std::make_shared<PrimitiveType>(args...);
    primitives_.push_back(ptr);
    return ptr;
  }

  void clear() {
    primitives_.clear();
  }

 private:
  void apply_keys_to_view();

  Vec2 mouse_direction_ = Vec2::Zero();

  //
  // Track some window properties
  //

  View3D view_;
  Projection projection_;
  std::atomic<bool> should_step_{false};
  std::atomic<bool> should_continue_{false};

  std::vector<std::shared_ptr<Primitive>> primitives_;

  GlSize gl_size_ = GlSize(640, 640);

  WindowPoint mouse_pos_last_click_;

  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<Window3D> get_window3d(const std::string &title = "main");
}  // namespace gl_viewer
