#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "lanczos/rigid_body.hh"
#include "viewer/gl_types.hh"

#include <unordered_map>

namespace viewer {
namespace {
using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;
}  // namespace

//
// A camera that generates critical pose information
// *doesn't* tell openGL how to use it
// That's up to you, amigo
class OrbitCamera {
 public:
  using KeyMap = std::unordered_map<int, bool>;

  OrbitCamera() = default;
  OrbitCamera(const double azimuth,
              const double elevation,
              const double zoom = 0.1,
              const double dist_to_target = 1.0)
      : azimuth_(azimuth),
        elevation_(elevation),
        zoom_(zoom),
        dist_to_target_(dist_to_target) {
  }

  void set_azimuth(const double az) {
    azimuth_ = az;
  }
  void set_elevation(const double el) {
    elevation_ = el;
  }
  void set_zoom(const double zoom) {
    zoom_ = zoom;
  }
  // Get the actual view
  SE3 camera_from_world() const;

  SE3 anchor_from_world() const;

  // This is a convenience function for computer vision applications
  // The OpenGL rendering convention is Z-Backward, Y-Up: Points with negative Z get rendered
  // The standard CV convention is that cameras image Z-Forward, Y-Down
  SE3 standard_camera_from_world() const;

  SE3 camera_from_anchor() const;

  void set_anchor_from_world(const SE3& anchor_from_world) {
    anchor_body_.from_world = anchor_from_world;
  }

  double zoom() const;

  // Advance the view simulation by some timestep
  OrbitCamera simulate(const double dt_sec) const;

  void apply_keys(const KeyMap& held_keys, double dt_sec);

  void apply_mouse(const WindowPoint& mouse_pos,
                   const WindowPoint& mouse_prev,
                   const bool left_mouse_held,
                   const bool right_mouse_held);

  void apply_scroll(const double amount);

  void toggle_orbit() {
    orbit_ = !orbit_;
  }

 private:
  lanczos::RigidBody anchor_body_;

  bool orbit_ = false;

  double azimuth_ = M_PI * 0.25;
  double elevation_ = M_PI * 0.25;
  double zoom_ = 1.0;
  double dist_to_target_ = 1.0;
};

}  // namespace viewer