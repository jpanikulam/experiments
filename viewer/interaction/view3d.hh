#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "lanczos/rigid_body.hh"

namespace viewer {
namespace {
using Vec3 = Eigen::Vector3d;
}  // namespace

//
// A create that generates critical pose information, but importantly,
// *doesn't* tell openGL how to use it
//
class AnchoredCamera {
 public:
  using KeyMap = std::unordered_map<char, bool>;

  // Get the actual view
  SE3 camera_from_world() const;

  SE3 anchor_from_world() const;

  double zoom() const;

  // Advance the view simulation by some timestep
  void simulate(const double dt) const;

  void apply_keys_to_view(const KeyMap& held_keys, double dt);

 private:
  SE3 anchor_from_world_;

  // Default identity
  constexpr double INITIAL_Z_OFFSET_M = 15.0;
  SE3 camera_from_anchor_ = SE3(SO3(), Vec3(0.0, 0.0, INITIAL_Z_OFFSET_M));

  // Tangent vector; Left tangent space or gtfo
  Vec3 linear_velocity_ = Vec3::Zero();
  Vec3 angular_velocity_ = Vec3::Zero();

  double zoom_ = 0.001;
  double azimuth_ = 0.0;
  double elevation_ = 0.0;
  double dist_to_target_ = 1.0;
};

SE3 AnchoredCamera::camera_from_world() {
  return camera_from_anchor_ * anchor_from_world_;
}

SE3 AnchoredCamera::anchor_from_world() {
  return anchor_from_world_;
}

double AnchoredCamera::zoom() {
  return zoom_;
}

AnchoredCamera AnchoredCamera::simulate(const double dt) const {
  // Copy ourselves into the output camera
  AnchoredCamera out_camera = this;

  const VecNd<6> delta = jcc::vstack(velocity, angular_velocity) * dt;
  out_camera.anchor_from_world_ = SE3::exp(delta) * anchor_from_world_;

  constexpr double translation_damping = 0.9;
  constexpr double rotation_damping = 0.9;

  velocity *= translation_damping;
  angular_velocity *= rotation_damping;
}

}  // namespace viewer