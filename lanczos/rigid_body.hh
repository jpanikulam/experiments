#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace lanczos {

namespace {
using Vec3 = Eigen::Vector3d;
}  // namespace
struct RigidBody {
  // NOTE:
  // These properties are specified in the world frame
  Vec3 positional_velocity = Vec3::Zero();
  Vec3 angular_velocity = Vec3::Zero();
  SE3 body_from_world;
};

struct RigidBodySimulationConfig {
  double positional_damping_per_sec = 0.0;
  double angular_damping_per_sec = 0.0;
};

RigidBody body_frame(const SE3& body_from_world,
                     const Vec3& body_frame_velocity,
                     const Vec3& body_frame_angular_velocity);

RigidBody simulate(const RigidBody& in_body,
                   const RigidBodySimulationConfig& cfg,
                   const double dt);

}  // namespace lanczos