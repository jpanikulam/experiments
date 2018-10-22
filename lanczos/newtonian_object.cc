#include "lanczos/newtonian_object.hh"

namespace lanczos {

NewtonianObject simulate(const NewtonianObject& object,
                         const Vec3& world_force,
                         const Vec3& world_torque,
                         const double dt_sec) {
  //
  // Generate deltas
  //

  // No inverse, baby!
  // (These solves could feasibly be a bottleneck)
  const Vec3 dv_dt = force / object.mass_kg;
  const Vec3 dw_dt = object.inertia_pkg.fullPivLu().solve(torque);

  //
  // Integrate
  //

  NewtonianObject new_object = object;

  new_object.body.vel += (dv_dt * dt_sec);
  new_object.body.ang_vel += (dw_dt * dt_sec);

  // new_object.
  // body_from_world
  const VecNd<6> lg_world2_from_world =
      jcc::vstack(new_object.body.vel, new_object.body.ang_vel) * dt_sec;
  const SE3 world2_from_world = SE3::exp(log_delta);

  //
  // Simulate (w/ orientation)
  //

  return new_object;
}
}  // namespace lanczos