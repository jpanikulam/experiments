#include "lanczos/newtonian_object.hh"

namespace lanczos {

NewtonianObject simulate(const NewtonianObject& object,
                         const Vec3& force,
                         const Vec3& torque,
                         const double dt) {
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

  new_object.body.angular_velocity += (dw_dt * dt);
  new_object.body.positional_velocity += (dv_dt * dt);

  //
  // Simulate (w/ orientation)
  //

  new_object.body = simulate(new_object.body, object.rigid_body_cfg, dt);

  return new_object;
}
}  // namespace lanczos