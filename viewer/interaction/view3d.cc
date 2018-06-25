#include "viewer/interaction/view3d.hh"

namespace viewer {

//void View3D::get_view() {
//  const Eigen::AngleAxisd az_rot(azimuth, Vec3::UnitY());
//  const Eigen::AngleAxisd elev_rot(elevation, Vec3::UnitX());
//  const Eigen::Quaterniond q(elev_rot * az_rot);
//  const SE3 instantaneous_rotation(SO3(q), Vec3::Zero());
//
//  glTransform(camera_from_target.inverse() * instantaneous_rotation);
//  glScaled(zoom, zoom, zoom);
//
//  draw_axes({SE3(), 0.5});
//  // glTransform(target_from_world * SE3(SO3::exp(Vec3(0.0, 0.0, 3.1415)),
//  // Vec3::Zero()));
//  glTransform(target_from_world);
//  draw_axes({SE3(), 1.5});
//  simulate();
//}

void View3D::simulate(const double dt) {
  const VecNd<6> delta = jcc::vstack(velocity, angular_velocity) * dt;
  target_from_world = SE3::exp(delta) * target_from_world;

  constexpr double translation_damping = 0.9;
  constexpr double rotation_damping = 0.9;

  velocity *= translation_damping;
  angular_velocity *= rotation_damping;
}
}  // namespace viewer