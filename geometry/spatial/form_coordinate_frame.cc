#include "geometry/spatial/form_coordinate_frame.hh"
#include "geometry/rotation_to.hh"

#include "logging/assert.hh"

namespace geometry {
namespace spatial {

SO3 form_coordinate_frame_from_zhat(const Unit3& zhat) {
  // const jcc::Vec3 xhat = perp(zhat);
  // const jcc::Vec3 yhat = zhat.vec().cross(xhat);

  const SO3 z_from_zhat = rotation_to(zhat.vec(), jcc::Vec3::UnitZ());
  return z_from_zhat;
}

SO3 form_coordinate_frame_from_zhat_and_x(const Unit3& zhat, const jcc::Vec3& x_dir) {
  const Unit3 xhat = Unit3(x_dir - zhat.project(x_dir));
  const Unit3 yhat = zhat.unit_cross(xhat);

  Eigen::Matrix3d mat;
  mat.col(0) = xhat.vec();
  mat.col(1) = yhat.vec();
  mat.col(2) = zhat.vec();

  const Eigen::Quaterniond q_world_from_frame(mat);
  return SO3(q_world_from_frame);
}

}  // namespace spatial
}  // namespace geometry