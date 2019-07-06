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

}  // namespace spatial
}  // namespace geometry