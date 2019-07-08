#pragma once

#include "geometry/types/unit_vector.hh"

namespace geometry {
namespace spatial {

// Form a coordinate frame that takes Z to provided zhat
// Returns z_from_zhat
SO3 form_coordinate_frame_from_zhat(const Unit3& zhat);

// Forms a coordinate frame that takes Z to provided zhat,
// and the x axis does its best to "point at" x_dir
SO3 form_coordinate_frame_from_zhat_and_x(const Unit3& zhat, const jcc::Vec3& x_dir);

}  // namespace spatial
}  // namespace geometry