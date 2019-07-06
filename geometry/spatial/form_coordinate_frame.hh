#pragma once

#include "geometry/types/unit_vector.hh"

namespace geometry {
namespace spatial {

SO3 form_coordinate_frame_from_zhat(const Unit3& zhat);

}  // namespace spatial
}  // namespace geometry