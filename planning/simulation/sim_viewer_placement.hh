#pragma once

#include "eigen.hh"

namespace jcc {
namespace simulation {

inline jcc::Vec3 snap_to_grid(const jcc::Vec3& point) {
  const jcc::Vec3 placement_minimum(-1000.0, -1000.0, 0.0);
  return point.array().floor().array().max(placement_minimum.array());
}
}  // namespace simulation
}  // namespace jcc