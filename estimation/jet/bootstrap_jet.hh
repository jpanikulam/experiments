#pragma once

#include "estimation/jet/jet_filter.hh"
#include "geometry/types/unit_vector.hh"
#include "geometry/kinematics/transform_network.hh"

namespace estimation {
namespace jet_filter {

struct BootstrapResult {
  JetFilter::JetFilterState xp0;
  Parameters parameters;
};

BootstrapResult bootstrap_jet(const geometry::Unit3& g_imu_frame,
                              const geometry::TransformNetwork& tfn,
                              const jcc::TimePoint& t0);

}  // namespace jet_filter
}  // namespace estimation