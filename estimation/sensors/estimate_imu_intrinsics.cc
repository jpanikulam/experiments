#include "estimation/sensors/estimate_imu_intrinsics.hh"

#include <cmath>

namespace estimation {
namespace sensors {

ImuIntrinsics estimate_imu_intrinsics(const std::vector<jcc::Vec3>& raw_measurements) {
  constexpr double G_MPSS = 9.81;

  // 10 percent
  constexpr double MAX_G_DEFECT_FRACTION = 0.1;

  std::vector<jcc::Vec3> to_fit;
  to_fit.reserve(raw_measurements.size());

  for (const auto& meas : raw_measurements) {
    const double error_fraction = std::abs(meas.norm() - G_MPSS) / G_MPSS;

    if (error_fraction > MAX_G_DEFECT_FRACTION) {
      to_fit.push_back(meas);
    }
  }

  const auto result = geometry::shapes::fit_ellipse(to_fit);
  const ImuIntrinsics intrinsics{.imu_gains = result.ellipse};
  return intrinsics;
}

}  // namespace sensors
}  // namespace estimation