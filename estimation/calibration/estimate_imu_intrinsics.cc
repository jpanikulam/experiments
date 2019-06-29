#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "geometry/shapes/fit_ellipse.hh"

#include <cmath>

namespace estimation {
namespace calibration {

jcc::Vec3 ImuModel::apply_calibration(const jcc::Vec3& raw_measurement) const {
  constexpr double G_MPSS = 9.81;
  return geometry::shapes::deform_ellipse_to_unit_sphere(raw_measurement,
                                                         intrinsics_.imu_gains) *
         G_MPSS;
}

ImuModel estimate_imu_intrinsics(const std::vector<jcc::Vec3>& raw_measurements) {
  constexpr double G_MPSS = 9.81;
  constexpr double MAX_G_DEFECT_FRACTION = 0.1;

  std::vector<jcc::Vec3> to_fit;
  to_fit.reserve(raw_measurements.size());

  for (const auto& meas : raw_measurements) {
    //
    // Discard measurements that do not appear to be close to G
    // This is because the IMU will be moved during the measurement process
    //
    const double error_fraction = std::abs(meas.norm() - G_MPSS) / G_MPSS;
    if (error_fraction < MAX_G_DEFECT_FRACTION) {
      to_fit.push_back(meas);
    }
  }

  const auto result = geometry::shapes::fit_ellipse(to_fit);
  const ImuIntrinsics intrinsics{.imu_gains = result.ellipse};
  return ImuModel(intrinsics);
}

}  // namespace calibration
}  // namespace estimation