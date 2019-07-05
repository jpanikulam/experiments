#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "geometry/shapes/fit_ellipse.hh"

#include <cmath>

namespace estimation {
namespace calibration {

jcc::Vec3 ImuModel::correct_measured_accel(const jcc::Vec3& raw_measurement) const {
  constexpr double G_MPSS = 9.81;
  return geometry::shapes::deform_ellipse_to_unit_sphere(raw_measurement,
                                                         intrinsics_.imu_gains) *
         G_MPSS;
}

jcc::Vec3 ImuModel::correct_measured_mag(const jcc::Vec3& raw_measurement) const {
  constexpr double G_MPSS = 9.81;
  return geometry::shapes::deform_ellipse_to_unit_sphere(raw_measurement,
                                                         intrinsics_.magnetometer_gains);
}

ImuModel estimate_imu_intrinsics(const ImuCalibrationMeasurements& imu_meas) {
  constexpr double G_MPSS = 9.81;
  constexpr double MAX_G_DEFECT_FRACTION = 0.1;

  JASSERT_FALSE(imu_meas.accel_meas.empty(), "Must have nonempty accel measurements");
  JASSERT_EQ(imu_meas.accel_meas.size(),
             imu_meas.mag_meas.size(),
             "Must have equal accel and magenetometer measurements");

  std::vector<jcc::Vec3> accel_to_fit;
  {
    accel_to_fit.reserve(imu_meas.accel_meas.size());
    for (const auto& meas : imu_meas.accel_meas) {
      //
      // Discard measurements that do not appear to be close to G
      // This is because the IMU will be moved during the measurement process
      //
      const double error_fraction =
          std::abs(meas.measurement.observed_accel.norm() - G_MPSS) / G_MPSS;
      if (error_fraction < MAX_G_DEFECT_FRACTION) {
        accel_to_fit.push_back(meas);
      }
    }
  }
  const auto accel_correction = geometry::shapes::fit_ellipse(accel_to_fit);

  std::vector<jcc::Vec3> mag_to_fit;
  {
    mag_to_fit.reserve(imu_meas.accel_meas.size());
    for (const auto& meas : imu_meas.accel_meas) {
      mag_to_fit.push_back(meas.measurement.observed_bfield);
    }
  }
  const auto mag_correction = geometry::shapes::fit_ellipse(mag_to_fit);

  const ImuIntrinsics intrinsics{.imu_gains = accel_correction.ellipse,
                                 .magnetometer_gains = mag_correction.ellipse};
  return ImuModel(intrinsics);
}

}  // namespace calibration
}  // namespace estimation