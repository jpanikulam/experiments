#include "estimation/sensors/make_interpolator.hh"
namespace estimation {

geometry::spatial::TimeInterpolator make_accel_interpolator(
    const std::vector<calibration::TimedMeasurement<jet_filter::AccelMeasurement>>& accel_meas,
    const calibration::ImuModel& imu_model) {
  std::vector<geometry::spatial::TimeControlPoint> points;

  for (const auto& accel : accel_meas) {
    const jcc::Vec3 corrected_accel =
        imu_model.apply_calibration(accel.measurement.observed_acceleration);
    points.push_back({accel.timestamp, corrected_accel});
  }
  const geometry::spatial::TimeInterpolator interp(points);
  return interp;
}

}  // namespace estimation