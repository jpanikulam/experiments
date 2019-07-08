#include "estimation/sensors/make_interpolator.hh"
namespace estimation {

geometry::spatial::TimeInterpolator make_accel_interpolator(
    const std::vector<TimedMeasurement<jet_filter::AccelMeasurement>>&
        accel_meas,
    const ImuModel& imu_model) {
  std::vector<geometry::spatial::TimeControlPoint> points;

  for (const auto& accel : accel_meas) {
    const jcc::Vec3 corrected_accel =
        imu_model.correct_measured_accel(accel.measurement.observed_acceleration);
    points.push_back({accel.timestamp, corrected_accel});
  }
  const geometry::spatial::TimeInterpolator interp(points);
  return interp;
}

geometry::spatial::TimeInterpolator make_gyro_interpolator(
    const std::vector<TimedMeasurement<jet_filter::GyroMeasurement>>&
        gyro_meas) {
  std::vector<geometry::spatial::TimeControlPoint> points;

  for (const auto& gyro : gyro_meas) {
    points.push_back({gyro.timestamp, gyro.measurement.observed_w});
  }
  const geometry::spatial::TimeInterpolator interp(points);
  return interp;
}

}  // namespace estimation