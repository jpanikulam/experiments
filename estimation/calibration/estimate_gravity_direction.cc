#include "estimation/calibration/estimate_gravity_direction.hh"

#include "estimation/sensors/make_interpolator.hh"

#include "numerics/compute_convenient_statistics.hh"
#include "numerics/compute_mean.hh"

#include <limits>

namespace estimation {

namespace {

jcc::Vec3 compute_biggest_difference(const std::vector<jcc::Vec3>& vecs) {
  double biggest_cross = 0.0;
  jcc::Vec3 best;

  for (const auto& v1 : vecs) {
    for (const auto& v2 : vecs) {
      const double cross = v1.cross(v2).norm();

      if (cross > biggest_cross) {
        best = v2.cross(v1);
        biggest_cross = cross;
      }
    }
  }
  return best;
}

struct Bracket {
  TimePoint start;
  TimePoint end;
};

geometry::UnitVector3 estimate_from_bracket(
    const ImuCalibrationMeasurements& measurements,
    const ImuModel& imu_model,
    const EstimationConfig& cfg,
    const Bracket& bracket) {
  constexpr double G_MPSS = 9.81;

  JASSERT_GT(
      estimation::to_seconds(bracket.end - bracket.start),
      cfg.min_stationary_time_sec,
      "Could not find a time bracket where the vehicle is stationary and pointed at "
      "the fiducial for sufficiently long");

  std::vector<double> bracket_norm_accel;
  std::vector<jcc::Vec3> bracket_accel;

  for (const auto& accel : measurements.accel_meas) {
    if (accel.timestamp >= bracket.start && accel.timestamp <= bracket.end) {
      const jcc::Vec3 corrected_accel =
          imu_model.correct_measured_accel(accel.measurement.observed_acceleration);

      if (std::abs(corrected_accel.norm() - G_MPSS) > cfg.max_g_defect) {
        continue;
      }

      bracket_norm_accel.push_back(corrected_accel.norm());
      bracket_accel.push_back(corrected_accel);
    }
  }

  JASSERT_GT(bracket_accel.size(),
             100u,
             "Should have gotten at least 80 imu measurements in the window");

  // These are not independent samples, but let's just work with it
  const auto stats = numerics::compute_convenient_statistics(bracket_norm_accel);
  JASSERT_LT(stats.max - stats.min,
             cfg.max_accel_range_mpss,
             "Accelerometer measurement range must be small");

  JASSERT_LE(std::abs(stats.mean - G_MPSS),
             cfg.max_g_defect,
             "Mean was too far from g to be comfortable!");

  const jcc::Vec3 biggest_error = compute_biggest_difference(bracket_accel);
  const double max_angle_error = std::asin(biggest_error.norm());
  JASSERT_LT(
      max_angle_error, 0.01, "Accelerometer measurement angle range must be small");

  const jcc::Vec3 average = numerics::compute_mean(bracket_accel);

  return geometry::UnitVector3(average);
}

}  // namespace

GravityEstimationResult estimate_gravity_direction(
    const CalibrationMeasurements& measurements,
    const ImuCalibrationMeasurements& imu_measurements,
    const ImuModel& imu_model,
    const EstimationConfig& cfg) {
  const auto accel_interp =
      make_accel_interpolator(imu_measurements.accel_meas, imu_model);
  JASSERT_GT(imu_measurements.accel_meas.size(),
             1000u,
             "Should have at least 1000 accel measurements");

  const auto first = measurements.fiducial_meas.front().timestamp;

  bool in_bracket = false;
  TimePoint bracket_start = first;
  TimePoint bracket_end = first;

  for (std::size_t i = 1u; i < measurements.fiducial_meas.size(); ++i) {
    const auto prev = measurements.fiducial_meas.at(i - 1);
    const auto cur = measurements.fiducial_meas.at(i);

    const auto fiducial_from_camera_0 = prev.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_1 = cur.measurement.T_fiducial_from_camera;

    const auto t0 = prev.timestamp;
    const auto t1 = cur.timestamp;

    const SE3 camera_1_from_camera_0 =
        fiducial_from_camera_1.inverse() * fiducial_from_camera_0;

    const double dt = to_seconds(t1 - t0);
    const jcc::Vec3 dx_dt = camera_1_from_camera_0.translation() / dt;
    const auto t = average(t0, t1);

    const bool accel_available = static_cast<bool>(accel_interp(t));
    const bool dt_small_enough = dt < cfg.max_fiducial_dt_sec;
    const bool estimated_speed_small_enough = dx_dt.norm() < cfg.max_speed_mps;
    const bool valid = dt_small_enough &&               //
                       estimated_speed_small_enough &&  //
                       accel_available;

    if (valid) {
      if (in_bracket) {
        bracket_end = t;
      } else {
        bracket_start = t;
        bracket_end = t;
        in_bracket = true;
      }
    } else {
      in_bracket = false;
    }

    if (valid && in_bracket &&
        (estimation::to_seconds(bracket_end - bracket_start) >
         cfg.min_stationary_time_sec)) {
      break;
    }
  }

  JASSERT(in_bracket, "Did not end inside a bracket, result is invalid.");

  const Bracket bracket{.start = bracket_start, .end = bracket_end};

  const auto direction = estimate_from_bracket(imu_measurements, imu_model, cfg, bracket);

  return GravityEstimationResult{
      .direction = direction,
      .time = (bracket.start +
               estimation::to_duration(
                   estimation::to_seconds(bracket.end - bracket.start) * 0.5))};
}

}  // namespace estimation