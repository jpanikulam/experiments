#pragma once

#include "estimation/calibration/calibration_dataset.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "geometry/types/unit_vector.hh"

namespace estimation {

// One should typically expect to use the defaults.
struct EstimationConfig {
  double max_speed_mps = 0.1;
  double max_fiducial_dt_sec = 0.2;
  double min_stationary_time_sec = 1.0;
  double max_g_defect = 0.02;
  // Largest permissible difference between max and min accel
  // in the range where the vehicle appears to be stationary
  double max_accel_range_mpss = 0.05;
};

struct GravityEstimationResult {
  geometry::UnitVector3 direction;
  TimePoint time;
  // TODO: (Instead of asserting)
  // std::vector<std::string> warning
  // bool success;
};

GravityEstimationResult estimate_gravity_direction(
    const CalibrationMeasurements& measurements,
    const ImuCalibrationMeasurements& imu_measurements,
    const ImuModel& imu_model,
    const EstimationConfig& cfg = {});

}  // namespace estimation