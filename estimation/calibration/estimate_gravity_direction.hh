#pragma once

#include "estimation/calibration/calibration_dataset.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "geometry/types/unit_vector.hh"

namespace estimation {
namespace calibration {

struct EstimationConfig {
  double max_speed_mps = 0.005;
  double max_fiducial_dt_sec = 0.2;
  double min_stationary_time = 5.0;
  double max_g_defect = 0.1;
};

geometry::UnitVector3 estimate_gravity_direction(
    const CalibrationMeasurements& measurements,
    const ImuModel& imu_model,
    const EstimationConfig& cfg);

}  // namespace calibration
}  // namespace estimation