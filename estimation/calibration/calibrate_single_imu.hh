#pragma once

#include "estimation/calibration/calibration_dataset.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/calibration/estimate_gravity_direction.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {

struct SingleImuCalibration {
  ImuModel imu_model;
  GravityEstimationResult g_estimate;
  SO3 camera_from_gyro;
  int imu_id;
};

struct CreateSingleImuModelConfig {
  bool visualize_imu_model = true;
  bool visualize_gyro = false;
  bool visualize_magnetometer = false;
};

SingleImuCalibration create_single_imu_model(
    const CalibrationMeasurements& all_cal_measurements,
    const ImuCalibrationMeasurements& imu_cal_measurements,
    const CreateSingleImuModelConfig& cfg = {});

}  // namespace estimation