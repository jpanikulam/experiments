#pragma once

#include "estimation/calibration/calibrate_single_imu.hh"
#include "geometry/kinematics/transform_network.hh"

namespace estimation {
namespace jet {

struct JetModel {
  geometry::TransformNetwork transform_network;
  std::map<int, SingleImuCalibration> imu_calibration_from_imu_id;
};

std::string make_imu_id_string(const int imu_id);

JetModel create_static_jet_model(const CalibrationMeasurements& cal_data,
                                 const CreateSingleImuModelConfig& cal_imu_cfg);
}  // namespace jet
}  // namespace estimation