#include "estimation/jet/create_static_jet_model.hh"

namespace estimation {
namespace jet {

std::string make_imu_id_string(const int imu_id) {
  const std::string imu_id_str = "imu_" + std::to_string(imu_id);
  return imu_id_str;
}

JetModel create_static_jet_model(const CalibrationMeasurements& cal_measurements,
                                 const CreateSingleImuModelConfig& imu_cal_cfg) {
  JetModel jet_model;

  {
    const SO3 R_vehicle_from_camera = SO3::exp(jcc::Vec3::UnitY() * M_PI * 0.5) *
                                      SO3::exp(jcc::Vec3::UnitZ() * M_PI * 0.5);
    const SE3 vehicle_from_camera(R_vehicle_from_camera, jcc::Vec3(0.11, 0.0, 0.32));
    jet_model.transform_network.add_edge("vehicle", "camera", vehicle_from_camera);
  }

  for (const auto& imu_measurements : cal_measurements.imu_cal) {
    const auto one_imu = estimation::create_single_imu_model(
        cal_measurements, imu_measurements.second, imu_cal_cfg);
    jet_model.imu_calibration_from_imu_id[imu_measurements.first] = one_imu;

    const std::string id_string = make_imu_id_string(imu_measurements.first);
    if (id_string == "imu_78") {
      jet_model.transform_network.add_edge(
          "camera", "imu_78",
          SE3(one_imu.camera_from_gyro, jcc::Vec3(0.06, -0.14, -0.235)));
    }
    if (id_string == "imu_36") {
      jet_model.transform_network.add_edge(
          "camera", "imu_36", SE3(one_imu.camera_from_gyro, jcc::Vec3(0.0, -0.020, 0.0)));
    }
  }
  return jet_model;
}

}  // namespace jet
}  // namespace estimation