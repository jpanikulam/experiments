#include "estimation/calibration/warn_sensor_rates.hh"
#include "logging/assert.hh"

namespace estimation {

void warn_sensor_rates(
    const estimation::calibration::CalibrationMeasurements& cal_measurements) {
  for (const auto& imu : cal_measurements.imu_cal) {
    const auto& imu_meas = imu.second;

    std::stringstream imu_lead_ss;
    imu_lead_ss << "[IMU-" << imu_meas.imu_id << "]";
    const std::string imu_lead = imu_lead_ss.str();

    if (imu_meas.accel_meas.size() > 0) {
      const double total_time = estimation::to_seconds(
          imu_meas.accel_meas.back().timestamp - imu_meas.accel_meas.front().timestamp);
      const double avg_rate_hz = imu_meas.accel_meas.size() / total_time;
      if (avg_rate_hz < 80) {
        jcc::Error() << imu_lead << " Accel recording rate was very low: " << avg_rate_hz
                     << " Hz" << std::endl;
      } else if (avg_rate_hz < 100) {
        jcc::Warning() << imu_lead << " Accel recording rate was low: " << avg_rate_hz
                       << " Hz" << std::endl;
      } else {
        jcc::Success() << imu_lead << " Accel recording rate was good: " << avg_rate_hz
                       << " Hz" << std::endl;
      }
    } else {
      jcc::Error() << imu_lead << " No acceleration measurements" << std::endl;
    }

    if (imu_meas.gyro_meas.size() > 0) {
      const double total_time = estimation::to_seconds(
          imu_meas.gyro_meas.back().timestamp - imu_meas.gyro_meas.front().timestamp);
      const double avg_rate_hz = imu_meas.gyro_meas.size() / total_time;
      if (avg_rate_hz < 80) {
        jcc::Error() << imu_lead << " Gyro recording rate was very low: " << avg_rate_hz
                     << " Hz" << std::endl;
      } else if (avg_rate_hz < 100) {
        jcc::Warning() << imu_lead << " Gyro recording rate was low: " << avg_rate_hz
                       << " Hz" << std::endl;
      } else {
        jcc::Success() << imu_lead << " Gyro recording rate was good: " << avg_rate_hz
                       << " Hz" << std::endl;
      }
    } else {
      jcc::Error() << imu_lead << " No Gyro measurements" << std::endl;
    }

    if (imu_meas.mag_meas.size() > 0) {
      const double total_time = estimation::to_seconds(
          imu_meas.mag_meas.back().timestamp - imu_meas.mag_meas.front().timestamp);
      const double avg_rate_hz = imu_meas.mag_meas.size() / total_time;
      if (avg_rate_hz < 80) {
        jcc::Error() << imu_lead
                     << " Magnetometer recording rate was very low: " << avg_rate_hz
                     << " Hz" << std::endl;
      } else if (avg_rate_hz < 100) {
        jcc::Warning() << imu_lead
                       << " Magnetometer recording rate was low: " << avg_rate_hz << " Hz"
                       << std::endl;
      } else {
        jcc::Success() << imu_lead
                       << " Magnetometer recording rate was good: " << avg_rate_hz
                       << " Hz" << std::endl;
      }
    } else {
      jcc::Error() << imu_lead << " No Magnetometer measurements" << std::endl;
    }
  }
  if (cal_measurements.fiducial_meas.size() > 0) {
    const double total_time =
        estimation::to_seconds(cal_measurements.fiducial_meas.back().timestamp -
                               cal_measurements.fiducial_meas.front().timestamp);
    const double avg_rate_hz = cal_measurements.fiducial_meas.size() / total_time;
    if (avg_rate_hz < 6) {
      jcc::Error() << "[Camera] Fiducial recording rate was very low: " << avg_rate_hz
                   << " Hz" << std::endl;
    } else if (avg_rate_hz < 11) {
      jcc::Warning() << "[Camera] Fiducial recording rate was low: " << avg_rate_hz
                     << " Hz" << std::endl;
    } else {
      jcc::Success() << "[Camera] Fiducial recording rate was good: " << avg_rate_hz
                   << " Hz" << std::endl;
    }
  } else {
    jcc::Error() << "[Camera] No fiducial measurements" << std::endl;
  }
}
}  // namespace estimation