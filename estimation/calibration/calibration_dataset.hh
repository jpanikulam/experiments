#pragma once

#include "estimation/jet/fiducial_measurement.hh"
#include "estimation/jet/jet_rk4.hh"
#include "estimation/sensors/magnetometer_measurement.hh"
#include "estimation/time_point.hh"

namespace estimation {
namespace calibration {

template <typename MeasurementType>
struct TimedMeasurement {
  MeasurementType measurement;
  TimePoint timestamp;
};

struct ImuCalibrationMeasurements {
  std::vector<TimedMeasurement<jet_filter::AccelMeasurement>> accel_meas;
  std::vector<TimedMeasurement<jet_filter::GyroMeasurement>> gyro_meas;
  std::vector<TimedMeasurement<MagnetometerMeasurement>> mag_meas;
}

struct CalibrationMeasurements {
  std::map<int, ImuCalibrationMeasurements> imu_cal;
  std::vector<TimedMeasurement<jet_filter::FiducialMeasurement>> fiducial_meas;

  estimation::TimePoint first() const {
    return fiducial_meas.front().timestamp;
  }

  estimation::TimePoint last() const {
    return fiducial_meas.back().timestamp;
  }
};

}  // namespace calibration
}  // namespace estimation