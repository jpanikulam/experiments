#pragma once

#include "estimation/jet/fiducial_measurement.hh"
#include "estimation/jet/jet_rk4.hh"
#include "estimation/time_point.hh"

namespace estimation {
namespace calibration {

template <typename MeasurementType>
struct TimedMeasurement {
  MeasurementType measurement;
  TimePoint timestamp;
};

struct CalibrationMeasurements {
  std::vector<TimedMeasurement<jet_filter::AccelMeasurement>> accel_meas;
  std::vector<TimedMeasurement<jet_filter::GyroMeasurement>> gyro_meas;
  std::vector<TimedMeasurement<jet_filter::FiducialMeasurement>> fiducial_meas;
};

}  // namespace calibration
}  // namespace estimation