#pragma once

#include "estimation/jet/fiducial_measurement.hh"
#include "estimation/jet/jet_rk4.hh"
#include "estimation/sensors/magnetometer_measurement.hh"
#include "util/time_point.hh"

#include <map>
#include <opencv2/opencv.hpp>

namespace estimation {

template <typename MeasurementType>
struct TimedMeasurement {
  MeasurementType measurement;
  jcc::TimePoint timestamp;
};

struct ImuCalibrationMeasurements {
  std::vector<TimedMeasurement<jet_filter::AccelMeasurement>> accel_meas;
  std::vector<TimedMeasurement<jet_filter::GyroMeasurement>> gyro_meas;
  std::vector<TimedMeasurement<MagnetometerMeasurement>> mag_meas;

  int imu_id;

  jcc::TimePoint first() const {
    return first_;
  }

  jcc::TimePoint last() const {
    return last_;
  }

  void set_first(const jcc::TimePoint& first) {
    first_ = first;
  }
  void set_last(const jcc::TimePoint& last) {
    last_ = last;
  }

 private:
  jcc::TimePoint first_;
  jcc::TimePoint last_;
};

struct CalibrationMeasurements {
  std::map<int, ImuCalibrationMeasurements> imu_cal;
  std::vector<TimedMeasurement<jet_filter::FiducialMeasurement>> fiducial_meas;

  jcc::TimePoint first() const {
    return fiducial_meas.front().timestamp;
  }

  jcc::TimePoint last() const {
    return fiducial_meas.back().timestamp;
  }
};

// Hate.
// Hate.
// Hate.
struct ImageMeasurement {
  jcc::TimePoint time;
  cv::Mat image;
  std::string serial_number;
};

}  // namespace estimation