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
  TimePoint timestamp;
};

struct ImuCalibrationMeasurements {
  std::vector<TimedMeasurement<jet_filter::AccelMeasurement>> accel_meas;
  std::vector<TimedMeasurement<jet_filter::GyroMeasurement>> gyro_meas;
  std::vector<TimedMeasurement<MagnetometerMeasurement>> mag_meas;

  int imu_id;

  TimePoint first() const {
    return first_;
  }

  TimePoint last() const {
    return last_;
  }

  void set_first(const TimePoint& first) {
    first_ = first;
  }
  void set_last(const TimePoint& last) {
    last_ = last;
  }

 private:
  TimePoint first_;
  TimePoint last_;
};

struct CalibrationMeasurements {
  std::map<int, ImuCalibrationMeasurements> imu_cal;
  std::vector<TimedMeasurement<jet_filter::FiducialMeasurement>> fiducial_meas;

  TimePoint first() const {
    return fiducial_meas.front().timestamp;
  }

  TimePoint last() const {
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