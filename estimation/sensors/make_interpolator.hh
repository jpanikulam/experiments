#pragma once

#include "eigen.hh"

#include "estimation/calibration/calibration_dataset.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "geometry/spatial/time_interpolator.hh"

#include <vector>

namespace estimation {

geometry::spatial::TimeInterpolator make_accel_interpolator(
    const std::vector<TimedMeasurement<jet_filter::AccelMeasurement>>&
        accel_meas,
    const ImuModel& imu_model);

geometry::spatial::TimeInterpolator make_gyro_interpolator(
    const std::vector<TimedMeasurement<jet_filter::GyroMeasurement>>&
        gyro_meas);
}  // namespace estimation