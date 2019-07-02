#pragma once

#include "eigen.hh"

#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/calibration/calibration_dataset.hh"
#include "geometry/spatial/time_interpolator.hh"

#include <vector>

namespace estimation {

geometry::spatial::TimeInterpolator make_accel_interpolator(
    const std::vector<calibration::TimedMeasurement<jet_filter::AccelMeasurement>>& accel_meas,
    const calibration::ImuModel& imu_model);

}  // namespace estimation