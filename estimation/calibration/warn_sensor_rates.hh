#pragma once

#include "estimation/calibration/calibration_dataset.hh"

namespace estimation {
void warn_sensor_rates(
    const estimation::CalibrationMeasurements& cal_measurements);
}  // namespace estimation