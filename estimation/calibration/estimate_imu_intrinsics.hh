#pragma once

#include "eigen.hh"
#include "geometry/shapes/ellipse.hh"
#include "estimation/calibration/calibration_dataset.hh"

namespace estimation {

struct ImuIntrinsics {
  // A perfect IMU, if stationary, will measure "g" ~9.81 away from the
  // center of the earth. In other words, if the IMU were pointed in
  // every possible direction, one would see a sphere of radius g.
  //
  // An imperfect IMU will trace out an ellipsoid, instead of a sphere. We typically want
  // to deform this ellipse to a sphere of radius g.
  //
  geometry::shapes::Ellipse imu_gains;
  geometry::shapes::Ellipse magnetometer_gains;
};

class ImuModel {
 public:
  ImuModel() = default;
  ImuModel(const ImuIntrinsics& intrinsics) : intrinsics_(intrinsics) {
  }

  // Take an IMU observation, and transform it into a true acceleration
  jcc::Vec3 correct_measured_accel(const jcc::Vec3& observed_accel) const;

  jcc::Vec3 correct_measured_mag(const jcc::Vec3& raw_measurement) const;

  const ImuIntrinsics& intrinsics() const {
    return intrinsics_;
  }

 private:
  ImuIntrinsics intrinsics_;
};

ImuModel estimate_imu_intrinsics(const ImuCalibrationMeasurements& imu_meas);

}  // namespace estimation