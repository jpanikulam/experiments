#pragma once

#include "eigen.hh"
// For parametric ellipse
#include "geometry/shapes/ellipse.hh"

namespace estimation {
namespace calibration {

struct ImuIntrinsics {
  // A perfect IMU, if stationary, will measure "g" ~9.81 away from the
  // center of the earth. In other words, if the IMU were pointed in
  // every possible direction, one would see a sphere of radius g.
  //
  // An imperfect IMU will trace out an ellipsoid, instead of a sphere. We typically want
  // to deform this ellipse to a sphere of radius g.
  //
  geometry::shapes::Ellipse imu_gains;
};

class ImuModel {
 public:
  ImuModel(const ImuIntrinsics& intrinsics) : intrinsics_(intrinsics) {
  }

  // Take an IMU observation, and transform it into a true acceleration
  jcc::Vec3 apply_calibration(const jcc::Vec3& observed) const;

 private:
  ImuIntrinsics intrinsics_;
};

ImuModel estimate_imu_intrinsics(const std::vector<jcc::Vec3>& measurements);

}  // namespace calibration
}  // namespace estimation