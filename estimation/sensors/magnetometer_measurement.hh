#pragma once

namespace estimation {

struct MagnetometerMeasurement {
  jcc::Vec3 observed_bfield;
  static constexpr int DIM = 3;
};

}  // namespace estimation
