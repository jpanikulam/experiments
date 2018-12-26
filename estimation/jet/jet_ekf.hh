#pragma once

#include "estimation/filter.hh"
#include "estimation/jet/jet_rk4.hh"
#include "estimation/observation_model.hh"

namespace estimation {
namespace jet_filter {

using JetEkf = Ekf<State>;
using ImuModel = ObservationModel<State, AccelMeasurement>;

struct FiducialMeasurement {
  int fiducial_id = -1;
  SE3 T_fiducial_from_camera;
  static constexpr int DIM = 6;
};

}  // namespace jet_filter
}  // namespace estimation