#pragma once

#include "estimation/jet/jet_ekf.hh"

namespace estimation {
namespace jet_filter {

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  JetFilter(const JetFilterState& xp0);

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t);

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t);

  void free_run();

  const JetFilterState& state() const {
    return xp_;
  }

 private:
  JetFilterState xp_;
  JetEkf ekf_;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
};

}  // namespace jet_filter
}  // namespace estimation
