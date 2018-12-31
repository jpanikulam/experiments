#pragma once

#include "estimation/jet/jet_ekf.hh"

namespace estimation {
namespace jet_filter {

// TODO generate this
FiducialMeasurement observe_fiducial(const State& x, const Parameters& p);

// TODO: generate this
jcc::Vec3 accel_error_model(const State& x,
                            const AccelMeasurement& z,
                            const Parameters& p);

// TODO: generate this
VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p);

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  JetFilter(const JetFilterState& xp0);

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t);

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t);

  void free_run();

  const Parameters& parameters() const {
    return parameters_;
  }

  const JetFilterState& state() const {
    return xp_;
  }

 private:
  JetFilterState xp_;
  JetEkf ekf_;
  Parameters parameters_;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
  };

}  // namespace jet_filter
}  // namespace jet_filter
