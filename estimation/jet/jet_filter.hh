#pragma once

#include "estimation/jet/jet_ekf.hh"

namespace estimation {
namespace jet_filter {

inline SE3 get_world_from_body(const State& x) {
  const SE3 T_world_from_body = SE3(x.R_world_from_body, x.x_world);
  return T_world_from_body;
}

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  static JetFilterState reasonable_initial_state(const TimePoint& t);
  static Parameters reasonable_parameters();

  JetFilter(const JetFilterState& xp, const Parameters& parameters);
  JetFilter(const Parameters& parameters);

  void reset(const JetFilterState& xp) {
    xp_ = xp;
    initialized_ = true;
  }

  bool initialized() const {
    return initialized_;
  }

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t, bool imu2 = false);
  void measure_gyro(const GyroMeasurement& meas, const TimePoint& t, bool imu2 = false);

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t);

  State free_run();
  State run_until(const TimePoint& t);
  jcc::Optional<State> next_measurement();

  State view(const TimePoint& t) const;
  State predict(const TimePoint& t) const;

  const Parameters& parameters() const {
    return parameters_;
  }

  void set_parameters(const Parameters& p) {
    parameters_ = p;
  }

  const JetFilterState& state() const {
    return xp_;
  }

 private:
  void setup_models();

  JetFilterState xp_;
  Parameters parameters_;
  JetEkf ekf_;

  bool initialized_ = false;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
  int gyro_id_ = -1;

  int imu_2_id_ = -1;
  int gyro_2_id_ = -1;
};

}  // namespace jet_filter
}  // namespace estimation
