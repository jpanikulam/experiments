#pragma once

#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/jet/jet_filter.hh"
#include "geometry/kinematics/transform_network.hh"

#include <deque>
#include <memory>
#include <queue>

namespace estimation {
namespace jet_filter {

enum class FilterSmState {
  // Do not occupy this state!
  UNINITIALIZED = 0,

  // During:
  //  - Load intrinsics
  // Exit:
  //   -> BOOTSTRAPPING
  //      Loaded intrinsics
  STARTUP = 1,

  // During:
  //  - Estimate gravity
  //  - Form the "World frame"
  //  - Run diagnostics
  // Exit:
  //   -> INITIALIZING
  //      All operations complete
  BOOTSTRAPPING,

  // During:
  //  - Choose an initial state
  // Exit:
  //   -> RUNNING
  //      Initial state is set
  // INITIALIZING,

  // During:
  //  - Recieve messages
  //  - Publish filter state
  // Exit:
  //   -> Crash
  //      Off-nominal behavior
  //   -> [TODO] BOOSTRAPPING
  //      Lost pose for a significant amount of time
  RUNNING,
};

struct FilterManagerConfiguration {
  std::map<int, ImuModel> imu_model_from_id;
  geometry::TransformNetwork transform_network;

  double max_fiducial_latency_s = 0.4;
};

class FilterManager {
 public:
  FilterManager();

  void update(const TimePoint& current_time);
  JetFilter::JetFilterState state(const TimePoint& current_time) const;

 private:
  void init(const FilterManagerConfiguration& filter_manager_cfg);
  void bootstrap();
  void run();

  void measure_imu(const TimedMeasurement<AccelMeasurement>& meas, int imu_id);
  void measure_gyro(const TimedMeasurement<GyroMeasurement>& meas);

  void measure_fiducial(const TimedMeasurement<FiducialMeasurement>& meas);

  geometry::TransformNetwork transform_network_;

  TimeDuration max_fiducial_latency_;

  JetFilter jf_primary_;
  JetFilter jf_laggard_;

  // Each queue must be strictly increasing in time
  std::deque<TimedMeasurement<AccelMeasurement>> imu_queue_;
  std::deque<TimedMeasurement<GyroMeasurement>> gyro_queue_;
  std::deque<TimedMeasurement<FiducialMeasurement>> fiducial_queue_;

  std::map<int, ImuModel> imu_model_from_id_;

  FilterSmState state_ = FilterSmState::STARTUP;
};

}  // namespace jet_filter
}  // namespace estimation
