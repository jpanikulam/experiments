#pragma once

#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/jet/jet_filter.hh"
#include "geometry/kinematics/transform_network.hh"

#include <deque>
#include <memory>
#include <queue>

namespace estimation {
namespace jet_filter {

enum class FilterStage {
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

inline std::string stage_to_string(const FilterStage stage) {
  switch (stage) {
    case FilterStage::UNINITIALIZED:
      return "UNINITIALIZED";
      break;
    case FilterStage::STARTUP:
      return "STARTUP";
      break;
    case FilterStage::BOOTSTRAPPING:
      return "BOOTSTRAPPING";
      break;
    case FilterStage::RUNNING:
      return "RUNNING";
      break;
  }
  return "Unknonw";
}

struct FilterManagerConfiguration {
  std::map<int, ImuModel> imu_model_from_id;
  geometry::TransformNetwork transform_network;

  double max_fiducial_latency_s = 0.4;
};

class FilterManager {
 public:
  FilterManager();

  void update(const jcc::TimePoint& current_time);

  // Jet state
  JetFilter::JetFilterState state(const jcc::TimePoint& current_time) const;

  // State machine state
  FilterStage stage() const {
    return stage_;
  }
  void measure_imu(const TimedMeasurement<AccelMeasurement>& meas, int imu_id);
  void measure_gyro(const TimedMeasurement<GyroMeasurement>& meas);
  void measure_fiducial(const TimedMeasurement<FiducialMeasurement>& meas);
  void init(const FilterManagerConfiguration& filter_manager_cfg);

 private:
  void bootstrap();
  void run();

  geometry::TransformNetwork transform_network_;

  jcc::TimeDuration max_fiducial_latency_;

  JetFilter jf_primary_;
  JetFilter jf_laggard_;

  // Each queue must be strictly increasing in time
  std::deque<TimedMeasurement<AccelMeasurement>> imu_queue_;
  std::deque<TimedMeasurement<GyroMeasurement>> gyro_queue_;
  std::deque<TimedMeasurement<FiducialMeasurement>> fiducial_queue_;

  void clear_queues() {
    imu_queue_.clear();
    gyro_queue_.clear();
    fiducial_queue_.clear();
  }

  std::map<int, ImuModel> imu_model_from_id_;

  FilterStage stage_ = FilterStage::STARTUP;
};

}  // namespace jet_filter
}  // namespace estimation
