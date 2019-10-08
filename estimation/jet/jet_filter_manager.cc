// TODO: Code-generate this
#include "estimation/jet/jet_filter_manager.hh"
#include "geometry/spatial/form_coordinate_frame.hh"

#include "estimation/jet/bootstrap_jet.hh"

namespace estimation {
namespace jet_filter {

constexpr int IMU_1_ID = 78;
constexpr int IMU_2_ID = 36;

FilterManager::FilterManager() {
  stage_ = FilterStage::STARTUP;
}

void FilterManager::measure_imu(const TimedMeasurement<AccelMeasurement>& meas,
                                int imu_id) {
  JASSERT(stage_ != FilterStage::STARTUP, "Must not be initializing");
  JASSERT_EQ(imu_id, IMU_1_ID, "IMU-2 Unsupported");
  const jcc::Vec3 corrected_accel = imu_model_from_id_.at(imu_id).correct_measured_accel(
      meas.measurement.observed_acceleration);

  const AccelMeasurement corrected_meas{.observed_acceleration = corrected_accel};

  imu_queue_.push_back({corrected_meas});
}
void FilterManager::measure_gyro(const TimedMeasurement<GyroMeasurement>& meas) {
  JASSERT(stage_ != FilterStage::STARTUP, "Must not be initializing");
  gyro_queue_.push_back({meas});
}
void FilterManager::measure_fiducial(const TimedMeasurement<FiducialMeasurement>& meas) {
  JASSERT(stage_ != FilterStage::STARTUP, "Must not be initializing");
  fiducial_queue_.push_back({meas});
}

void FilterManager::init(const FilterManagerConfiguration& config) {
  jcc::Warning() << "Starting up" << std::endl;

  transform_network_ = config.transform_network;
  imu_model_from_id_ = config.imu_model_from_id;
  max_fiducial_latency_ = jcc::to_duration(config.max_fiducial_latency_s);

  stage_ = FilterStage::BOOTSTRAPPING;
}

void FilterManager::bootstrap() {
  jcc::Warning() << "Bootstrapping" << std::endl;
  // TODO: Use the more sophisticated stuff
  const auto& last_imu = imu_queue_.back();
  const geometry::Unit3 g_imu_frame(last_imu.measurement.observed_acceleration);

  const auto& last_fiducial = fiducial_queue_.back();
  const SE3 fiducial_from_camera = last_fiducial.measurement.T_fiducial_from_camera;
  transform_network_.update_edge("fiducial", "camera", fiducial_from_camera);

  const auto bootstrap_result =
      bootstrap_jet(g_imu_frame, transform_network_, last_fiducial.timestamp);

  //
  // Initialize the Three Filter Tango
  //
  jf_primary_.set_parameters(bootstrap_result.parameters);
  jf_primary_.reset(bootstrap_result.xp0);
  jf_laggard_.set_parameters(bootstrap_result.parameters);
  jf_laggard_.reset(bootstrap_result.xp0);

  clear_queues();

  stage_ = FilterStage::RUNNING;
  jcc::Success() << "Done bootstrapping" << std::endl;
  jcc::Success() << "Running..." << std::endl;
}

void FilterManager::run() {
  const auto min_time = jf_primary_.state().time_of_validity;
  int num_dropped = 0;
  for (const auto& gyro_meas : gyro_queue_) {
    if (gyro_meas.timestamp < min_time) {
      continue;
    }

    jf_primary_.measure_gyro(gyro_meas.measurement, gyro_meas.timestamp);
    jf_laggard_.measure_gyro(gyro_meas.measurement, gyro_meas.timestamp);
  }
  if (num_dropped > 0) {
    jcc::Warning() << "Dropping " << num_dropped << " old gyro measurements" << std::endl;
  }

  gyro_queue_.clear();

  for (const auto& fiducial_meas : fiducial_queue_) {
    jf_laggard_.measure_fiducial(fiducial_meas.measurement, fiducial_meas.timestamp);

    if (fiducial_meas.timestamp < jf_primary_.state().time_of_validity) {
      std::cout << "Last fiducial measurement age 1: "
                <<jcc::to_seconds(jf_primary_.state().time_of_validity -
                              fiducial_meas.timestamp)
                << std::endl;

      // Get ready to catch up! (Swapping filters!)
      jf_primary_ = jf_laggard_;
    }
  }

  if (!fiducial_queue_.empty()) {
    transform_network_.update_edge(
        "fiducial", "camera", fiducial_queue_.back().measurement.T_fiducial_from_camera);
  }

  clear_queues();

  jf_primary_.free_run();
  jf_laggard_.run_until(jf_primary_.state().time_of_validity - max_fiducial_latency_);
}

void FilterManager::update(const jcc::TimePoint& current_time) {
  switch (stage_) {
    case FilterStage::UNINITIALIZED: {
      JASSERT(false, "Filter state must never be UNINITIALIZED");
      break;
    }
    case FilterStage::STARTUP: {
      JASSERT(false, "Filter must not be in STARTUP at this stage");
      break;
    }
    case FilterStage::BOOTSTRAPPING: {
      if (!fiducial_queue_.empty() && imu_queue_.size() > 50) {
        bootstrap();
      }
      break;
    }
    case FilterStage::RUNNING: {
      run();
      break;
    }
  }
}

JetFilter::JetFilterState FilterManager::state(const jcc::TimePoint& current_time) const {
  JASSERT(stage_ == FilterStage::RUNNING, "Filter must be in RUNNING at this stage");
  const auto state = jf_primary_.view(current_time);
  return state;
}

}  // namespace jet_filter
}  // namespace estimation
