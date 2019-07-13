// TODO: Code-generate this
#include "estimation/jet/jet_filter_manager.hh"
#include "geometry/spatial/form_coordinate_frame.hh"

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
  max_fiducial_latency_ = estimation::to_duration(config.max_fiducial_latency_s);

  stage_ = FilterStage::BOOTSTRAPPING;
}

namespace {
struct BootstrapResult {
  JetFilter::JetFilterState xp0;
  Parameters parameters;
};

Parameters construct_jet_parameters(const geometry::TransformNetwork& tfn) {
  auto p = JetFilter::reasonable_parameters();
  {
    p.T_world_from_fiducial = tfn.find_source_from_destination("world", "fiducial");
    p.T_imu1_from_vehicle = tfn.find_source_from_destination("imu_78", "vehicle");
    p.T_imu2_from_vehicle = tfn.find_source_from_destination("imu_36", "vehicle");
    p.T_camera_from_vehicle = tfn.find_source_from_destination("camera", "vehicle");
  }

  return p;
}

//
// Setup parameters
//  - Get fiducial_from_world
//  - Load camera/imu* <- vehicle
//
// Establish world-frame
//  - Establish initial pose
//
BootstrapResult bootstrap_jet(const geometry::Unit3& g_imu_frame,
                              const geometry::TransformNetwork& tfn,
                              const TimePoint& t0) {
  BootstrapResult result;
  const geometry::Unit3 g_vehicle_frame =
      tfn.transform_a_from_b("vehicle", "imu_78", g_imu_frame);

  const SE3 vehicle_from_fiducial =
      tfn.find_source_from_destination("vehicle", "fiducial");

  const jcc::Vec3 fiducial_vehicle_frame = vehicle_from_fiducial.translation();
  const SO3 world_from_vehicle = geometry::spatial::form_coordinate_frame_from_zhat_and_x(
      g_vehicle_frame, fiducial_vehicle_frame);

  geometry::TransformNetwork tfn_tmp = tfn;
  tfn_tmp.add_edge("world", "vehicle", SE3(world_from_vehicle, jcc::Vec3::Zero()));

  const auto parameters = construct_jet_parameters(tfn_tmp);

  auto xp0 = JetFilter::reasonable_initial_state(t0);
  {
    const SE3 world_from_vehicle =
        tfn_tmp.find_source_from_destination("world", "vehicle");

    xp0.x.R_world_from_body = world_from_vehicle.so3();
    xp0.x.x_world = world_from_vehicle.translation();
  }

  return BootstrapResult{.xp0 = xp0,  //
                         .parameters = parameters};
}
}  // namespace

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

  imu_queue_.clear();
  fiducial_queue_.clear();
  stage_ = FilterStage::RUNNING;
}

void FilterManager::run() {
  for (const auto& gyro_meas : gyro_queue_) {
    jf_primary_.measure_gyro(gyro_meas.measurement, gyro_meas.timestamp);
    jf_laggard_.measure_gyro(gyro_meas.measurement, gyro_meas.timestamp);
  }

  gyro_queue_.clear();

  for (const auto& fiducial_meas : fiducial_queue_) {
    jf_laggard_.measure_fiducial(fiducial_meas.measurement, fiducial_meas.timestamp);

    if (fiducial_meas.timestamp < jf_primary_.state().time_of_validity) {
      // Get ready to catch up! (Swapping filters!)
      jf_primary_ = jf_laggard_;
    }
  }

  if (!fiducial_queue_.empty()) {
    transform_network_.update_edge(
        "fiducial", "camera", fiducial_queue_.back().measurement.T_fiducial_from_camera);
  }
  fiducial_queue_.clear();

  jf_primary_.free_run();
  jf_laggard_.run_until(jf_primary_.state().time_of_validity - max_fiducial_latency_);
}

void FilterManager::update(const TimePoint& current_time) {
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

JetFilter::JetFilterState FilterManager::state(const TimePoint& current_time) const {
  JASSERT(stage_ == FilterStage::RUNNING, "Filter must be in RUNNING at this stage");
  const auto state = jf_primary_.view(current_time);
  return state;
}

}  // namespace jet_filter
}  // namespace estimation
