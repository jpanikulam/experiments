#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {

struct FiducialMeasurement {
  int fiducial_id = -1;
  SE3 fiducial_pose;
};

Parameters get_parameters() {
  const jcc::Vec3 g(0.0, 0.0, 0.0);
  const SE3 vehicle_from_sensor;
  Parameters p;
  p.T_sensor_from_body = vehicle_from_sensor;
  return p;
}

State dynamics(const State& x, const double h) {
  const Parameters p = get_parameters();
  return rk4_integrate(x, p, h);
}

jcc::Vec3 accel_error_model(const State& x, const AccelMeasurement& z) {
  const Parameters p = get_parameters();
  const jcc::Vec3 expected_a_mpss = observe_accel(x, p);
  const jcc::Vec3 error = z.observed_acceleration - expected_a_mpss;
  return error;
}

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  JetFilter(const JetFilterState& xp0) : xp_(xp0), ekf_(dynamics) {
    const ImuModel accel_model(accel_error_model);
    imu_id_ = ekf_.add_model(accel_model);

    // fiducial_id_ = ekf_.add_model(fiducial_model);
  }

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
    ekf_.measure(meas, t, imu_id_);
  }

  // void measure_fiducial(const FiducialMeasurement& meas) {
  // ekf_.measure(meas, fiducial_id_);
  // }

  void free_run() {
    xp_ = ekf_.service_all_measurements(xp_);
  }

  const JetFilterState& state() const {
    return xp_;
  }

 private:
  void observe_imu() const;
  void observe_fiducial() const;

 private:
  JetFilterState xp_;
  JetEkf ekf_;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
};

void go() {
  // const Parameters p = get_parameters();
  // const SO3 r_vehicle_from_sensor;
  // const jcc::Vec3 t_vehicle_from_sensor = jcc::Vec3(0.0, 0.0, 1.0);
  // const SE3 vehicle_from_sensor = SE3(r_vehicle_from_sensor, t_vehicle_from_sensor);

  // State x;
  // x.eps_ddot[1] = 1.0;
  //

  FilterState<State> xp0;
  xp0.P.setIdentity();
  xp0.x.eps_ddot[0] = -0.9;

  JetFilter jf(xp0);
  AccelMeasurement meas;
  meas.observed_acceleration[0] = 2.0;

  // return;

  TimePoint start_time = {};
  for (int k = 0; k < 10; ++k) {
    const TimePoint obs_time = to_duration(k * 0.5) + start_time;
    jf.measure_imu(meas, obs_time);
    jf.free_run();

    const auto xp = jf.state();

    std::cout << "Modelled Error: " << accel_error_model(xp.x, meas).transpose()
              << std::endl;
    const auto res = observe_accel(xp.x, get_parameters());
    std::cout << "Expected Accel: " << res.transpose() << std::endl;
  }
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::go();
}