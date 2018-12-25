#include "estimation/jet/jet_ekf.hh"
#include "estimation/jet/jet_rk4.hh"

#include "sophus.hh"

namespace estimation {
namespace jet_filter {

struct FiducialMeasurement {
  int fiducial_id = -1;
  SE3 fiducial_pose;
};

State dynamics(const State& x, const double h) {
  const Parameters z = {};
  return rk4_integrate(x, z, h);
}

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  JetFilter() : ekf_(dynamics) {
    const auto error_model = [](const State& x, const AccelMeasurement& z) -> jcc::Vec3 {
      const jcc::Vec3 g(0.0, 0.0, -9.81);
      const SE3 vehicle_from_sensor;

      const jcc::Vec3 expected_a_mpss = observe_accel(x.T_body_from_world, x.eps_dot,
                                                      x.eps_ddot, vehicle_from_sensor, g);
      const jcc::Vec3 error = expected_a_mpss - z.observed_acceleration;

      return error;
    };
    const ImuModel accel_model(error_model);
    // using ObservationFunction =
    // std::function<Update(const JetFilterState&, const AccelMeasurement&)>;
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
    xp_.P.setZero();
    xp_ = ekf_.service_all_measurements(xp_);
  }

 private:
  void observe_imu() const;
  void observe_fiducial() const;

  // void handle(int i) {
  //   switch (i) {
  //     case 0:
  //       const FilterStateUpdate update = observe_imu(x, imu_measurements_.top());
  //       imu_measurements_.pop();
  //       break;
  //     case 1:
  //       const FilterStateUpdate update =
  //           observe_fiducial(x, fiducial_measurements_.top());
  //       fiducial_measurements_.pop();
  //       break;
  //   }
  // }

 private:
  JetEkf ekf_;

  JetFilterState xp_;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
};

void go() {
  const jcc::Vec3 g(0.0, 0.0, 0.0);
  const SO3 r_vehicle_from_sensor;
  const jcc::Vec3 t_vehicle_from_sensor = jcc::Vec3(0.0, 0.0, 1.0);
  const SE3 vehicle_from_sensor = SE3(r_vehicle_from_sensor, t_vehicle_from_sensor);

  State x;
  // x.eps_dot = VecNd<6>::Ones();
  x.eps_dot[0] = 1.0;
  x.eps_dot[3] = 0.0;
  x.eps_dot[4] = 0.0;
  x.eps_dot[5] = 1.0;

  x.eps_ddot[2] = 0.0;
  x.eps_ddot[5] = 0.0;

  const auto res =
      observe_accel(x.T_body_from_world, x.eps_dot, x.eps_ddot, vehicle_from_sensor, g);
  std::cout << res.transpose() << std::endl;
  // JetFilter jf;
  //   AccelMeasurement meas;
  //   TimePoint start_time = {};
  //   for (int k = 0; k < 10; ++k) {
  //     const TimePoint obs_time = to_duration(k * 0.1) + start_time;
  //     jf.measure_imu(meas, obs_time);
  //   }
  //   jf.free_run();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::go();
}