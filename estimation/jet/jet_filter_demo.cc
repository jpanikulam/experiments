#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "estimation/jet/jet_filter.hh"
#include "estimation/jet/jet_pose_opt.hh"

#include "estimation/jet/jet_rk4.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {
/*
void go() {
  setup();
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const jcc::Vec3 g(0.0, 0.0, 0.0);
  const SO3 r_sensor_from_body;
  const jcc::Vec3 t_sensor_from_body = jcc::Vec3(0.0, 0.0, 1.0);
  const SE3 sensor_from_body = SE3(r_sensor_from_body, t_sensor_from_body);

  State x;
  x.T_body_from_world = SE3(SO3(), jcc::Vec3(5.0, 0.0, 0.0));

  x.eps_dot[0] = 0.0;
  x.eps_dot[1] = 0.0;
  x.eps_dot[2] = 0.0;
  x.eps_dot[3] = 0.0;
  x.eps_dot[4] = 0.0;
  x.eps_dot[5] = 0.0;

  x.eps_ddot[0] = 0.0;
  x.eps_ddot[1] = 1.0;
  x.eps_ddot[2] = 0.0;
  x.eps_ddot[3] = 0.0;
  x.eps_ddot[4] = 0.0;
  x.eps_ddot[5] = 1.0;

  p = mock_parameters();
  const auto res = observe_accel(x, p);

  constexpr double dt = 0.01;
  while (!view->should_close()) {
    x = dynamics(x, dt);

    geo->add_axes({x.T_body_from_world.inverse()});
    const SE3 world_from_sensor = (sensor_from_body * x.T_body_from_world).inverse();
    geo->add_axes({world_from_sensor, 0.25});
    const auto res =
        observe_accel(x.T_body_from_world, x.eps_dot, x.eps_ddot, sensor_from_body, g);

    geo->add_ray(
        {world_from_sensor.translation(), world_from_sensor.so3() * res, res.norm()});

    geo->flip();

    view->spin_until_step();
  }
}
*/

Parameters mock_parameters() {
  const jcc::Vec3 g(0.0, 0.0, 4.1);
  const SE3 vehicle_from_sensor;
  Parameters p;
  p.T_sensor_from_body = vehicle_from_sensor;
  p.g_world = g;
  return p;
}

void setup() {
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void draw_states(viewer::SimpleGeometry& geo,
                 const std::vector<State>& states,
                 bool truth) {
  for (const auto& state : states) {
    const SE3 T_world_from_body = state.T_body_from_world.inverse();

    if (truth) {
      geo.add_axes({T_world_from_body, 0.5, 2.0, true});
    } else {
      geo.add_axes({T_world_from_body, 1.0});
    }
  }
}
}  // namespace

class JetOptimizer {
 public:
  JetOptimizer() {
    imu_id_ = pose_opt_.add_error_model<AccelMeasurement>(accel_error_model);
    fiducial_id_ = pose_opt_.add_error_model<FiducialMeasurement>(fiducial_error_model);
  }

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
    pose_opt_.add_measurement(meas, t, imu_id_);
  }

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t) {
    pose_opt_.add_measurement(meas, t, fiducial_id_);
  }

  JetPoseOptimizer::Solution solve(const std::vector<State> x, const Parameters& p) {
    return pose_opt_.solve({x, p});
  }

 private:
  JetPoseOptimizer pose_opt_{rk4_integrate};
  int imu_id_ = -1;
  int fiducial_id_ = -1;
};

void run_filter() {
  const Parameters true_params = mock_parameters();

  JetOptimizer jet_opt;

  FilterState<State> xp0;
  xp0.P.setIdentity();
  xp0.x.eps_ddot[0] = 0.0;

  State true_x = xp0.x;
  true_x.eps_dot[0] = 0.1;
  true_x.eps_dot[3] = -0.1;
  true_x.eps_dot[4] = 0.1;

  JetFilter jf(xp0);

  // AccelMeasurement imu_meas;
  // imu_meas.observed_acceleration[0] = 2.0;

  std::vector<State> ground_truth;
  std::vector<State> est_states;

  TimePoint start_time = {};
  constexpr auto dt = to_duration(0.5);

  constexpr int NUM_SIM_STEPS = 76;

  for (int k = 0; k < NUM_SIM_STEPS; ++k) {
    const TimePoint obs_time = (dt * k) + start_time;
    const AccelMeasurement imu_meas = {observe_accel(true_x, true_params)};
    std::cout << "Accel: " << imu_meas.observed_acceleration.transpose() << std::endl;

    jf.measure_imu(imu_meas, obs_time);
    jet_opt.measure_imu(imu_meas, obs_time);
    jf.free_run();
    est_states.push_back(jf.state().x);
    ground_truth.push_back(true_x);

    constexpr auto dt2 = to_duration(0.1);
    true_x = rk4_integrate(true_x, true_params, to_seconds(dt2));
    ground_truth.push_back(true_x);
    const FiducialMeasurement fiducial_meas = observe_fiducial(true_x, true_params);

    jf.measure_fiducial(fiducial_meas, obs_time + dt2);
    jet_opt.measure_fiducial(fiducial_meas, obs_time + dt2);
    jf.free_run();
    est_states.push_back(jf.state().x);

    const auto xp = jf.state();
    std::cout << "k: " << k << std::endl;
    std::cout << "\teps_dot   : " << xp.x.eps_dot.transpose() << std::endl;
    std::cout << "\teps_ddot  : " << xp.x.eps_ddot.transpose() << std::endl;
    std::cout << "\taccel_bias: " << xp.x.accel_bias.transpose() << std::endl;
    std::cout << "\tgyro_bias : " << xp.x.gyro_bias.transpose() << std::endl;

    std::cout << "\tModelled Error: "
              << accel_error_model(xp.x, imu_meas, true_params).transpose() << std::endl;
    const auto res = observe_accel(xp.x, mock_parameters());
    std::cout << "\tExpected Measurement: " << res.transpose() << std::endl;

    true_x = rk4_integrate(true_x, true_params, to_seconds(dt));
  }

  // Do the optimization

  // const auto solution = jet_opt.solve(ground_truth, {});
  std::vector<State> init(NUM_SIM_STEPS * 2, xp0.x);
  const auto solution = jet_opt.solve(init, {});
  std::cout << "Optimized g: " << solution.p.g_world.transpose() << std::endl;
  std::cout << "Optimized T_sensor_from_body: "
            << solution.p.T_sensor_from_body.log().transpose() << std::endl;

  setup();
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  draw_states(*geo, ground_truth, true);
  draw_states(*geo, solution.x, false);

  geo->flip();

  view->spin_until_step();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  // estimation::jet_filter::go();
  estimation::jet_filter::run_filter();
}