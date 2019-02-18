#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "estimation/jet/jet_filter.hh"
#include "estimation/jet/jet_optimizer.hh"

#include "estimation/jet/jet_rk4.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

Parameters mock_parameters() {
  Parameters p;
  // const jcc::Vec3 g(0.0, 0.0, -1.3);
  // p.g_world = g;

  // const jcc::Vec3 t(0.1, 0.5, 0.1);
  const jcc::Vec3 t(0.0, 0.1, 0.25);
  // const jcc::Vec3 t = jcc::Vec3::Zero();
  const SO3 r_vehicle_from_sensor = SO3::exp(jcc::Vec3(0.5 * M_PI, 0.0, 0.1));
  // const SO3 r_vehicle_from_sensor =
      // SO3::exp(jcc::Vec3(-3.10341, -0.0387912, -0.00578994));

  const SE3 sensor_from_body(r_vehicle_from_sensor, t);
  p.T_imu_from_vehicle = sensor_from_body;
  return p;
}

void setup() {
  std::cout << "Getting view" << std::endl;
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);

  std::cout << "Adding primitive" << std::endl;
  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void draw_states(viewer::SimpleGeometry& geo,
                 const std::vector<JetOptimizer::StateObservation>& states,
                 bool truth) {
  const int n_states = static_cast<int>(states.size());
  for (int k = 0; k < n_states; ++k) {
    auto& state = states.at(k).x;
    const SE3 T_world_from_body = get_world_from_body(state);
    if (truth) {
      geo.add_axes({T_world_from_body, 0.1});
    } else {
      geo.add_axes({T_world_from_body, 0.05, 2.0, true});

      if (k < n_states - 1) {
        const auto& next_state = states.at(k + 1).x;
        const SE3 T_world_from_body_next = get_world_from_body(next_state);
        geo.add_line(
            {T_world_from_body.translation(), T_world_from_body_next.translation()});
      }
    }
  }
}

void print_state(const State& x) {
  std::cout << "\teps_dot   : " << x.eps_dot.transpose() << std::endl;
  std::cout << "\teps_ddot  : " << x.eps_ddot.transpose() << std::endl;
  std::cout << "\taccel_bias: " << x.accel_bias.transpose() << std::endl;
  std::cout << "\tgyro_bias : " << x.gyro_bias.transpose() << std::endl;
  // const auto res = observe_accel(xp.x, true_params);
  // std::cout << "\tExpected Measurement: " << res.transpose() << std::endl;
}

}  // namespace

void run_filter() {
  std::cout << "Starting filter simulation" << std::endl;
  const Parameters true_params = mock_parameters();

  JetOptimizer jet_opt;

  FilterState<State> xp0 = JetFilter::reasonable_initial_state();

  xp0.time_of_validity = {};

  State true_x = xp0.x;
  true_x.eps_dot[4] = 0.1;

  true_x.eps_dot[0] = 1.0;
  true_x.eps_dot[4] = -0.6;
  true_x.eps_dot[5] = -0.2;

  // true_x.accel_bias = jcc::Vec3(0.1, -0.05, 0.15);
  true_x.accel_bias = jcc::Vec3::Zero();

  true_x.eps_ddot[1] = 0.01;
  true_x.eps_ddot[5] = 0.01;
  true_x.eps_ddot[3] = 0.02;

  JetFilter jf(xp0);

  setup();
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  const auto obs_geo = view->add_primitive<viewer::SimpleGeometry>();
  constexpr double sphere_size_m = 0.05;

  std::vector<JetOptimizer::StateObservation> ground_truth;
  std::vector<JetOptimizer::StateObservation> est_states;

  TimePoint start_time = xp0.time_of_validity;
  constexpr auto dt = to_duration(0.1);

  constexpr int NUM_SIM_STEPS = 500;

  TimePoint current_time = start_time;

  constexpr bool ACCEL_OBS = true;
  constexpr bool GYRO_OBS = false;
  constexpr bool FIDUCIAL_OBS = true;

  for (int k = 0; k < NUM_SIM_STEPS; ++k) {
    if (k > 75 && k < 130) {
      true_x.eps_ddot[0] = 0.1;
      // true_x.eps_ddot[1] = -0.02;
      // true_x.eps_ddot[2] = -0.03;

      true_x.eps_ddot[3] = 0.1;
      true_x.eps_ddot[4] = 0.05;
      true_x.eps_ddot[5] = -0.01;
    } else if (k > 130) {
      true_x.eps_ddot.setZero();
    }

    //
    // Accelerometer Observation
    //
    // if ((k % 10 == 0) && k > 75) {
    if (ACCEL_OBS && k > 1) {
      ground_truth.push_back({true_x, current_time});
      const AccelMeasurement imu_meas = observe_accel(true_x, true_params);
      std::cout << "Accel: " << imu_meas.observed_acceleration.transpose() << std::endl;

      jf.measure_imu(imu_meas, current_time);
      jet_opt.measure_imu(imu_meas, current_time);

      jf.free_run();
      // est_states.push_back({jf.state().x, current_time});
      // assert(jf.state().time_of_validity == current_time);

      const jcc::Vec4 accel_color(0.0, 0.7, 0.7, 0.8);
      obs_geo->add_sphere(
          {get_world_from_body(jf.state().x).translation(), sphere_size_m, accel_color});

      const SE3 world_from_body = get_world_from_body(jf.state().x);
      const jcc::Vec3 observed_accel_world_frame =
          (world_from_body.so3() * true_params.T_imu_from_vehicle.so3().inverse() *
           imu_meas.observed_acceleration);

      obs_geo->add_line(
          {world_from_body.translation(),
           world_from_body.translation() + (0.25 * observed_accel_world_frame),
           accel_color});
    }

    //
    // Gyro Observation
    //
    if (GYRO_OBS) {
      constexpr auto dt2 = to_duration(0.01);
      true_x = rk4_integrate(true_x, true_params, to_seconds(dt2));
      current_time += dt2;
      ground_truth.push_back({true_x, current_time});

      const GyroMeasurement gyro_meas = observe_gyro(true_x, true_params);
      std::cout << "Gyro: " << gyro_meas.observed_w.transpose() << std::endl;

      jf.measure_gyro(gyro_meas, current_time);
      jet_opt.measure_gyro(gyro_meas, current_time);

      jf.free_run();
      est_states.push_back({jf.state().x, current_time});
      assert(jf.state().time_of_validity == current_time);

      const jcc::Vec4 gyro_color(0.7, 0.1, 0.7, 0.8);
      obs_geo->add_sphere(
          {get_world_from_body(jf.state().x).translation(), sphere_size_m, gyro_color});

      const SE3 world_from_body = get_world_from_body(jf.state().x);
      const jcc::Vec3 observed_w_world_frame =
          (world_from_body.so3() * true_params.T_imu_from_vehicle.so3().inverse() *
           gyro_meas.observed_w);

      obs_geo->add_line({world_from_body.translation(),
                         world_from_body.translation() + (0.25 * observed_w_world_frame),
                         gyro_color});
    }

    //
    // Fiducial Measurement
    //

    if (FIDUCIAL_OBS && ((k % 1) == 0)) {
      constexpr auto dt2 = to_duration(0.05);
      true_x = rk4_integrate(true_x, true_params, to_seconds(dt2));
      current_time += dt2;

      ground_truth.push_back({true_x, current_time});
      const FiducialMeasurement fiducial_meas = observe_fiducial(true_x, true_params);

      jf.measure_fiducial(fiducial_meas, current_time);
      jet_opt.measure_fiducial(fiducial_meas, current_time);

      jf.free_run();
      est_states.push_back({jf.state().x, current_time});
      assert(jf.state().time_of_validity == current_time);
      obs_geo->add_sphere({get_world_from_body(jf.state().x).translation(), sphere_size_m,
                           jcc::Vec4(1.0, 0.0, 0.0, 1.0)});
    }

    //
    // Printout
    //
    {
      const auto xp = jf.state();
      // std::cout << "k: " << k << std::endl;
      // std::cout << "(Est)" << std::endl;
      // print_state(xp.x);

      // std::cout << "(True)" << std::endl;
      // print_state(true_x);

      // obs_geo->add_axes({get_world_from_body(true_x)});
      // obs_geo->flush();
      // view->spin_until_step();
    }
    true_x = rk4_integrate(true_x, true_params, to_seconds(dt));
    current_time += dt;
  }

  // Do the optimization

  obs_geo->flip();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  draw_states(*geo, ground_truth, true);
  geo->flip();

  //
  // Create solver visitor
  //

  const auto visitor_geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto visitor = [&view, &visitor_geo](const JetPoseOptimizer::Solution& soln) {
    visitor_geo->clear();
    draw_states(*visitor_geo, soln.x, false);
    visitor_geo->flip();
    std::cout << "\tOptimized T_imu_from_vehicle: "
              << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
              << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
    view->spin_until_step();
  };

  const auto solution = jet_opt.solve(est_states, jf.parameters(), visitor);
  // const auto solution = jet_opt.solve(est_states, mock_parameters(), visitor);

  for (std::size_t k = 0; k < solution.x.size(); ++k) {
    const auto& state = solution.x.at(k).x;
    std::cout << "----- " << k << std::endl;
    print_state(state);
    std::cout << "\\\\\\\\\\\\" << std::endl;
    print_state(ground_truth.at(k).x);
  }
  std::cout << "Optimized T_imu_from_vehicle: "
            << solution.p.T_imu_from_vehicle.translation().transpose() << "; "
            << solution.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;

  draw_states(*geo, solution.x, false);

  view->spin_until_step();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  // estimation::jet_filter::go();
  estimation::jet_filter::run_filter();
}