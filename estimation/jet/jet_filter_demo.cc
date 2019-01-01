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

Parameters mock_parameters() {
  Parameters p;
  const jcc::Vec3 g(0.0, 0.0, -1.3);
  p.g_world = g;

  // const jcc::Vec3 t(0.1, 0.5, 0.1);
  const jcc::Vec3 t(0.0, 0.1, 0.25);
  // const jcc::Vec3 t = jcc::Vec3::Zero();
  const SO3 r_vehicle_from_sensor = SO3::exp(jcc::Vec3(0.0, 0.0, 0.0));
  const SE3 sensor_from_body(r_vehicle_from_sensor, t);
  p.T_imu_from_vehicle = sensor_from_body;
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
  const int n_states = static_cast<int>(states.size());
  for (int k = 0; k < n_states; ++k) {
    auto& state = states.at(k);
    const SE3 T_world_from_body = state.T_body_from_world.inverse();
    if (truth) {
      geo.add_axes({T_world_from_body, 0.1});
    } else {
      geo.add_axes({T_world_from_body, 0.05, 2.0, true});

      if (k < n_states - 1) {
        const auto& next_state = states.at(k + 1);
        const SE3 T_world_from_body_next = next_state.T_body_from_world.inverse();
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

template <int dim, int row, int mat_size>
void set_diag_to_value(MatNd<mat_size, mat_size>& mat, double value) {
  mat.template block<dim, dim>(row, row) = (MatNd<dim, dim>::Identity() * value);
}
}  // namespace

class JetOptimizer {
 public:
  JetOptimizer() {
    MatNd<AccelMeasurement::DIM, AccelMeasurement::DIM> accel_cov;
    accel_cov.setZero();
    {
      set_diag_to_value<AccelMeasurementDelta::observed_acceleration_error_dim,
                        AccelMeasurementDelta::observed_acceleration_error_ind>(accel_cov,
                                                                                0.01);
    }

    MatNd<FiducialMeasurement::DIM, FiducialMeasurement::DIM> fiducial_cov;
    {
      fiducial_cov.block<3, 3>(0, 0) = MatNd<3, 3>::Identity() * 0.001;
      fiducial_cov.block<3, 3>(3, 3) = MatNd<3, 3>::Identity() * 0.0001;
    }
    MatNd<State::DIM, State::DIM> state_cov;
    {
      state_cov.setZero();
      set_diag_to_value<StateDelta::accel_bias_error_dim,
                        StateDelta::accel_bias_error_ind>(state_cov, 0.0001);
      set_diag_to_value<StateDelta::gyro_bias_error_dim, StateDelta::gyro_bias_error_ind>(
          state_cov, 0.0001);
      set_diag_to_value<StateDelta::eps_dot_error_dim, StateDelta::eps_dot_error_ind>(
          state_cov, 0.1);
      set_diag_to_value<StateDelta::eps_ddot_error_dim, StateDelta::eps_ddot_error_ind>(
          state_cov, 0.1);

      constexpr int T_error_dim = StateDelta::T_body_from_world_error_log_dim;
      constexpr int T_error_ind = StateDelta::T_body_from_world_error_log_ind;
      set_diag_to_value<T_error_dim, T_error_ind>(state_cov, 0.001);
    }

    const MatNd<GyroMeasurement::DIM, GyroMeasurement::DIM> gyro_cov = accel_cov;

    imu_id_ =
        pose_opt_.add_error_model<AccelMeasurement>(observe_accel_error_model, accel_cov);
    gyro_id_ =
        pose_opt_.add_error_model<GyroMeasurement>(observe_gyro_error_model, gyro_cov);

    fiducial_id_ = pose_opt_.add_error_model<FiducialMeasurement>(fiducial_error_model,
                                                                  fiducial_cov);
    pose_opt_.set_dynamics_cov(state_cov);
  }

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
    pose_opt_.add_measurement(meas, t, imu_id_);
  }

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t) {
    pose_opt_.add_measurement(meas, t, fiducial_id_);
  }

  void measure_gyro(const GyroMeasurement& meas, const TimePoint& t) {
    pose_opt_.add_measurement(meas, t, gyro_id_);
  }

  JetPoseOptimizer::Solution solve(const std::vector<State> x, const Parameters& p) {
    const auto view = viewer::get_window3d("Mr. Filter, filters");
    const auto geo = view->add_primitive<viewer::SimpleGeometry>();
    const auto visitor = [&view, &geo](const JetPoseOptimizer::Solution& soln) {
      geo->clear();
      draw_states(*geo, soln.x, false);
      geo->flip();
      std::cout << "\tOptimized g: " << soln.p.g_world.transpose() << std::endl;
      std::cout << "\tOptimized T_imu_from_vehicle: "
                << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
                << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
      view->spin_until_step();
    };

    return pose_opt_.solve({x, p}, visitor);
  }

 private:
  JetPoseOptimizer pose_opt_{rk4_integrate};
  int imu_id_ = -1;
  int gyro_id_ = -1;
  int fiducial_id_ = -1;
};

void run_filter() {
  const Parameters true_params = mock_parameters();

  JetOptimizer jet_opt;

  FilterState<State> xp0;
  {
    MatNd<State::DIM, State::DIM> state_cov;
    state_cov.setZero();
    set_diag_to_value<StateDelta::accel_bias_error_dim, StateDelta::accel_bias_error_ind>(
        state_cov, 0.0001);
    set_diag_to_value<StateDelta::gyro_bias_error_dim, StateDelta::gyro_bias_error_ind>(
        state_cov, 0.0001);
    set_diag_to_value<StateDelta::eps_dot_error_dim, StateDelta::eps_dot_error_ind>(
        state_cov, 0.01);
    set_diag_to_value<StateDelta::eps_ddot_error_dim, StateDelta::eps_ddot_error_ind>(
        state_cov, 0.1);

    xp0.P = state_cov;
  }
  // xp0.x.eps_dot[0] = 1.0;
  // xp0.x.eps_dot[4] = -0.6;
  // xp0.x.eps_dot[5] = -0.2;

  // xp0.x.eps_ddot[1] = 0.01;
  // xp0.x.eps_ddot[5] = 0.01;
  // xp0.x.eps_ddot[3] = 0.02;

  xp0.time_of_validity = {};

  State true_x = xp0.x;
  true_x.eps_dot[4] = 0.1;

  true_x.eps_dot[0] = 1.0;
  true_x.eps_dot[4] = -0.6;
  true_x.eps_dot[5] = -0.2;

  true_x.eps_ddot[1] = 0.01;
  true_x.eps_ddot[5] = 0.01;
  true_x.eps_ddot[3] = 0.02;

  JetFilter jf(xp0);

  // AccelMeasurement imu_meas;
  // imu_meas.observed_acceleration[0] = 2.0;

  setup();
  const auto view = viewer::get_window3d("Mr. Filter, filters");
  const auto obs_geo = view->add_primitive<viewer::SimpleGeometry>();
  constexpr double sphere_size_m = 0.05;

  std::vector<State> ground_truth;
  std::vector<State> est_states;

  TimePoint start_time = {};
  constexpr auto dt = to_duration(0.1);

  constexpr int NUM_SIM_STEPS = 300;

  TimePoint current_time = start_time;

  for (int k = 0; k < NUM_SIM_STEPS; ++k) {
    if (k > 75 && k < 130) {
      // true_x.eps_ddot[0] = 0.1;
      // true_x.eps_ddot[1] = -0.02;
      // true_x.eps_ddot[2] = -0.03;

      // true_x.eps_ddot[3] = 0.1;
      // true_x.eps_ddot[4] = 0.05;
      // true_x.eps_ddot[5] = -0.01;
    } else if (k > 130) {
      // true_x.eps_ddot.setZero();
    }

    //
    // Accelerometer Observation
    //
    // if ((k % 10 == 0) && k > 75) {
    if (true) {
      ground_truth.push_back(true_x);
      const AccelMeasurement imu_meas = observe_accel(true_x, true_params);
      std::cout << "Accel: " << imu_meas.observed_acceleration.transpose() << std::endl;

      jf.measure_imu(imu_meas, current_time);
      jet_opt.measure_imu(imu_meas, current_time);

      jf.free_run();
      est_states.push_back(jf.state().x);
      assert(jf.state().time_of_validity == current_time);

      const jcc::Vec4 accel_color(0.0, 0.7, 0.7, 0.8);
      obs_geo->add_sphere({jf.state().x.T_body_from_world.inverse().translation(),
                           sphere_size_m, accel_color});

      const SE3 world_from_body = jf.state().x.T_body_from_world.inverse();
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
    if (true) {
      constexpr auto dt2 = to_duration(0.01);
      true_x = rk4_integrate(true_x, true_params, to_seconds(dt2));
      current_time += dt2;
      ground_truth.push_back(true_x);

      const GyroMeasurement gyro_meas = observe_gyro(true_x, true_params);
      std::cout << "Gyro: " << gyro_meas.observed_w.transpose() << std::endl;

      jf.measure_gyro(gyro_meas, current_time);
      jet_opt.measure_gyro(gyro_meas, current_time);

      jf.free_run();
      est_states.push_back(jf.state().x);
      assert(jf.state().time_of_validity == current_time);

      const jcc::Vec4 gyro_color(0.7, 0.1, 0.7, 0.8);
      obs_geo->add_sphere({jf.state().x.T_body_from_world.inverse().translation(),
                           sphere_size_m, gyro_color});

      const SE3 world_from_body = jf.state().x.T_body_from_world.inverse();
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

    {
      constexpr auto dt2 = to_duration(0.05);
      true_x = rk4_integrate(true_x, true_params, to_seconds(dt2));
      current_time += dt2;

      ground_truth.push_back(true_x);
      const FiducialMeasurement fiducial_meas = observe_fiducial(true_x, true_params);

      jf.measure_fiducial(fiducial_meas, current_time);
      jet_opt.measure_fiducial(fiducial_meas, current_time);

      jf.free_run();
      est_states.push_back(jf.state().x);
      assert(jf.state().time_of_validity == current_time);
      obs_geo->add_sphere({jf.state().x.T_body_from_world.inverse().translation(),
                           sphere_size_m, jcc::Vec4(1.0, 0.0, 0.0, 1.0)});
    }

    //
    // Printout
    //
    {
      const auto xp = jf.state();
      std::cout << "k: " << k << std::endl;
      print_state(xp.x);
    }
    true_x = rk4_integrate(true_x, true_params, to_seconds(dt));
    current_time += dt;
  }

  // Do the optimization

  obs_geo->flip();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  draw_states(*geo, ground_truth, true);
  geo->flip();

  // const std::vector<State> crap_init(est_states.size(), xp0.x);
  // const auto solution = jet_opt.solve(crap_init, jf.parameters());
  const auto solution = jet_opt.solve(est_states, jf.parameters());
  // const auto solution = jet_opt.solve(ground_truth, jf.parameters());

  for (std::size_t k = 0; k < solution.x.size(); ++k) {
    const auto& state = solution.x.at(k);
    std::cout << "----- " << k << std::endl;
    print_state(state);
    std::cout << "\\\\\\\\\\\\" << std::endl;
    print_state(ground_truth.at(k));
  }
  std::cout << "Optimized g: " << solution.p.g_world.transpose() << std::endl;
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